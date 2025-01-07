import sys  
import os 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from SMrTa.MRTASolver import MRTASolver, Robot
from SMrTa.MRTASolver.objects import Task
from Vision import Vision 
import threading
import json
import networkx as nx
# from RobotController import RobotController

class Coordinator:
    def __init__(self):
        video = "img/mazewmarkers.png"
        self.vg = Vision()
        self.vg.callback = self.run_solver
        threading.Thread(target=self.vg.run, daemon=True).start()

    def run_solver(self):
        print("Running solver...")
        robots, waypoints = self.vg.detect_markers(self.vg.frame)

        with open('devices.json', 'r') as f:
            robot_configs = json.load(f)['devices']
        # self.robots = {f"robot {i+1}": RobotController( robot['name'], robot['address'],robot['write_uuid']) for i, robot in enumerate(robots)}


        agents = [Robot(id=f"robot {i+1}", start=pos) for i, pos in enumerate([robots[i] for i in range(len(robots))])]

        # Tasks in config file defined with start/end locations given as marker ids, mapped to grid coordinates by vg.detect_markers in 'waypoints'
        with open('task_config.json', 'r') as file:
            task_config = json.load(file)
        self.tasks = [Task(id=i, start=waypoints[int(task['start'])], end=waypoints[int(task['end'])], deadline=task['deadline']) for i, task in enumerate(task_config['tasks'])]
        self.action_points = [robots[i] for i in range(len(robots))]
        for i, task in enumerate(self.tasks):
            if waypoints[task_config['tasks'][i]['start']] not in self.action_points:
                self.action_points.append(waypoints[task_config['tasks'][i]['start']])
            if waypoints[task_config['tasks'][i]['end']] not in self.action_points:
                self.action_points.append(waypoints[task_config['tasks'][i]['end']])

        # Remap agent and task start/end values to the index of the corresponding action_point [0, len(action_points)-1], leaving self.action_points containing the grid coordinates of the markers
        print("Agents:")
        for a in agents:
            print(a.id, a.start)
            a.start = self.action_points.index(a.start)
        print("Tasks:")
        for t in self.tasks:
            print(t.id, t.start, t.end)
            t.start = self.action_points.index(t.start)
            t.end = self.action_points.index(t.end)

        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV', 
            agents=agents,
            tasks_stream=[[self.tasks, 0]],
            room_graph=self.vg.create_travel_time_matrix(self.action_points),
            capacity=1,
            num_aps=len(self.action_points),
            aps_list=[len(self.action_points)],
            fidelity=1,
        )
        if solver.sol is None:
            print("No solution found!")
            return None
        
        schedules = self.convert_solution_to_schedules(solver.sol)
        print(schedules)
        instructions_set = self.generate_p2p_movement_instructions(schedules)
        self.send_instructions(instructions_set)
        return schedules
    
    def convert_solution_to_schedules(self, solution):
            num_robots = len(solution['agt'])
            robot_schedules = []
            
            for robot_id in range(num_robots):
                schedule = []
                agent_data = solution['agt'][robot_id]

                for i in range(len(agent_data['t'])):
                    time = agent_data['t'][i]
                    action_id = agent_data['id'][i]

                    location = None
                    action_type = None
                    task_num = None

                    if action_id < num_robots:
                        # This is the agent's home/start location
                        location = self.action_points[action_id]
                        action_type = "WAIT"
                    else:
                        # Task-related action
                        task_idx = (action_id - num_robots) // 2
                        is_pickup = ((action_id - num_robots) % 2 == 0)
                        if is_pickup:
                            action_type = "PICKUP"
                            location = self.action_points[self.tasks[task_idx].start]
                        else:
                            action_type = "DROPOFF"
                            location = self.action_points[self.tasks[task_idx].end]
                        task_num = task_idx

                    schedule.append({
                        'time': time,
                        'location': location,
                        'action': action_type,
                        'task_id': task_num
                    })
                schedule.sort(key=lambda x: x['time'])
                robot_schedules.append(schedule)

            for robot_id, schedule in enumerate(robot_schedules):
                print(f"\nRobot {robot_id} Plan:")
                print("Time  | Location | Action  | Task")
                print("-" * 40)
                
                for step in schedule:
                    task_str = f"Task {step['task_id']}" if step['task_id'] is not None else "N/A"
                    print(f"{step['time']} | {step['location']} | {step['action']} | {task_str}") 
            self.vg.schedules = robot_schedules
            self.vg.solver_ran = True
            return robot_schedules

    def generate_p2p_movement_instructions(self, robot_schedules):
        instructions_set = []
        
        for robot_id, rschedule in enumerate(robot_schedules):
            if not rschedule:  # Handle empty schedule
                instructions_set.append([])
                continue
                
            instructions = []
            prev_dir = None

            print(f"Robot {robot_id} Instructions:")
            for i in range(len(rschedule)-1):
                next_action = rschedule[i+1]['action']
                path = self.vg.ap_paths.get((rschedule[i]['location'], rschedule[i+1]['location']))
                if path is None:
                    path = nx.shortest_path(self.vg.graph, source=rschedule[i]['location'], target=rschedule[i+1]['location'], weight='weight')
                    self.vg.ap_paths[rschedule[i]['location'], rschedule[i+1]['location']] = path

                if not path:
                    continue
                    
                if len(path) > 1:
                    step = 0
                    while step < len(path)-1:
                        dx = path[step+1][1] - path[step][1]  # 1 for x since grid is (row,col)
                        dy = path[step+1][0] - path[step][0]
                        magnitude = max(abs(dx), abs(dy))
                        curr_dir = (dx/magnitude, dy/magnitude) if magnitude > 0 else (0, 0)
                        
                        turn_angle = self.vg.calculate_turn_angle(prev_dir, curr_dir)
                        if turn_angle:
                            instructions.append(f"TURN+{turn_angle}")

                        # Combine consecutive straight movements
                        n = 1
                        while (step + n < len(path)-1):
                            next_dx = path[step+n+1][1] - path[step+n][1]
                            next_dy = path[step+n+1][0] - path[step+n][0]
                            next_magnitude = max(abs(next_dx), abs(next_dy))
                            next_dir = (next_dx/next_magnitude, next_dy/next_magnitude) if next_magnitude > 0 else (0, 0)
                            if next_dir == curr_dir:
                                n += 1
                            else:
                                break

                        move_duration = n * self.vg.graph.get_edge_data(path[step], path[step+1])['weight'] * self.vg.MOVE_DURATION_MS_PER_CM
                        instructions.append(f"MOVE+{move_duration}")
                        
                        step += n
                        prev_dir = curr_dir

                    if next_action == "PICKUP" or next_action == "DROPOFF":
                        instructions.append("TURN+360")

            instructions_set.append(instructions)
            instr_str = " > ".join(instructions)
            print(f"Robot {robot_id} Instruction string: \n {instr_str}")
            
        return instructions_set

    def send_instructions(self, instructions_set):
        for i, instructions in enumerate(instructions_set):
            robot_id = f"robot {i+1}"
            for instruction in instructions:
                # self.robots[robot_id].send_command(instruction)
                print(f"Sent instruction {instruction} to {robot_id}")
            print(f"Finished sending instructions to {robot_id}")

def main():
    Coordinator()
    while input() != 'x':
        pass

if __name__ == '__main__':
    main()