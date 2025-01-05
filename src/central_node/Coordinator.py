import sys  
import os 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from SMrTa.MRTASolver import MRTASolver, Robot
from SMrTa.MRTASolver.objects import Task
from Vision import Vision 
import threading


class Coordinator:
    def __init__(self):
        video = "img/mazewmarkers.png"
        self.vg = Vision()
        threading.Thread(target=self.vg.run, daemon=True).start()
        threading.Thread(target=self.check_solver_inputs_ready, daemon=True).start()

    def check_solver_inputs_ready(self):
        while True:
            if input() == 'b':
                self.run_solver()

    def run_solver(self):
        robots, waypoints = self.vg.detect_markers(self.vg.frame)
        self.agents = [Robot(id=i, start=pos) for i, pos in enumerate([robots[i] for i in range(len(robots))])]
        self.tasks = [Task(id=i, start=waypoints[2*i], end=waypoints[2*i+1]) for i in range(len(waypoints)//2)]
        self.action_points = [robots[i] for i in range(len(robots))] + [waypoints[i] for i in range(len(waypoints))]
        solver_graph = self.vg.create_travel_time_matrix(self.action_points)

        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV', 
            agents=self.agents,
            tasks_stream=[[self.tasks, 0]],
            room_graph=solver_graph,
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
            return robot_schedules

def main():
    c = Coordinator()
    c.run_solver()

if __name__ == '__main__':
    main()