import sys
import os
import threading
from time import sleep
import networkx as nx
import json
import subprocess
import time
import socket
from cbs_mapf.planner import Planner, Agent
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from lib.SMrTa.MRTASolver import MRTASolver, Robot
from lib.SMrTa.MRTASolver.objects import Task
from Vision import Vision
from src.robot.RobotController import RobotController


class Coordinator:
    def __init__(self):
        self.schedules = None # {robot_id: [waypoint1, waypoint2, ...]}
        self.stage_instructions = {} # {stage: {robot_id: [instr1, instr2, ...]}}
        self.collision_free_paths = {} # {stage: {robot_id: [path1, path2, ...]}}
        self.start_server()
        self.connect_robots()
        video='img/reo.mp4'
        self.vision = Vision(self, 0)
        self.vision.app.exec_()

        self.vision.cap.release()
        self.stop_server()

    def start_server(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.bind(("127.0.0.1", 8000))
                s.close()
                print("Starting server...")
                server_path = os.path.join(os.path.dirname(__file__), 'Server.py')
                stdout_file = open('server_stdout.txt', 'w')
                stderr_file = open('server_stderr.txt', 'w')
                self.server_process = subprocess.Popen(
                    [sys.executable, server_path], 
                    stdout=stdout_file,
                    stderr=stderr_file
                )
            except socket.error as e:
                print(f"Socket error: {e}")
                self.server_process = None
        except Exception as e:
            print(f"Error starting server: {e}")
            self.server_process = None

    def stop_server(self):
        if hasattr(self, 'server_process') and self.server_process:
            print("Stopping server...")
            self.server_process.terminate()
            try:
                self.server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server_process.kill()

    def connect_robots(self):
        with open('devices.json', 'r') as f:
            robot_configs = json.load(f)['devices']
        self.robots = {
            robot['name']: RobotController(robot['name'], robot['address'], robot['write_uuid']) for robot in robot_configs
        }
        connected_count = 0
        for robot_name, robot in self.robots.items():
            print(f"Connecting to {robot_name}...")
            if robot.connect():
                connected_count += 1
                print(f"Connected to {robot_name}")
            else:
                print(f"Failed to connect to {robot_name}")
        
        print(f"Successfully connected to {connected_count}/{len(self.robots)} robots")

    def run_solver(self):
        agents = [Robot(id=f"robot {marker_id}", start=pos) for marker_id, pos in self.vision.robot_coords.items()]
        
        tasks = [Task(id=task['id'], start=tuple(task['start']), end=tuple(task['end']), deadline=task['deadline']) for task in self.vision.ui.task_coords.values()]
        
        self.action_points = [coord for coord in self.vision.robot_coords.values()]
        for task in tasks:
            self.action_points.append(task.start)
            self.action_points.append(task.end)

        # Remap agent and task start/end values
        print("Agents:")
        for a in agents:
            print(a.id, a.start)
            a.start = self.action_points.index(a.start)
        print("Tasks:")
        for t in tasks:
            print(t.id, t.start, t.end)
            t.start = self.action_points.index(t.start)
            t.end = self.action_points.index(t.end)
        print("Running solver...")
        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV', 
            agents=agents,
            tasks_stream=[[tasks, 0]],
            room_graph=self.vision.create_travel_time_matrix(self.action_points),
            capacity=1,
            num_aps=len(self.action_points),
            aps_list=[np.ceil(len(tasks)/len(agents))*2+1, len(self.action_points)],
            fidelity=1,
        )
        if solver.sol is None:
            print("No solution found!")
            return None
        
        solution = solver.sol

        num_robots = len(solution['agt'])
        robot_schedules = {}
        
        for robot_id in range(num_robots):
            schedule = []
            agent_data = solution['agt'][robot_id]

            for i in range(len(agent_data['t'])):
                time = agent_data['t'][i]
                action_id = agent_data['id'][i]

                if time == agent_data['t'][-1]:
                    if i > 0 and agent_data['t'][i - 1] == time:
                        break
                        
                location = None
                action_type = None
                task_num = None

                if action_id < num_robots:
                    # Agent's start location
                    location = self.action_points[action_id]
                    action_type = "WAIT"
                else:
                    task_idx = (action_id - num_robots) // 2
                    is_pickup = ((action_id - num_robots) % 2 == 0)
                    if is_pickup:
                        action_type = "PICKUP"
                        location = self.action_points[tasks[task_idx].start]
                    else:
                        action_type = "DROPOFF"
                        location = self.action_points[tasks[task_idx].end]
                    task_num = task_idx
                schedule.append({
                    'time': time,
                    'location': location,
                    'action': action_type,
                    'task_id': task_num
                })
            schedule.sort(key=lambda x: x['time'])
            robot_schedules[robot_id] = schedule
        self.schedules = robot_schedules
        for robot_id, schedule in robot_schedules.items():
            print(f"\nRobot {robot_id} Plan:")
            print("Time  | Location | Action  | Task")
            print("-" * 40)
            for step in schedule:
                task_str = f"Task {step['task_id']}" if step['task_id'] is not None else "N/A"
                print(f"{step['time']} | {step['location']} | {step['action']} | {task_str}") 

        for stage in range(max(len(s) for s in robot_schedules.values())):
            self.stage_instructions[stage] = self.generate_collision_free_instructions_for_stage(robot_schedules, stage)
        return robot_schedules
        

    def generate_collision_free_instructions_for_stage(self, robot_schedules, stage):
        """
        For a given stage index, plan multi-robot collision-free paths for all robots,
        then generate turn/move instructions assuming each starts facing down (0,1)
        and ends facing down (0,1).
        Returns a dict: { robot_id: [instr1, instr2, ...] }
        """
        obstacle_mask = self.vision.get_obstacle_grid(self.vision.cap.read()[1], with_buffer=False)
        obstacles = [
            (c, r)
            for r in range(obstacle_mask.shape[0])
            for c in range(obstacle_mask.shape[1])
            if obstacle_mask[r, c] == 1
        ]
        obstacles += [
            (0, 0),
            (self.vision.ENV_WIDTH_CM-1, 0),
            (0, self.vision.ENV_HEIGHT_CM-1),
            (self.vision.ENV_WIDTH_CM-1, self.vision.ENV_HEIGHT_CM-1)
        ]

        planner = Planner(
            grid_size=1,
            robot_radius=2,
            static_obstacles=obstacles,
        )

        starts, goals, robot_order = [], [], []
        for robot_id, schedule in robot_schedules.items():
            if stage + 1 < len(schedule):
                s = schedule[stage]['location']
                g = schedule[stage + 1]['location']
                if s != g:
                    starts.append((s[1], s[0]))  # (x,y)
                    goals.append((g[1], g[0]))
                    robot_order.append(robot_id)

        if len(starts) < 2:
            xy_paths = []
        else:
            try:
                xy_paths = planner.plan(starts, goals, assign=assigner)
                print(f"Generated paths for stage {stage}: {xy_paths}")
            except Exception:
                xy_paths = []
                print(f"Error generating paths for stage {stage}, using shortest path as fallback.")
        self.collision_free_paths[stage] = {}
        stage_instructions = {}
        for robot_id in robot_schedules.keys():
            stage_instructions[robot_id] = []

        print(f"Processing paths for {len(robot_order)} robots in stage {stage}")

        for i, robot_id in enumerate(robot_order):
            try:
                path = []
                if i < len(xy_paths) and xy_paths[i] is not None and len(xy_paths[i]) > 0:
                    path = [(p[1], p[0]) for p in xy_paths[i]]
                    print(f"Robot {robot_id}: Using collision-free path with {len(path)} points")
                if not path:
                    start = robot_schedules[robot_id][stage]['location']
                    end = robot_schedules[robot_id][stage + 1]['location']
                    try:
                        path = nx.shortest_path(self.vision.graph, source=start, target=end, weight='weight')
                        print(f"Robot {robot_id}: Using shortest path with {len(path)} points")
                    except nx.NetworkXNoPath:
                        print(f"Robot {robot_id}: No path found between {start} and {end}")
                        path = []
                        
                if not path:
                    print(f"Robot {robot_id}: No path available, skipping instructions")
                    self.collision_free_paths[stage][robot_id] = []
                    continue
                    
                self.collision_free_paths[stage][robot_id] = path
            except Exception as e:
                print(f"Error processing path for robot {robot_id}: {e}")
                self.collision_free_paths[stage][robot_id] = []
                continue
            print("Paths found, generating instructions...")
            instrs, prev_dir = [], (0, 1)
            step = 0
            while step < len(path) - 1:
                dx = path[step+1][1] - path[step][1]
                dy = path[step+1][0] - path[step][0]
                magnitude = max(abs(dx), abs(dy))
                curr_dir = (dx/magnitude, dy/magnitude) if magnitude > 0 else prev_dir

                angle = self.vision.calculate_turn_angle(prev_dir, curr_dir)
                if angle < 0:
                    instrs.append(f"RIGHT+{abs(int(angle))}")
                elif angle > 0:
                    instrs.append(f"LEFT+{int(angle)}")

                n = 1
                while step + n < len(path) - 1:
                    next_dx = path[step+n+1][1] - path[step+n][1]
                    next_dy = path[step+n+1][0] - path[step+n][0]
                    next_magnitude = max(abs(next_dx), abs(next_dy))
                    next_dir = (next_dx/next_magnitude, next_dy/next_magnitude) if next_magnitude > 0 else curr_dir
                    if next_dir == curr_dir:
                        n += 1
                    else:
                        break
                try:
                    dist = nx.shortest_path_length(self.vision.update_graph_8d(), source=path[step], target=path[step+n], weight='weight')
                except nx.NetworkXNoPath:
                    print(f"Error calculating distance from {path[step]} to {path[step+n]}, skipping")
                    dx = path[step+n][1] - path[step][1]
                    dy = path[step+n][0] - path[step][0]
                    dist = int(np.sqrt(dx**2 + dy**2))
                instrs.append(f"MOVE+{int(dist)}")
                prev_dir = curr_dir
                step += n

            action = robot_schedules[robot_id][stage + 1]['action']
            if action in ("PICKUP", "DROPOFF"):
                instrs.append("RIGHT+360")

            final_turn = self.vision.calculate_turn_angle(prev_dir, (0, 1))
            if final_turn < 0:
                instrs.append(f"RIGHT+{abs(int(final_turn))}")
            elif final_turn > 0:
                instrs.append(f"LEFT+{int(final_turn)}")

            stage_instructions[robot_id] = instrs

        return stage_instructions

    def execute(self):
        execution_thread = threading.Thread(target=self.execute_thread)
        execution_thread.daemon = True
        execution_thread.start()

    def execute_thread(self):
        for stage in self.stage_instructions:
            threads = []
            instructions_set = self.stage_instructions[stage]
            
            for robot_id, instructions in instructions_set.items():
                robot_name = f"robot {robot_id}"
                if robot_name in self.robots and instructions:
                    thread = threading.Thread(
                        target=self.send_instructions_to_robot,
                        args=(robot_name, instructions, robot_id)
                    )
                    thread.start()
                    threads.append(thread)
            for thread in threads:
                thread.join()
        print("All robots completed their instructions or encountered errors")

    def send_instructions_to_robot(self, robot_name, instructions, robot_id):
        robot = self.robots[robot_name]
        if not robot.connected and not robot.connect():
            print(f"Failed to connect to robot {robot_id}, cannot send instructions")
            return
        print(f"Sending {len(instructions)} instructions to robot {robot_id}")
        for i, instruction in enumerate(instructions):
            try:
                print(f"Sending {instruction} to robot {robot_id}")
                result = robot.send_command(instruction)
                print(f"Result: {result}")
            except Exception as e:
                print(f"Error at instruction {i+1}: {e}")
                break
        print(f"Finished sending instructions for robot {robot_id}")


def assigner(starts, goals):
    assert(len(starts) == len(goals))
    agents = []
    for i, start in enumerate(starts):
        agents.append(Agent(start, goals[i]))
    return agents


def main():
    Coordinator()

if __name__ == '__main__':
    main()