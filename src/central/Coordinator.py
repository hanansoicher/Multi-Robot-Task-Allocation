import os
import sys
import json
import socket
import threading
import subprocess
import numpy as np
from collections import OrderedDict

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from UI import UI
from Vision import Vision
from MAPF_Wrapper import MAPF_Wrapper
from robot.RobotController import RobotController
from lib.SMrTa.MRTASolver import MRTASolver, Robot
from lib.SMrTa.MRTASolver.objects import Task

# All grid coordinates (row, col) refer to the center of the grid cell, i.e., (row + 0.5, col + 0.5)

class Coordinator:
    def __init__(self):
        self.robots = OrderedDict() # {robot_id: RobotController}
        self.collision_free_paths = None # {robot_id: (time, row, col, orientation) path}
        self.schedules = None # {robot_id: (time, row, col, orientation) path}
        self.robot_instructions = None # {robot_id: [instruction]}

        self.start_server()
        self.connect_robots()
        # camera = 'robotenv_video.mp4'
        camera = 0
        self.vision = Vision(camera_input=camera)
        
        try:
            self.mapf_wrapper = MAPF_Wrapper()
        except Exception as e:
            print(f"[Coordinator] Failed to initialize MAPF-PC wrapper: {e}")
            self.mapf_wrapper = None
            
        self.ui = UI(coordinator=self, vision=self.vision)
        sys.exit(self.ui._app.exec_())


    def start_server(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(("127.0.0.1", 8000))
            s.close()
            print("[Coordinator] Starting BLE HTTP server")
            server_process = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(__file__), "server.py")])
        except OSError:
            print("[Coordinator] Server already running on port 8000.")
        finally:
            try:
                s.close()
            except Exception:
                pass


    def connect_robots(self):
        with open("devices.json", "r", encoding="utf-8") as f:
            devices = json.load(f).get("devices", [])

        devices.sort(key=lambda d: d["marker_id"])
        connected = 0
        for d in devices:
            robot = RobotController(int(d["marker_id"]), d["address"], d["write_uuid"])
            if robot.connect():
                connected += 1
                self.robots[robot.id] = robot
        print(f"[Coordinator] Connected {connected}/{len(devices)} robots.")


    def run_solver(self):
        agents = [Robot(id=marker_id, start=self.vision.robot_coords[marker_id]) for marker_id in sorted(self.vision.robot_coords.keys())]
        tasks = [Task(id=task['id'], start=tuple(task['start']), end=tuple(task['end']), deadline=task['deadline']) for task in self.ui.task_coords.values()]
        
        action_points = [agent.start for agent in agents]
        for task in tasks:
            action_points.append(task.start)
            action_points.append(task.end)

        # Remap agent and task start/end values
        for a in agents:
            a.start = action_points.index(a.start)
        for t in tasks:
            t.start = action_points.index(t.start)
            t.end = action_points.index(t.end)
        print("Running solver...")
        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV', 
            agents=agents,
            tasks_stream=[[tasks, 0]],
            room_graph=self.vision.create_travel_time_matrix(action_points),
            capacity=1,
            num_aps=len(action_points),
            aps_list=[np.ceil(len(tasks)/len(agents))*2+1, len(action_points)],
            fidelity=1,
            debug=False
        )
        if solver.sol is None:
            print("MRTA solver failed")
            return None
        
        solution = solver.sol
        num_robots = len(solution['agt'])

        robot_schedules = {}
        for id, agent in enumerate(agents):
            schedule = []
            agent_data = solution['agt'][id]
            print(f"Processing agent {agent.id} with data: {agent_data}")
        
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
                    location = action_points[action_id]
                    action_type = "WAIT" # Start location
                else:
                    task_index = (action_id - num_robots) // 2
                    if ((action_id - num_robots) % 2 == 0):
                        action_type = "PICKUP"
                        location = action_points[tasks[task_index].start]
                    else:
                        action_type = "DROPOFF"
                        location = action_points[tasks[task_index].end]
                    task_num = task_index
                schedule.append({
                    'time': time,
                    'location': location,
                    'action': action_type,
                    'task_id': task_num
                })
            schedule.sort(key=lambda x: x['time'])
            robot_schedules[agent.id] = schedule[:-1] # don't return to start location
        
        robot_schedules = OrderedDict(sorted(robot_schedules.items(), key=lambda kv: int(kv[0])))

        for robot_id, schedule in robot_schedules.items():
            print(f"\nRobot {robot_id} Plan:")
            print("Time  | Location | Action  | Task")
            print("-" * 40)
            for step in schedule:
                task_str = f"Task {step['task_id']}" if step['task_id'] is not None else "N/A"
                print(f"{step['time']} | {step['location']} | {step['action']} | {task_str}") 

        print("\n" + "-"*50)
        print("Generating collision-free paths\n")
        print("-"*50)
        
        try:
            grid_size = (self.vision.ROWS, self.vision.COLS)
            
            obstacles = self.vision.get_obstacle_coordinates()
            print(f"Found {len(obstacles)} obstacles")
            print(f"Grid size: {grid_size}")

            collision_free_paths = self.mapf_wrapper.generate_collision_free_paths(robot_schedules, grid_size, obstacles=obstacles)
            if collision_free_paths:
            #     print(f"\nSuccessfully generated collision-free paths for {len(collision_free_paths)} robots")
                # original_paths = {k: v[:-2] for k, v in collision_free_paths.items()}
                
                print("\nPostprocessing paths")
                postprocessed_paths = self.add_turns_and_waits(collision_free_paths)

                self.collision_free_paths = postprocessed_paths
                print(f"Postprocessed paths:")
                for robot_id, path in postprocessed_paths.items():
                    print(f"Robot {robot_id}: {path}")
                
                self.generate_movement_instructions_from_mapf_paths(postprocessed_paths)
            else:
                print("Failed to generate collision-free paths")
                self.collision_free_paths = None
                
        except Exception as e:
            print(f"Error generating collision-free paths: {e}")
            self.collision_free_paths = None
        self.schedules = robot_schedules

    def add_turns_and_waits(self, paths):
        """Add turn steps and wait steps to ensure all robots move synchronously."""
        for robot_id, path in paths.items():
            print(f"  Initial path for robot {robot_id}: {path}")

        validated_paths = {}
        for robot_id, path in paths.items():
            validated_paths[robot_id] = []
            for i, (time, row, col) in enumerate(path):
                if i == 0:
                    orientation = (1, 0)  # All robots start facing down
                else:
                    prev_row, prev_col = path[i-1][1], path[i-1][2]
                    drow = row - prev_row
                    dcol = col - prev_col
                    
                    if drow == 0 and dcol == 0: # Same position, keep previous orientation
                        orientation = validated_paths[robot_id][i-1][3]
                    elif drow == 0: # Horizontal movement
                        orientation = (0, 1 if dcol > 0 else -1)
                    elif dcol == 0: # Vertical movement
                        orientation = (1 if drow > 0 else -1, 0)
                    else:
                        # Diagonal movement not allowed
                        orientation = validated_paths[robot_id][i-1][3]
                        
                validated_paths[robot_id].append([time, row, col, orientation])
        
        iteration = 0
        while iteration < 100:
            print(f"\n========================= Iteration {iteration} =========================")
            iteration += 1
            
            # Find the next time when any robots need to turn
            next_turn_time, next_turns = self.find_next_turn(validated_paths)
            
            if next_turn_time == float('inf') or len(next_turns) == 0:
                print("No more turns to check")
                break
                
            print(f"next turns: {next_turns}")
            
            insertion_time = next_turn_time + 1 # robot moves into the coordinate it needs to turn at next_turn_time, turn  with updated orientationshould be inserted at next_turn_time + 1
            
            # For each robot, either a single 90 degree turn step or a wait step is inserted at insertion_time. 
            for robot_id in validated_paths.keys():
                path = validated_paths[robot_id]
                
                insertion_index = next((i for i, step in enumerate(path) if step[0] >= insertion_time), None)
                if insertion_index is None: 
                    continue

                if robot_id in next_turns: #insert 90 degree turn step
                    turn = next_turns[robot_id]
                    current_orientation = turn['from_orientation']
                    
                    dir_to_angle = { # (0=Right, 90=Down, 180=Left, 270=Up) (top left of environment is (0, 0))
                        (0, 1): 0, 
                        (1, 0): 90, 
                        (0, -1): 180, 
                        (-1, 0): 270}
                    angle_to_dir = {v: k for k, v in dir_to_angle.items()}
                    
                    curr_angle = dir_to_angle.get(current_orientation, 0)
                    intermediate_angle = (curr_angle + (90 if turn['angle'] > 0 else -90)) % 360
                    intermediate_orientation = angle_to_dir.get(intermediate_angle, current_orientation)
                    turn_step = [insertion_time, turn['position'][0], turn['position'][1], intermediate_orientation]
                    
                    # Shift remaining steps and update their times
                    shifted_path = path[:insertion_index] + [turn_step]
                    for step in path[insertion_index:]:
                        shifted_path.append([step[0] + 1, step[1], step[2], intermediate_orientation])    
                    validated_paths[robot_id] = shifted_path
                    print(f"Inserted turning step for Robot {robot_id} at time {insertion_time} and coordinate {turn['position']} from orientation {current_orientation} to {intermediate_orientation}")
                else: #insert wait step
                    prev_step = path[insertion_index - 1]
                    wait_step = [insertion_time, prev_step[1], prev_step[2], prev_step[3]]
                    
                    # Shift remaining steps
                    shifted_path = path[:insertion_index] + [wait_step]
                    for step in path[insertion_index:]:
                        wait_step = [step[0] + 1, step[1], step[2], step[3]]
                        shifted_path.append(wait_step)
                    validated_paths[robot_id] = shifted_path
                    print(f"Inserted wait step for Robot {robot_id} at time {insertion_time}")
            
        print("Final postprocessed paths: ")
        for robot_id, path in validated_paths.items():
            print(f"  Robot {robot_id}: {path}")
        return validated_paths


    def find_next_turn(self, paths):
        """Find the earliest time when any robot needs to turn.
        Returns (time, {robot_id: turn_info}) where time is when the robot reaches the position where it will turn.
        """
        next_turns = {}
        earliest_turn_time = float('inf')
        
        for robot_id, path in paths.items():
            for i in range(len(path) - 1):
                curr_step = path[i]
                next_step = path[i + 1]
                
                curr_pos = (curr_step[1], curr_step[2])
                next_pos = (next_step[1], next_step[2])
                
                # Skip if same position (already processed turn/wait)
                if curr_pos == next_pos:
                    continue
                    
                drow = next_pos[0] - curr_pos[0]
                dcol = next_pos[1] - curr_pos[1]
                
                if drow == 0 and dcol != 0:
                    next_orientation = (0, 1 if dcol > 0 else -1)
                elif dcol == 0 and drow != 0:
                    next_orientation = (1 if drow > 0 else -1, 0)

                current_orientation = curr_step[3]
                if current_orientation != next_orientation:
                    turn_time = curr_step[0]
                    
                    if turn_time <= earliest_turn_time:
                        dir_to_angle = {
                            (0, 1): 0,
                            (1, 0): 90, 
                            (0, -1): 180, 
                            (-1, 0): 270
                        }
                        curr_angle = dir_to_angle.get(current_orientation, 0)
                        next_angle = dir_to_angle.get(next_orientation, 0)
                        
                        angle = next_angle - curr_angle
                        if angle > 180:
                            angle -= 360
                        elif angle < -180:
                            angle += 360
                        
                        if turn_time < earliest_turn_time:
                            earliest_turn_time = turn_time
                            next_turns = {}
                        
                        next_turns[robot_id] = {
                            'time': int(turn_time),
                            'position': curr_pos,
                            'angle': int(angle),
                            'from_orientation': current_orientation,
                            'to_orientation': next_orientation
                        }
                        break  # Only find earliest turn for each robot
        
        return int(earliest_turn_time) if earliest_turn_time < float('inf') else float('inf'), next_turns


    def generate_movement_instructions_from_mapf_paths(self, paths):
        if not paths:
            print("No collision-free paths available")
            return

        print("\nConverting paths to instructions")
        final_instructions = {robot_id: [] for robot_id in paths.keys()}

        for robot_id, path in paths.items():
            for i in range(1, len(path)):
                prev_step = path[i-1]
                curr_step = path[i]
                # Check for wait step (same position and orientation)
                if (curr_step[1] == prev_step[1] and curr_step[2] == prev_step[2] and curr_step[3] == prev_step[3]):
                    final_instructions[robot_id].append("W")
                else:
                    curr_orientation = curr_step[3]
                    prev_orientation = prev_step[3]
                    angle = 0
                    if curr_orientation != prev_orientation:
                        # (0=Right, 90=Down, 180=Left, 270=Up) (top left of environment is (0, 0))
                        dir_to_angle = {
                            (0, 1): 0,    # Right (increasing col)
                            (1, 0): 90,   # Down (increasing row)
                            (0, -1): 180, # Left (decreasing col)
                            (-1, 0): 270  # Up (decreasing row)
                        }
                        angle = dir_to_angle.get(curr_orientation, 0) - dir_to_angle.get(prev_orientation, 0)
                    if angle > 180:
                        angle -= 360
                    elif angle < -180:
                        angle += 360
                    if angle > 0: # Right
                        final_instructions[robot_id].append("R")
                    elif angle < 0: # Left
                        final_instructions[robot_id].append("L")
                    elif angle == 0:
                        final_instructions[robot_id].append("M")
        self.robot_instructions = final_instructions
        for robot_id, instructions in self.robot_instructions.items():
            print(f"\nRobot {robot_id} instructions: {instructions}")
            print(f"Total instructions: {len(instructions)}")


    def execute_instructions(self):
        execution_thread = threading.Thread(target=self.execute_thread)
        execution_thread.daemon = True
        execution_thread.start()
        return True

    def execute_thread(self):        
        # Fill all robot queues with their instructions first
        ble_mtu = 20 
        prefix = "CMDS+"
        commands_per_chunk = (ble_mtu - len(prefix))
        for robot_id in self.robot_instructions.keys():
            robot = self.robots[robot_id]
            robot.send_command("CLEAR")
            instructions = self.robot_instructions[robot_id]
            command_string = ''.join(instructions)
            i = 0
            start = 0
            while start < len(command_string):
                i += 1
                end = min(start + commands_per_chunk, len(command_string))
                chunk = command_string[start:end]
                payload = prefix + chunk
                print(f"Sending chunk {i} to robot {robot_id}: {payload}")
                ret = robot.send_command(payload)
                attempts = 0
                while ret is None:
                    print(f"Failure {attempts} sending chunk {i} to robot {robot_id}")
                    ret = robot.send_command(payload)                        
                    attempts += 1
                    if attempts == 5:
                        print(f"Failed to send chunk {i} to robot {robot_id} after 5 attempts")
                        raise Exception(f"Failed to send chunk {i} to robot {robot_id} after 5 attempts")
                start = end
        print("All commands sent to all robots. Starting synchronized execution.")
        max_steps = max(len(instrs) for instrs in self.robot_instructions.values())
        # For each step, send EXEC to all robots in parallel
        for step in range(max_steps):
            def send_execute(robot):
                attempts = 0
                ret = robot.send_command("EXEC")
                while ret is None:
                    print(f"Failure {attempts} sending 'EXEC' to robot {robot.id}")
                    ret = robot.send_command("EXEC")
                    attempts += 1
                    if attempts == 5:
                        print(f"Failed to send 'EXEC' to robot {robot.id} after 5 attempts")
                        raise Exception(f"Failed to send 'EXEC' to robot {robot.id} after 5 attempts")

            execute_threads = []
            for robot_id in self.robot_instructions.keys():
                t = threading.Thread(target=send_execute, args=(self.robots[robot_id],))
                t.start()
                execute_threads.append(t)
            for t in execute_threads:
                t.join()
            print(f"Step {step+1}/{max_steps} executed by all robots.")
        for robot_id in self.robot_instructions.keys():
            self.robots[robot_id].send_command("CLEAR")
        


if __name__ == "__main__":
    Coordinator()
