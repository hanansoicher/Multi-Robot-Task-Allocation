import os
import sys
import json
import socket
import threading
import subprocess
import time
import numpy as np
import networkx as nx
from collections import OrderedDict

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from UI import UI
from MAPF_Wrapper import MAPF_Wrapper
from src.robot.RobotController import RobotController
from lib.SMrTa.MRTASolver import MRTASolver, Robot
from lib.SMrTa.MRTASolver.objects import Task

# All grid coordinates (row, col) refer to the center of the grid cell, i.e., (row + 0.5, col + 0.5)

class Coordinator:
    GRID_SIZE_CM = 15
    ENV_WIDTH_CM = 90
    ENV_HEIGHT_CM = 60
    # ENV_WIDTH_CM = 90
    # ENV_HEIGHT_CM = 90
    ROWS, COLS = ENV_HEIGHT_CM // GRID_SIZE_CM, ENV_WIDTH_CM // GRID_SIZE_CM

    MAX_NUM_TASKS = 20

    def __init__(self):
        self.robots = OrderedDict() # {robot_id: RobotController}
        self.collision_free_paths = None # {robot_id: (time, row, col, orientation) grid-level paths}
        self.schedules = None # {robot_id: (time, row, col, orientation) of task locations}
        self.robot_instructions = None # {robot_id: [instruction]}
        self.task_progress = None # {robot_id: (task_id, bool completed_pickup, bool completed_dropoff) for task in self.schedules}
        self.last_completed_step = 0
        
        self.execution_running = False
        self.execution_thread = None
        self.stop_execution_event = threading.Event()
        self.execution_lock = threading.Lock()

        self.start_server()
        self.connect_robots()
        # camera = 'img/robotenv_video.mp4'
        camera = 0
        
        try:
            self.mapf_wrapper = MAPF_Wrapper()
        except Exception as e:
            print(f"[Coordinator] Failed to initialize MAPF-PC wrapper: {e}")
            self.mapf_wrapper = None
            
        self.ui = UI(coordinator=self, camera_input=camera)
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
            marker_id = int(d["marker_id"])
            # Temporary fix: marker 3 represents robot 2 (since marker 2 was not printed)
            if marker_id == 3:
                robot_id = 2  # Map marker 3 to robot 2
            else:
                robot_id = marker_id  # Map other markers normally
            
            print(f"[Coordinator] Connecting marker {marker_id} as robot {robot_id} at address {d['address']}")
            robot = RobotController(robot_id, d["address"], d["write_uuid"])
            if robot.connect():
                connected += 1
                self.robots[robot.id] = robot
                print(f"[Coordinator] Successfully connected robot {robot.id}")
            else:
                print(f"[Coordinator] Failed to connect robot {robot_id}")
        print(f"[Coordinator] Connected {connected}/{len(devices)} robots.")


    def run_solver(self):        
        print(f"[Coordinator] Robot coordinates: {self.ui.robot_coords}")
        print(f"[Coordinator] Task coordinates: {self.ui.task_coords}")
        
        try:
            agents = [Robot(id=marker_id, start=self.ui.robot_coords[marker_id]) for marker_id in sorted(self.ui.robot_coords.keys())]
            tasks = [Task(id=task['id'], start=tuple(task['start']), end=tuple(task['end']), deadline=task['deadline']) for task in self.ui.task_coords.values()]

            for a in agents:
                a.start = a.start[0] * self.COLS + a.start[1]  # Convert (row, col) to linear index
            for t in tasks:
                t.start = t.start[0] * self.COLS + t.start[1]  # Convert (row, col) to linear index
                t.end = t.end[0] * self.COLS + t.end[1]  # Convert (row, col) to linear index

        except Exception as e:
            print(f"[Coordinator] Error in agent/task setup: {e}")
            raise
        try:
            if self.task_progress is None:
                room_graph = self.create_travel_time_matrix([(row, col) for row in range(self.ROWS) for col in range(self.COLS)])
                try:
                    aps_list = [np.ceil(len(tasks)/len(agents))*2+1, len(room_graph)]
                    num_aps = len(room_graph)
                    
                    self.solver = MRTASolver(
                        solver_name='z3',
                        theory='QF_UFLIA', 
                        agents=agents,
                        tasks_stream=[[tasks, self.last_completed_step]],
                        room_graph=room_graph,
                        capacity=1,
                        num_aps=num_aps,
                        aps_list=aps_list,
                        fidelity=1,
                        incremental=True,
                        debug=False
                    )
                    print("[Coordinator] MRTASolver initialized successfully")
                except Exception as e:
                    print(f"[Coordinator] MRTASolver initialization failed: {e}")
                    print(f"[Coordinator] Exception type: {type(e)}")
                    import traceback
                    print(f"[Coordinator] Traceback: {traceback.format_exc()}")
                    raise
                actual_agent_arrivals = [[] for _ in range(len(agents))]
                self.num_tasks = len(tasks)
                initial_orientations = {robot.id: (1, 0) for robot in agents}
            else:
                actual_agent_arrivals = []
                
                for robot_id, schedule in self.schedules.items():
                    arrivals = []
                    for step in schedule:
                        if step['task_id'] in self.task_progress[robot_id]:
                            pickup_done, dropoff_done = self.task_progress[robot_id][step['task_id']]   
                            if pickup_done and step['action'] == 'PICKUP':
                                arrivals.append(step['time'])
                            elif dropoff_done and step['action'] == 'DROPOFF':
                                arrivals.append(step['time'])
                    actual_agent_arrivals.append(arrivals)
                initial_orientations = {robot.id: self.collision_free_paths[robot.id][self.last_completed_step+1][3] for robot in agents}
                self.solver.add_incoming_tasks(tasks[self.num_tasks:], self.last_completed_step)
            print(f"Orientations: {initial_orientations}")
            self.num_tasks = len(tasks)
            plan = self.solver.allocate_next_task_set(actual_agent_arrivals)
            
        except Exception as e:
            print(f"[Coordinator] Error in solver setup/execution: {e}")
            raise
        if plan is None:
            print("MRTA solver failed")
            return None
        try:
            self.process_solution(plan, agents, tasks, initial_orientations)
            self.execute_instructions()
            return True
        except Exception as e:
            print(f"[Coordinator] Error in process_solution/execute_instructions: {e}")
            raise


    def process_solution(self, solution, agents, tasks, initial_orientations): # If task_progress is not None, make sure to trim completed tasks
        num_robots = len(solution['agt'])
        print(solution)

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
                    # Use the robot's actual start position instead of converting from action_id
                    robot_id = agents[action_id].id
                    location = self.ui.robot_coords[robot_id]
                    action_type = "WAIT" # Start location
                else:
                    task_index = (action_id - num_robots) // 2
                    print(task_index)
                    if ((action_id - num_robots) % 2 == 0):
                        action_type = "PICKUP"
                        # Convert linear index back to (row, col)
                        row = tasks[task_index].start // self.COLS
                        col = tasks[task_index].start % self.COLS
                        location = (row, col)
                    else:
                        action_type = "DROPOFF"
                        # Convert linear index back to (row, col)
                        row = tasks[task_index].end // self.COLS
                        col = tasks[task_index].end % self.COLS
                        location = (row, col)
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
        remaining_schedules = robot_schedules.copy()
        # Remove completed tasks from schedules (only if task_progress exists)
        if self.task_progress is not None:
            for robot_id, schedule in robot_schedules.items():
                remaining_schedule = []
                for task in schedule:
                    if task['task_id'] is not None and task['task_id'] in self.task_progress[robot_id]:
                        pickup_done, dropoff_done = self.task_progress[robot_id][task['task_id']]
                        if task['action'] == 'PICKUP':
                            # Skip pickup if it's already completed
                            if pickup_done:
                                continue
                        elif task['action'] == 'DROPOFF':
                            # Skip dropoff if it's already completed
                            if dropoff_done:
                                continue
                    remaining_schedule.append(task)
                remaining_schedules[robot_id] = remaining_schedule

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
            grid_size = (self.ROWS, self.COLS)
            
            obstacles = self.ui.get_obstacle_coordinates()
            print(f"Found {len(obstacles)} obstacles")
            print(f"Grid size: {grid_size}")

            collision_free_paths = self.mapf_wrapper.generate_collision_free_paths(remaining_schedules, grid_size, obstacles=obstacles)
            if collision_free_paths:
                postprocessed_paths = self.add_turns_and_waits(collision_free_paths, initial_orientations)

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
        self.schedules = self.update_robot_schedules(remaining_schedules)
        self.ui._pending_schedules = self.schedules
        print(f"[Coordinator] Schedules: ")
        for robot_id, schedule in self.schedules.items():
            print(f"  Robot {robot_id}: {schedule}")

    def add_turns_and_waits(self, paths, initial_orientations):
        """Add turn steps and wait steps to ensure all robots move synchronously."""
        for robot_id, path in paths.items():
            print(f"  Initial path for robot {robot_id}: {path}")

        validated_paths = {}
        for robot_id, path in paths.items():
            validated_paths[robot_id] = []
            for i, (time, row, col) in enumerate(path):
                if i == 0:
                    orientation = initial_orientations[robot_id]
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
            # print(f"\n========================= Iteration {iteration} =========================")
            iteration += 1
            
            # Find the next time when any robots need to turn
            next_turn_time, next_turns = self.find_next_turn(validated_paths)
            
            if next_turn_time == float('inf') or len(next_turns) == 0:
                # print("No more turns to check")
                break
                
            # print(f"next turns: {next_turns}")
            
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
                    # print(f"Inserted turning step for Robot {robot_id} at time {insertion_time} and coordinate {turn['position']} from orientation {current_orientation} to {intermediate_orientation}")
                else: #insert wait step
                    prev_step = path[insertion_index - 1]
                    wait_step = [insertion_time, prev_step[1], prev_step[2], prev_step[3]]
                    
                    # Shift remaining steps
                    shifted_path = path[:insertion_index] + [wait_step]
                    for step in path[insertion_index:]:
                        wait_step = [step[0] + 1, step[1], step[2], step[3]]
                        shifted_path.append(wait_step)
                    validated_paths[robot_id] = shifted_path
                    # print(f"Inserted wait step for Robot {robot_id} at time {insertion_time}")
            
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


    def update_robot_schedules(self, robot_schedules): #Update time steps of robot schedules to match collision-free paths
        if self.collision_free_paths is None:
            return robot_schedules
        paths = self.collision_free_paths.copy()
        
        for robot_id, schedule in robot_schedules.items():
            print(f"[Coordinator] Robot {robot_id} schedule: ")            
            sorted_schedule = sorted(schedule, key=lambda x: x['time'])            
            completed_path_steps = set()
            
            for task_num, task in enumerate(sorted_schedule):
                print(f"  Task {task['task_id']} before update: {task}")
                try:
                    # For first task, don't check previous time constraint
                    if task_num == 0:
                        updated_step = next((step for step in paths[robot_id] if step[1] == task['location'][0] and step[2] == task['location'][1] and tuple(step) not in completed_path_steps), None)
                    else:
                        updated_step = next((step for step in paths[robot_id] if step[1] == task['location'][0] and step[2] == task['location'][1] and step[0] >= sorted_schedule[task_num-1]['time'] and tuple(step) not in completed_path_steps), None)
                    if updated_step:
                        task['time'] = updated_step[0]
                        completed_path_steps.add(tuple(updated_step))
                        print(f"  Task {task['task_id']} after update: {task}")
                    else:
                        print(f"  No matching path step found for task {task['task_id']} at location {task['location']}")
                        print(f"  Remaining path steps: {[step for step in paths[robot_id] if tuple(step) not in completed_path_steps]}")
                except Exception as e:
                    print(f"  ERROR updating task {task['task_id']}: {e}")
                    raise
        return robot_schedules

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
        with self.execution_lock:
            self.execution_running = True
            self.stop_execution_event.clear()
            self.execution_thread = threading.Thread(target=self.execute_thread)
            self.execution_thread.daemon = True
            self.execution_thread.start()
            return True

    def execute_thread(self):        
        # Fill all robot queues with their instructions first
        for robot_id in self.robot_instructions.keys():
            robot = self.robots[robot_id]
            instructions = self.robot_instructions[robot_id]
            command_string = ''.join(instructions)
            command_string = "CMDS+" + command_string
            
            # print(f"[Coordinator] Sending command string to robot {robot_id}: {command_string}")
            
            response = robot.send_command(command_string)
            
            if response is None:
                print(f"[Coordinator] Failed to send command string to robot {robot_id}")
                print(f"[Coordinator] Robot {robot_id} connection status: {robot.connected}")
                raise Exception(f"Failed to send command string to robot {robot_id}")
            else:
                print(f"[Coordinator] Successfully sent command string to robot {robot_id}: {response}")
        print("All commands sent to all robots. Starting synchronized execution.")
        max_steps = max(len(instrs) for instrs in self.robot_instructions.values())
        # For each step, send EXEC to all robots in parallel
        # schedules are sorted by time step, so robots will reach pickup and dropoff coords for each assigned task in order and at the time step they are scheduled for
        self.task_progress = {}
        for robot_id in self.robot_instructions.keys():
            self.task_progress[robot_id] = {}
            for step in self.schedules[robot_id]:
                if step['task_id'] is not None:
                    self.task_progress[robot_id][step['task_id']] = (False, False) 
        for step in range(max_steps):
            if self.stop_execution_event.is_set():
                print(f"[Coordinator] Execution stopped, last completed step: {self.last_completed_step}")
                print(f"[Coordinator] Task progress: {self.task_progress}")
                return
            
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
            self.last_completed_step = step
            self.ui._pending_schedules = self.schedules
            for robot_id in self.robot_instructions.keys(): # Update completed tasks
                curr_task = next((task for task in self.schedules[robot_id] if task['time'] == step), None)
                if curr_task is not None:
                    if curr_task['action'] == "PICKUP":
                        self.task_progress[robot_id][curr_task['task_id']] = (True, False)
                    elif curr_task['action'] == "DROPOFF":
                        self.task_progress[robot_id][curr_task['task_id']] = (True, True)

        with self.execution_lock:
            self.execution_running = False
            self.task_progress = None
            self.last_completed_step = 0

    def create_travel_time_matrix(self, locations): # locations is a list of (row, col) tuples representing task waypoints and robot start positions
        n = len(locations)
        matrix = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                try:
                    path = nx.shortest_path(self.ui.graph, (int(locations[i][0]), int(locations[i][1])), (int(locations[j][0]), int(locations[j][1])), weight='weight')
                    num_moves = len(path) - 1
                    prev_orientation = None
                    num_turns = 2 # 2 for potential 180 degree turn to begin the path
                    for k in range(len(path)-1):
                        dx = path[k+1][1] - path[k][1]
                        dy = path[k+1][0] - path[k][0]
                        curr_orientation = (dy, dx)
                        if prev_orientation is not None:
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
                            num_turns += int(abs(angle) // 90)
                        prev_orientation = curr_orientation
                    travel_time = num_moves + num_turns
                    matrix[i][j] = matrix[j][i] = travel_time
                except nx.NetworkXNoPath:
                    matrix[i][j] = matrix[j][i] = 999999
        return matrix


if __name__ == "__main__":
    Coordinator()
