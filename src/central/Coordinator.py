import sys
import os
import threading
from time import sleep
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from lib.SMrTa.MRTASolver import MRTASolver, Robot
from lib.SMrTa.MRTASolver.objects import Task
from Vision import Vision
import networkx as nx
import json
from src.robot.RobotController import RobotController
import subprocess
import time
import socket

class HTTPCoordinator:
    def __init__(self):
        self.start_http_server()
        
        with open('devices.json', 'r') as f:
            robot_configs = json.load(f)['devices']
        
        self.robots = {
            f"robot {i+1}": RobotController(
                robot['name'], 
                robot['address'], 
                robot['write_uuid']
            ) for i, robot in enumerate(robot_configs)
        }

        self.connect_robots()

        # video = "img/test_maze.mp4"
        # video = "img/gridrvv.mp4"
        # video = "img/emptygrid.mp4"
        video=0
        self.vision = Vision(self, video, initialize_tape=False)
        self.vision.app.exec_()

        self.vision.cap.release()
        self.stop_http_server()

    def start_http_server(self):
        """Start the FastAPI server as a subprocess"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.bind(("127.0.0.1", 8000))
                s.close()
                print("Starting HTTP server...")
                server_path = os.path.join(os.path.dirname(__file__), 'HTTPServer.py')
                print(f"Server path: {server_path}")
                print(f"Does server file exist: {os.path.exists(server_path)}")
                
                stdout_file = open('server_stdout.txt', 'w')
                stderr_file = open('server_stderr.txt', 'w')
                
                self.server_process = subprocess.Popen(
                    [sys.executable, server_path], 
                    stdout=stdout_file,
                    stderr=stderr_file
                )
                time.sleep(5)
                print(f"Server process status: {'Running' if self.server_process.poll() is None else 'Exited with code ' + str(self.server_process.poll())}")
            except socket.error as e:
                print(f"Socket error: {e}")
                print("HTTP server is already running on port 8000")
                self.server_process = None
        except Exception as e:
            print(f"Error starting HTTP server: {e}")
            self.server_process = None

    def stop_http_server(self):
        """Stop the FastAPI server subprocess if it was started by us"""
        if hasattr(self, 'server_process') and self.server_process:
            print("Stopping HTTP server...")
            self.server_process.terminate()
            try:
                self.server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server_process.kill()

    def connect_robots(self):
        """Connect to all robots"""
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
        """Run the solver algorithm"""
        _, frame = self.vision.cap.read()
        cv2.imwrite("debug_frame.jpg", frame)
        r_coords = self.vision.find_robots(frame)
        robot_coords = []
        for robot_name, r_pos in r_coords.items():
            robot_coords.append(self.vision.find_closest_intersection(r_pos))
        print(robot_coords)
        # if len(robot_coords) == 0:
        #     robot_coords = [(469, 469)]
        agents = [Robot(id=f"robot {i+1}", start=pos) for i, pos in enumerate([robot_coords[i] for i in range(1)])]
        
        self.tasks = [Task(id=task['id'], start=tuple(task['start']), end=tuple(task['end']), deadline=task['deadline']) for task in self.vision.ui.task_coords.values()]
        
        self.action_points = [robot_coords[i] for i in range(len(robot_coords))]
        for task in self.tasks:
            if task.start not in self.action_points:
                self.action_points.append(task.start)
            if task.end not in self.action_points:
                self.action_points.append(task.end)

        # Remap agent and task start/end values
        print("Agents:")
        for a in agents:
            print(a.id, a.start)
            a.start = self.action_points.index(a.start)
        print("Tasks:")
        for t in self.tasks:
            print(t.id, t.start, t.end)
            t.start = self.action_points.index(t.start)
            t.end = self.action_points.index(t.end)
        print("Running solver...")
        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV', 
            agents=agents,
            tasks_stream=[[self.tasks, 0]],
            room_graph=self.vision.create_travel_time_matrix(self.action_points),
            capacity=1,
            num_aps=len(self.action_points),
            aps_list=[len(self.action_points)],
            fidelity=1,
        )
        if solver.sol is None:
            print("No solution found!")
            return None
        
        solution = solver.sol

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
                    # Agent's start location
                    location = self.action_points[action_id]
                    action_type = "WAIT"
                else:
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
        self.vision.schedules = robot_schedules
        return robot_schedules

    def generate_p2p_movement_instructions(self, robot_schedules):
        instructions_set = []
        for robot_id, rschedule in enumerate(robot_schedules):
            if not rschedule:
                instructions_set.append([])
                continue
            instructions = []
            prev_dir = (0, 0)
            for i in range(len(rschedule)-1):
                next_action = rschedule[i+1]['action']
                path = self.vision.shortest_paths.get((rschedule[i]['location'], rschedule[i+1]['location']))
                if path is None:
                    path = nx.shortest_path(self.vision.graph, source=rschedule[i]['location'], target=rschedule[i+1]['location'], weight='weight')
                    self.vision.shortest_paths[rschedule[i]['location'], rschedule[i+1]['location']] = path
                if not path:
                    continue
                if len(path) > 1:
                    step = 0
                    while step < len(path)-1:
                        dx = path[step+1][1] - path[step][1]  # 1 for x since grid is (row,col)
                        dy = path[step+1][0] - path[step][0]
                        magnitude = max(abs(dx), abs(dy))
                        curr_dir = (dx/magnitude, dy/magnitude) if magnitude > 0 else (0, 0)
                        
                        turn_angle = self.vision.calculate_turn_angle(prev_dir, curr_dir)
                        if turn_angle < 0:
                            instructions.append(f"RIGHT+{abs(int(turn_angle))}")
                        elif turn_angle > 0:
                            instructions.append(f"LEFT+{abs(int(turn_angle))}")

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

                        move_duration = nx.shortest_path_length(self.vision.graph, source=path[step], target=path[step+n], weight='weight') * self.vision.MOVE_DURATION_MS_PER_CM
                        instructions.append(f"MOVE+{int(move_duration)}")
                        step += n
                        prev_dir = curr_dir

                    if next_action == "PICKUP" or next_action == "DROPOFF":
                        instructions.append("RIGHT+360")
            instructions_set.append(instructions)
        return instructions_set

    def send_instructions(self, instructions_set):
        """Send instructions to all robots (synchronous version)"""
        threads = []
        for robot_id, instructions in enumerate(instructions_set):
            robot_name = f"robot {robot_id+1}"
            if robot_name in self.robots and instructions:
                thread = threading.Thread(
                    target=self.send_instructions_to_robot,
                    args=(robot_name, instructions, robot_id+1)
                )
                thread.start()
                threads.append(thread)
        
        for thread in threads:
            thread.join()
        
        print("All robots completed their instructions or encountered errors")

    def send_instructions_to_robot(self, robot_name, instructions, robot_id):
        """Send instructions to a specific robot"""
        robot = self.robots[robot_name]
        
        if not robot.connected and not robot.connect():
            print(f"Failed to connect to robot {robot_id}, cannot send instructions")
            return
            
        print(f"Sending {len(instructions)} instructions to robot {robot_id}")
        
        for i, instruction in enumerate(instructions):
            try:
                print(f"Sending: {instruction}")
                result = robot.send_command(instruction)
                print(f"Result: {result}")
            except Exception as e:
                print(f"Error at instruction {i+1}: {e}")
                break
                
        print(f"Completed instructions for robot {robot_id}")

def main():
    HTTPCoordinator()

if __name__ == '__main__':
    main()