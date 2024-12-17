import sys  
import os 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from SMrTa.MRTASolver import MRTASolver
from SMrTa.MRTASolver.objects import Robot, Task
import numpy as np
import VideoToGraph as v2g
import time
import cv2 as cv
from src.individual.Robot import RobotConnection
from util import UtilityFunctions as uf
from Graph import Graph as gr

from robot import Robot as IndividualNode
import json 
from dotenv import load_dotenv

load_dotenv()

MOVE_DURATION_MS = 13  # 13 ms to move 1 cm
TURN_DURATION_MS = 50  # 50 ms to turn 45 degrees
DIAGONAL_MULTIPLIER = 1.414  # sqrt(2) for diagonal movement

def main():
    web_cam_close = "img/video/webcam_red_close.mov"
    web_cam_further_angle = "img/video/webcam_red_further_angle.mov"
    web_cam_further_top = "img/video/webcam_red_further_top.mov"
    web_cam_distance = "img/video/contour_robots1.mov"
    robots = {
        "robot 1": {
            "name": "robot 1",
            "address": "XX:XX:XX:XX:XX:XX",
            "write_uuid": 11111,
            "START": (30, 2)
        },
        "robot 2": {
            "name": "robot 2",
            "address": "YY:YY:YY:YY:YY:YY",
            "write_uuid": 22222,
            "START": (10, 2)
        }
    }

    # Read robots
    # with open('devices.json', 'r') as f:
    #     robots = json.load(f)['devices']

    video_feed = [web_cam_close, web_cam_further_angle, web_cam_further_top]

    e = os.environ["VIDEO_FEED"]
    print("Searching for env", e)
    video_feed = [int(os.getenv('VIDEO_FEED', 0))]
    video_feed = [web_cam_close, web_cam_further_angle, web_cam_further_top]
    video_feed = [web_cam_close]
    for video_input in video_feed:
        driver_code(video_input, robots)
        print("Video feed completed: ", video_input)

def driver_code(video_input, robots):
    solver_ran = False
    # parse the video adjust parameter to 0 to use webcam 
    central_node = CentralNode(video_input, robots)

    # Initialize
    # await central_node.init()

    while len(central_node.vg.corners) < 4:
        print("Waiting for corners to be detected")
        time.sleep(1)
    print("Corners detected")

    central_node.vg.initialize_tracker(central_node.vg.cap)

    central_node.vg.overlay_update_frame_interval = 15
    last_time = time.time()
    try:
        while True:
            if not central_node.vg.frame_queue.empty():
                frame = central_node.vg.frame_queue.get()
                pos1 = central_node.vg.get_robot_positions(uf.ROBOT_ONE)
                pos2 = central_node.vg.get_robot_positions(uf.ROBOT_TWO)
                instructions = [(uf.ROBOT_ONE, [(0,0)]), (uf.ROBOT_TWO, [(1,1)])]

                if pos1 is not None and pos2 is not None:     
                    instructions = [(uf.ROBOT_ONE, (float(pos1[0]), float(pos1[1]))), (uf.ROBOT_TWO, (float(pos2[0]), float(pos2[1])))]
                central_node.vg.display_robot_instructions(frame, instructions)
                cv.imshow(f'video feed: {video_input}', frame)
            if cv.waitKey(1) == ord('q') or central_node.vg.running == False:
                break
            if cv.waitKey(1) == ord('r'):
                central_node.vg.block_size_cm = (central_node.vg.block_size_cm % 15) + 2

            if cv.waitKey(1) == ord('t'):
                central_node.vg.deadline_threshold = (central_node.vg.deadline_threshold % 2000) - 100 
                for qr_code in central_node.vg.tracked_qr_objects.keys():
                    action_point_node = central_node.vg.get_nearest_node_to_actionpoint(qr_code)
                    if action_point_node:
                        print(f"Action point {qr_code}: {action_point_node}")

            if cv.waitKey(1) == ord('g'):
                central_node.vg.display_grid = not central_node.vg.display_grid

            if cv.waitKey(1) == ord('o'):
                central_node.vg.display_obstacles = not central_node.vg.display_obstacles
            
            if cv.waitKey(1) == ord('p'):
                central_node.vg.display_paths = not central_node.vg.display_paths

            if cv.waitKey(1) == ord('h'):
                central_node.vg.display_HUD = not central_node.vg.display_HUD

            if time.time() - last_time > 2:  
                last_time = time.time()
                # print(robots['robot 1']['START'])
                if not solver_ran:
                    print("Running SMT Solver")
                    solution = central_node.run_solver(robots)
                    solver_ran = True
                    task_schedules = central_node.convert_solution_to_schedules(solution)
                    instructions = central_node.generate_point_to_point_movement_instructions(task_schedules)
                    movement_schedule_1 = central_node.convert_movement_instructions_to_schedule(instructions[0], robots['robot 1']['START'])
                    movement_schedule_2 = central_node.convert_movement_instructions_to_schedule(instructions[1], robots['robot 2']['START'])
                    collisions = central_node.check_path_collisions(movement_schedule_1, movement_schedule_2)
                    if len(collisions) > 0:
                        print("Collisions detected! Choosing robot to wait...")
                        if collisions:
                            collision = collisions[0]  # Handle first collision
                            robot_to_wait = central_node.resolve_collision(task_schedules[0], task_schedules[1], collision, central_node.tasks)
                            if robot_to_wait is not None:
                                # Find index in instruction list where collision occurs 
                                current_time = 0
                                index_to_insert_wait = 0
                                for i, instruction in enumerate(instructions[robot_to_wait]):
                                    if ':' in instruction:
                                        _, duration = instruction.split(':')
                                        duration = int(duration)
                                    else:
                                        duration = TURN_DURATION_MS*8  # P/D duration
                                    current_time += duration
                                    if current_time > collision['time']:
                                        index_to_insert_wait = i
                                        break
                                # Insert wait at the correct index
                                instructions[robot_to_wait].insert(index_to_insert_wait, "W:100")
                                print(f"Robot {robot_to_wait} will wait")
                                print(f"New instructions for robot {robot_to_wait}: {instructions[robot_to_wait]}")
                    else:
                        print("No collisions detected! Sending instructions...")
                    
                    central_node.send_instructions(instructions)

    finally:
        central_node.tear_down()
        print("Final block finished")
    
class CentralNode:

    CORNER_OFFSET_CM = 0.5 # offset from the corner to the edge of our rectangle
    HEIGHT_CM = 61.5 - 2*CORNER_OFFSET_CM  
    LENGTH_CM = 92 - 2*CORNER_OFFSET_CM
    def __init__(self, camera_input, robots):
        self.init_robots(robots) # ensure connection is established
        self.vg = v2g.VideoToGraph(CentralNode.HEIGHT_CM, CentralNode.LENGTH_CM, camera_input, robots)
        self.robot_data = robots

    def init_robots(self, robots):
        self.robots = {}
        for robot_id in robots.keys():
            # Find matching device config
            self.robots[robot_id] = RobotConnection(
                device_address=robots[robot_id]["address"],
                device_name=robot_id,
                characteristic_uuid=robots[robot_id]["write_uuid"],
                start_loc=robots[robot_id]["START"]
            )

    def init_bluetooth_module(self):
        pass

    def run_solver(self, robots):
        # create and get the necessary input for mrta solver
        graph = self.vg.graph
        paths = self.vg.paths

        agents = [
            Robot(id='robot 1', start=robots['robot 1']['START']),
            Robot(id='robot 2', start=robots['robot 2']['START']),
        ]
        tasks = [
            Task(id=0, start=(11,1),  end=(15,2), deadline=1000),
            Task(id=1, start=(2,2),  end=(15,1), deadline=1000),
            Task(id=2, start=(30,4),  end=(7,1),  deadline=1000),
            # Task(id=3, start=(30,4),  end=(16,1),  deadline=1000),
            # Task(id=4, start=(30,4),  end=(16,3),  deadline=2000),
            # Task(id=5, start=(3,2),  end=(9, 4), deadline=2000),
            # Task(id=6, start=(7,9), end=(40,7),  deadline=2000)
        ]
        tasks_stream = [[tasks, 0]]
        self.agents = agents
        self.tasks = tasks

        # Ensure elements are added as the last element
        ap_set = []
        for a in agents:
            if a.start not in ap_set:
                ap_set.append(a.start)
        for t in tasks:
            if t.start not in ap_set:
                ap_set.append(t.start)
            if t.end not in ap_set:
                ap_set.append(t.end)

        self.action_points = ap_set
        num_aps = len(self.action_points)
        print("Action points: ", self.action_points)

        # Remap agent and task start/end indices into the action_points indices [0, len(action_points)-1], leaving self.action_points containing the intersection id of the action point
        for a in agents:
            a.start = self.action_points.index(a.start)

        for t in tasks:
            t.start = self.action_points.index(t.start)
            t.end = self.action_points.index(t.end)

        solver_size = len(self.action_points)
        solver_graph = np.ones((solver_size, solver_size)) * 10000
        for i in range(solver_size):
            for j in range(solver_size):
                if i == j:
                    solver_graph[i][j] = 0
                    solver_graph[j][i] = 0
                else:
                    try:
                        temp_graph = graph.copy()
                        
                        path = gr.safe_astar_path(temp_graph, self.action_points[i], self.action_points[j], gr.heuristic)
                        if path is None:
                            continue
                        # print(path)
                        turning_cost = 0
                        movement_cost = gr.get_path_weights(temp_graph, path)
                        # print(f"Movement cost: {movement_cost}")
                        # Add turning costs to edges along path
                        prev_direction = 0 # North
                        for src, dest in zip(path[:-1], path[1:]):
                            src_pos = temp_graph.nodes[src].get('pos')
                            dest_pos = temp_graph.nodes[dest].get('pos')
                            
                            # Calculate direction vector
                            dx = dest_pos[0] - src_pos[0]
                            dy = dest_pos[1] - src_pos[1]
                            
                            if dx == 0:
                                new_direction = 90 if dy > 0 else 270
                            elif dy == 0:
                                new_direction = 0 if dx > 0 else 180  
                            elif dx > 0:
                                new_direction = 45 if dy > 0 else 315
                            else:
                                new_direction = 135 if dy > 0 else 225
                                
                            angle_diff = abs(new_direction - prev_direction)
                            if angle_diff > 180:
                                angle_diff = 360 - angle_diff
                                
                            turning_cost += (angle_diff / 45) * TURN_DURATION_MS
                                
                            prev_direction = new_direction
                        total_cost = movement_cost + turning_cost
                        # print(f"Total cost: {total_cost}")
                        solver_graph[i][j] = int(total_cost)
                        solver_graph[j][i] = int(total_cost)
                        # print(solver_graph[i][j])
                    except Exception as e:
                        print(e)

        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV',
            agents=agents,
            tasks_stream=tasks_stream,
            room_graph=solver_graph.tolist(),
            capacity=1,
            num_aps=num_aps,
            aps_list=[num_aps],
            fidelity=1,
        )

        if solver.sol is None:
            print("No solution found!")
            return None

        return solver.sol

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

    def generate_point_to_point_movement_instructions(self, robot_schedules):
            self.paths = []
            PICKUP_CMD = "P" # Do a spin
            DROPOFF_CMD = "D" # Do a spin
            FORWARD_CMD = "F"
            TURN_LEFT_CMD = "L"
            TURN_RIGHT_CMD = "R"
            WAIT_CMD = "W"
            instructions_set = []
            for i, rschedule in enumerate(robot_schedules):
                robot_id = "robot 1" if i == 0 else "robot 2"
                instructions = []
                prev_direction = None
                movement_start = False
                # print(f"Robot {robot_id} paths:")
                for i in range(len(rschedule)-1):
                    src = rschedule[i]['location']
                    dest = rschedule[i+1]['location']

                    next_action = rschedule[i+1]['action']
                    if i > 0 and next_action != "WAIT":
                        movement_start = True
                    # Compute full path between src and dest
                    path = gr.safe_astar_path(self.vg.graph, self.vg.graph.nodes[src].get('pos'), self.vg.graph.nodes[dest].get('pos'), gr.heuristic)
                    print(path)
                    if self.vg.paths.get(robot_id) is None:
                        self.vg.paths[robot_id] = []
                    self.vg.paths[robot_id].append(path)

                    if movement_start == False and gr.get_path_weights(self.vg.graph, path) < rschedule[i+1]['time'] - rschedule[i]['time']:
                        instructions.append(f"{WAIT_CMD}:{int(rschedule[i+1]['time'] - rschedule[i]['time'] - gr.get_path_weights(self.vg.graph, path))}")
                    # print(path)

                    if len(path) > 1:
                        step = 0
                        while step < len(path)-1:
                            direction = self.direction_to_turn(path[step], path[step + 1])
                            if prev_direction is not None and prev_direction != direction:
                                direction_angles = {
                                    'N': 0,
                                    'NE': 45,
                                    'E': 90,
                                    'SE': 135,
                                    'S': 180,
                                    'SW': 225,
                                    'W': 270,
                                    'NW': 315
                                }
                                angle = direction_angles[direction] - direction_angles[prev_direction]
                                if angle > 180:
                                    angle = 360 - angle

                                duration = int(abs(angle) / 45 * TURN_DURATION_MS)
                                if angle > 0:
                                    instructions.append(f"{TURN_RIGHT_CMD}:{duration}")
                                elif angle < 0:
                                    instructions.append(f"{TURN_LEFT_CMD}:{duration}")

                            j = 1
                            while (step + j < len(path)-1):
                                if self.direction_to_turn(path[step + j], path[step + j + 1]) == direction:
                                    j += 1
                                else:
                                    break

                            instructions.append(f"{FORWARD_CMD}:{MOVE_DURATION_MS * j}")
                            step += j
                            prev_direction = direction

                    # After movement
                    if next_action == "PICKUP":
                        instructions.append(PICKUP_CMD)
                    elif next_action == "DROPOFF":
                        instructions.append(DROPOFF_CMD)                        

                instructions_set.append(instructions)
                print(f"Robot {robot_id} Instructions: {instructions}")
            return instructions_set
    
    def convert_movement_instructions_to_schedule(self, instruction_set, start_pos):
        schedule = []
        current_time = 0
        current_pos = start_pos
        current_direction = 'N'
        
        directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']

        
        direction_vectors = {
            'N': (-1, 0),
            'NE': (-1, 1),
            'E': (0, 1),
            'SE': (1, 1),
            'S': (1, 0),
            'SW': (1, -1),
            'W': (0, -1),
            'NW': (-1, -1)
        }
        
        # Add initial position
        schedule.append({
            'time': current_time,
            'location': current_pos
        })
        
        for instruction in instruction_set:
            if ':' in instruction:
                cmd, duration = instruction.split(':')
                duration = int(duration)
            else:
                cmd = instruction
                duration = TURN_DURATION_MS*8  # 360 degree spin for P/D
                
            if cmd == 'F':
                # Calculate new position based on direction and duration
                move_vector = direction_vectors[current_direction]
                
                distance = duration / MOVE_DURATION_MS
                
                if move_vector[0] != 0 and move_vector[1] != 0:
                    distance = distance * DIAGONAL_MULTIPLIER
                
                dx = move_vector[0] * distance
                dy = move_vector[1] * distance

                current_pos = (current_pos[0] + dx, current_pos[1] + dy)
                current_time += duration
                
            elif cmd in ['L', 'R']: # Position stays the same, turn in place
                num_turns = duration // TURN_DURATION_MS
                
                current_idx = directions.index(current_direction)
                
                if cmd == 'L':
                    new_idx = (current_idx - num_turns) % 8
                else:
                    new_idx = (current_idx + num_turns) % 8
                    
                current_direction = directions[new_idx]
                current_time += duration
                    
            elif cmd in ['P', 'D']:
                # Pickup/Dropoff adds time but doesn't change position
                current_time += TURN_DURATION_MS*8  # Duration for 360 degree spin
            elif cmd == 'W':
                # Initial wait time
                current_time += duration
            schedule.append({
                'time': current_time,
                'location': current_pos
            })
        
        return schedule

    def check_path_collisions(self, schedule1, schedule2, safety_radius=5):
        collisions = []
        
        # Check each pair of path segments
        for i in range(len(schedule1) - 1):
            for j in range(len(schedule2) - 1):
                r1_start = schedule1[i]
                r1_end = schedule1[i + 1]
                r2_start = schedule2[j]
                r2_end = schedule2[j + 1]
                
                # Find overlapping time range
                t_start = max(r1_start['time'], r2_start['time'])
                t_end = min(r1_end['time'], r2_end['time'])
                
                if t_start > t_end:
                    continue  # No time overlap
                    
                # Check positions at each time step
                for step in range(t_end - t_start):
                    t = t_start + (t_end - t_start) * (step / (t_end - t_start))
                    
                    # Interpolate positions of both robots at time t
                    for start, end, current_t in [(r1_start, r1_end, t), (r2_start, r2_end, t)]:
                        # Skip if time is outside segment
                        if current_t < start['time'] or current_t > end['time']:
                            continue
                            
                        # If robot is stationary
                        if start['time'] == end['time']:
                            pos1 = r1_start['location']
                            pos2 = r2_start['location']
                        else:
                            # Calculate interpolation factors
                            t1 = (t - r1_start['time']) / (r1_end['time'] - r1_start['time']) if (r1_end['time'] - r1_start['time']) != 0 else 0
                            t2 = (t - r2_start['time']) / (r2_end['time'] - r2_start['time']) if (r2_end['time'] - r2_start['time']) != 0 else 0
                            
                            # Interpolate positions
                            x1 = r1_start['location'][0] + (r1_end['location'][0] - r1_start['location'][0]) * t1
                            y1 = r1_start['location'][1] + (r1_end['location'][1] - r1_start['location'][1]) * t1
                            x2 = r2_start['location'][0] + (r2_end['location'][0] - r2_start['location'][0]) * t2
                            y2 = r2_start['location'][1] + (r2_end['location'][1] - r2_start['location'][1]) * t2
                            
                            pos1 = (x1, y1)
                            pos2 = (x2, y2)
                    
                    # Calculate distance between robots
                    dx = pos1[0] - pos2[0]
                    dy = pos1[1] - pos2[1]
                    distance = (dx*dx + dy*dy) ** 0.5
                    
                    if distance < safety_radius:
                        midpoint = ((pos1[0] + pos2[0]) / 2, (pos1[1] + pos2[1]) / 2)
                        collisions.append({
                            'time': t,
                            'location': midpoint,
                            'segment1': (i, i+1),
                            'segment2': (j, j+1)
                        })
                        print(f"Collision detected at time {t} between robots while robot 1 is moving from {r1_start['location']} to {r1_end['location']} and robot 2 is moving from {r2_start['location']} to {r2_end['location']}")
                        break  # Found collision for this segment pair
        
        return collisions

    def resolve_collision(self, task_schedule1, task_schedule2, collision, tasks):
        """
        Resolves collision by checking task deadlines and inserting wait if possible.
        
        Args:
            task_schedule1, task_schedule2: Task schedules from SMT solver (pickup/dropoff)
            collision: Dict with collision info
            tasks: List of Task objects with deadlines
            
        Returns:
            robot_to_wait or none if cannot resolve
        """
        collision_time = collision['time']
        
        # Find active tasks and their deadlines for both robots
        task1, task1_deadline = None, float('inf')
        task2, task2_deadline = None, float('inf')
        
        for step in task_schedule1:
            if step['task_id'] is not None and step['action'] == 'PICKUP':
                # Look for matching dropoff
                for dropoff in task_schedule1:
                    if (dropoff['task_id'] == step['task_id'] and dropoff['action'] == 'DROPOFF' and step['time'] <= collision_time <= dropoff['time']):
                        task1 = step['task_id']
                        task1_deadline = dropoff['time']
                        break
                if task1 is not None:
                    break

        for step in task_schedule2:
            if step['task_id'] is not None and step['action'] == 'PICKUP':
                # Find matching dropoff
                for dropoff in task_schedule2:
                    if (dropoff['task_id'] == step['task_id'] and dropoff['action'] == 'DROPOFF' and step['time'] <= collision_time <= dropoff['time']):
                        task2 = step['task_id']
                        task2_deadline = dropoff['time']
                        break
                if task2 is not None:
                    break
        
        # Calculate task completion times after potential wait
        task1_completion = task1_deadline if task1 is not None else float('inf')
        task2_completion = task2_deadline if task2 is not None else float('inf')

        wait_duration = 200  # Base wait time for robot to pass
        
        time_until_deadline1 = tasks[task1].deadline - task1_completion if task1 is not None else float('inf')
        time_until_deadline2 = tasks[task2].deadline - task2_completion if task2 is not None else float('inf')
        
        if time_until_deadline1 > wait_duration and (time_until_deadline1 > time_until_deadline2 or time_until_deadline2 < wait_duration):
            # Modify robot 1's schedule to wait
            return 0
        elif time_until_deadline2 > wait_duration:
            # Modify robot 2's schedule
            return 1  
        return None

    def send_instructions(self, instructions_set):
        for i, instructions in enumerate(instructions_set):
            robot_id = "robot 1" if i == 0 else "robot 2"
            for instruction in instructions:
                if ':' in instruction:
                    command, duration = instruction.split(':')
                    duration = int(duration)
                else:
                    command = instruction
                # if command == 'F':
                #     self.robots[robot_id].move(duration)
                # elif command == 'L':
                #     self.robots[robot_id].turn(duration)
                # elif command == 'R':
                #     self.robots[robot_id].turn(duration)
                # elif command == 'P' or command == 'D':
                #     self.robots[robot_id].turn(TURN_DURATION_MS*8)  # 360 degree spin
                # elif command == 'W':
                #     self.robots[robot_id].wait(duration)
                print(f"Sent instruction {instruction} to {robot_id}")
            print(f"Finished sending instructions to {robot_id}")

    def direction_to_turn(self, src, dest):
        if dest[1] == src[1] and dest[0] < src[0]:
            return 'N'
        elif dest[1] == src[1] and dest[0] > src[0]:
            return 'S'
        elif dest[0] == src[0] and dest[1] > src[1]:
            return 'E'
        elif dest[0] == src[0] and dest[1] < src[1]:
            return 'W'
        elif dest[0] > src[0] and dest[1] > src[1]:
            return 'SE'
        elif dest[0] > src[0] and dest[1] < src[1]:
            return 'NE'
        elif dest[0] < src[0] and dest[1] > src[1]:
            return 'SW'
        elif dest[0] < src[0] and dest[1] < src[1]:
            return 'NW'

    def robot_calibration_and_sync(self):
        # ensure that movement is calibrated
        # move forward, orientation etc
        return 1

    def tear_down(self):
        # Stop the thread and release resources 
        self.vg.tear_down()
        if self.vg.thread.is_alive():
            print(f"Thread {self.vg.thread.getName()} is alive: {self.vg.thread.is_alive()}")
            self.vg.thread.join()
        print("Tear down done")

if __name__ == "__main__":
    main()