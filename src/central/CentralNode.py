import numpy as np
import VideoToGraph as v2g
import time
import cv2 as cv
from util import UtilityFunctions as uf
from Graph import Graph as gr
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from SMrTa.MRTASolver import MRTASolver
from SMrTa.MRTASolver.objects import Robot, Task

def main():
    web_cam_close = "img/video/webcam_red_close.mov"
    web_cam_further_angle = "img/video/webcam_red_further_angle.mov"
    web_cam_further_top = "img/video/webcam_red_further_top.mov"
    robots = {
        'robot 1': 'R1:XX', #MAC address
        'robot 2': 'R2:XX:', 
    }
    video_feed = [web_cam_close, web_cam_further_angle, web_cam_further_top]
    for video_input in video_feed:
        driver_code(video_input, robots)
        print("Video feed completed: ", video_input)

def driver_code(video_input, robots):
    solver_ran = False
    # parse the video adjust parameter to 0 to use webcam 
    central_node = CentralNode(video_input, robots)
    while len(central_node.vg.corners) < 4:
        print("Waiting for corners to be detected")
        time.sleep(1)

    central_node.vg.overlay_update_frame_interval = 1
    last_time = time.time()
    try:
        while True:
            if not central_node.vg.frame_queue.empty():
                frame = central_node.vg.frame_queue.get()
                frame = central_node.vg.overlay_text(frame, f"blocksize in cm: {central_node.vg.block_size_cm}", (50,50))
                frame = central_node.vg.overlay_text(frame, f"Update rate: {central_node.vg.overlay_update_frame_interval}", (100,100))
                cv.imshow(f'video feed: {video_input}', frame)
            if cv.waitKey(1) == ord('q') or central_node.vg.running == False:
                break
            if time.time() - last_time > 2:  
                last_time = time.time()
                if not solver_ran:
                    solution = central_node.run_solver()
                    solver_ran = True
                    schedules = central_node.convert_solution_to_schedules(solution)
                    instructions = central_node.generate_point_to_point_movement_instructions(schedules)
                    central_node.send_instructions(instructions)
                    break
        
            if cv.waitKey(1) == ord('r'):
                central_node.vg.block_size_cm = (central_node.vg.block_size_cm % 15) + 2

            if cv.waitKey(1) == ord('t'):
                central_node.vg.overlay_update_frame_interval = (central_node.vg.overlay_update_frame_interval % 20) + 2

    finally:
        central_node.tear_down()
        print("Final block finished")
    
class CentralNode:

    HEIGHT_CM = 75
    LENGTH_CM = 150
    def __init__(self, camera_input, robots):
        self.bluetooth_client = self.init_bluetooth_module()
        self.robots = self.init_robots(robots, self.bluetooth_client) # ensure connection is established
        self.vg = v2g.VideoToGraph(CentralNode.HEIGHT_CM, CentralNode.LENGTH_CM, camera_input)
        self.robot_calibration_and_sync()

    def init_robots(self, robots, bluetooth_client):
        print("initialized robots: ",robots)
        pass

    def init_bluetooth_module(self):
        pass

    def run_solver(self):
        # create and get the necessary input for mrta solver
        graph = self.vg.graph
        paths = self.vg.paths

        print("graph: ", graph)
        print("paths: ", paths)
        try:
            gr.print_path_weights(graph, paths['robot 1'])
        except Exception as e:
            print(e)

        agents = [
            Robot(id=0, start=(10,1)),
            Robot(id=1, start=(1,9)),
        ]
        tasks = [
            Task(id=0, start=(11,1),  end=(15,2), deadline=1000),
            Task(id=1, start=(2,2),  end=(15,1), deadline=1000),
            Task(id=2, start=(33, 4),  end=(7,1),  deadline=1000),
            # Task(id=3, start=(3,2),  end=(9, 4), deadline=3500),
            # Task(id=4, start=(7,9), end=(7,7),  deadline=4000)
        ]
        tasks_stream = [[tasks, 0]]
        self.agents = agents
        self.tasks = tasks
        # ap_set = set()
        # for a in agents:
        #     ap_set.add(graph.nodes[a.start].get('pos'))
        # for t in tasks:
        #     ap_set.add(graph.nodes[t.start].get('pos'))
        #     ap_set.add(graph.nodes[t.end].get('pos'))

        # for a in agents:
        #     ap_set.add(a.start)
        # for t in tasks:
        #     ap_set.add(t.start)
        #     ap_set.add(t.end)
        
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
        print("Action points: ", ap_set)
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
                else:
                    try:
                        path = gr.safe_astar_path(graph, self.action_points[i], self.action_points[j], gr.heuristic)
                        print(path)
                        solver_graph[i][j] = gr.print_path_weights(graph, path)
                        solver_graph[j][i] = gr.print_path_weights(graph, path)
                    except:
                        solver_graph[i][j] = 10000
                        solver_graph[j][i] = 10000

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
            MOVE_DURATION = 200  # time to move between neighboring intersections
            TURN_DURATION = 100  # calculate time to turn 90, 180, 270, 360 degrees
            PICKUP_CMD = "P" # Do a spin
            DROPOFF_CMD = "D" # Do a spin
            FORWARD_CMD = "F"
            TURN_LEFT_CMD = "L"
            TURN_RIGHT_CMD = "R"
            for robot_id, rschedule in enumerate(robot_schedules):
                instructions = []
                prev_direction = None

                print(f"Robot {robot_id} Instructions:")
                for i in range(len(rschedule)-1):
                    src = rschedule[i]['location']
                    dest = rschedule[i+1]['location']
                    next_action = rschedule[i+1]['action']

                    # Compute full path between src and dest
                    path = gr.safe_astar_path(self.vg.graph, self.vg.graph.nodes[src].get('pos'), self.vg.graph.nodes[dest].get('pos'), gr.heuristic)
                    print(path)

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
                                    angle -= 360
                                elif angle <= -180:
                                    angle += 360

                                duration = int(abs(angle) / 45 * TURN_DURATION)
                                if angle > 0:
                                    instructions.append(f"{TURN_RIGHT_CMD}:{duration}")
                                elif angle < 0:
                                    instructions.append(f"{TURN_LEFT_CMD}:{duration}")

                            i = 1
                            while (step + i < len(path)-1):
                                if self.direction_to_turn(path[step + i], path[step + i + 1]) == direction:
                                    i += 1
                                else:
                                    break

                            instructions.append(f"{FORWARD_CMD}:{MOVE_DURATION * i}")
                            step += i
                            prev_direction = direction

                    # After movement
                    if next_action == "PICKUP":
                        instructions.append(PICKUP_CMD)
                    elif next_action == "DROPOFF":
                        instructions.append(DROPOFF_CMD)

                instructions_str = ">".join(instructions)
                print(f"Robot {robot_id} Instruction string:")
                print(instructions_str)
            return instructions

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

    def send_instructions(self, instructions):
        for robot, instruction in instructions:
            self.send_instruction(robot, instruction)
        pass

    def send_instruction(self, robot, instruction):
        print(f"sent to robot: {robot}, instruction: {instruction}")
        return
        self.bluetooth_client.send(robot, instruction)

    def robot_calibration_and_sync(self):
        # ensure that movement is calibrated
        # move forward, orientation etc
        pass

    def tear_down(self):
        # Stop the thread and release resources 
        self.vg.tear_down()
        if self.vg.thread.is_alive():
            print(f"Thread {self.vg.thread.getName()} is alive: {self.vg.thread.is_alive()}")
            self.vg.thread.join()
        print("Tear down done")

if __name__ == "__main__":
    main()