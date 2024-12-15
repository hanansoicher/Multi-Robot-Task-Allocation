import numpy as np
import VideoToGraph as v2g
import time
import cv2 as cv
from util import UtilityFunctions as uf
from Graph import Graph as gr
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
                instructions = central_node.run_solver()
                central_node.send_instructions(instructions)
        
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
        self.mrta_solver = self.init_mrta_solver()
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
            Robot(id=0, start=graph.nodes.get((5,5))),
            Robot(id=1, start=graph.nodes.get((15,15)))
        ]
        tasks = [
            Task(id=0, start=graph.nodes.get((5,5)),  end=graph.nodes.get((10,10)), deadline=1000),
            Task(id=1, start=graph.nodes.get((11,8)),  end=graph.nodes.get((5,5)), deadline=2500),
            Task(id=2, start=graph.nodes.get((2,15)),  end=graph.nodes.get((7,6)),  deadline=3000),
            Task(id=3, start=graph.nodes.get((11,12)),  end=graph.nodes.get((12, 4)), deadline=3500),
            Task(id=4, start=graph.nodes.get((17,9)), end=graph.nodes.get((13,19)),  deadline=4000)
        ]
        tasks_stream = [[tasks, 0]]

        ap_set = set()
        for a in agents:
            ap_set.add(a.start)
        for t in tasks:
            ap_set.add(t.start)
            ap_set.add(t.end)

        self.action_points = sorted(list(ap_set))
        num_aps = len(self.action_points)

        # Remap agent and task start/end indices into the action_points indices [0, len(action_points)-1], leaving self.action_points containing the intersection id of the action point
        for a in agents:
            a.start = self.action_points.index(a.start)

        for t in tasks:
            t.start = self.action_points.index(t.start)
            t.end = self.action_points.index(t.end)

        solver_size = len(self.action_points)
        solver_graph = np.ones((solver_size, solver_size), dtype=int) * 10000
        for i in range(solver_size):
            for j in range(solver_size):
                if i == j:
                    solver_graph[i][j] = 0
                else:
                    try:
                        path = gr.safe_astar_path(graph, self.action_points[i], self.action_points[j], gr.heuristic(self.action_points[i], self.action_points[j]))
                        solver_graph[i][j] = int(gr.print_path_weights(graph, path))
                        solver_graph[j][i] = int(gr.print_path_weights(graph, path))
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

        instructions = self.convert_solution_to_schedules(solver.sol)
        return instructions

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

                if location is not None and action_type is not None:
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
                print(f"{step['time']:5d} | {step['location']:8d} | {step['action']:7s} | {task_str}") 
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

                for i in range(len(rschedule)-1):
                    src = rschedule[i]['location']
                    dest = rschedule[i+1]['location']

                    # Compute full path between src and dest
                    path = gr.safe_astar_path(self.vg.graph, src, dest)
                    path = path.nodes
                    print(path)

    def send_instructions(self, instructions):
        for robot, instruction in instructions.items():
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