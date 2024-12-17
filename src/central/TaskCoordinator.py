# run MRTA solver 
# get the solution
# send instructions to robots

import os
import sys

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from SMrTa.MRTASolver import MRTASolver
from SMrTa.MRTASolver.objects import Robot, Task

import math
from src.structs import DecisionPoint
from dijkstar import Graph, find_path

class TaskCoordinator:
    def __init__(self):
        self.intersection_locations = list(range(16)) # for 4x4 grid
        self.intersection_graph = None

    def setup_solver_inputs(self):
        self.agents = [
            Robot(id=0, start=0),
            Robot(id=1, start=15)
        ]
        self.tasks = [
            Task(id=0, start=1,  end=14, deadline=1000),
            Task(id=1, start=2,  end=13, deadline=2500),
            Task(id=2, start=7,  end=8,  deadline=3000),
            Task(id=3, start=4,  end=11, deadline=3500),
            Task(id=4, start=10, end=3,  deadline=4000)
        ]

        ap_set = set()
        for a in self.agents:
            ap_set.add(a.start)
        for t in self.tasks:
            ap_set.add(t.start)
            ap_set.add(t.end)

        self.action_points = sorted(list(ap_set))

        # Remap agent and task start/end indices into the action_points indices [0, len(action_points)-1], leaving self.action_points containing the intersection id of the action point
        for a in self.agents:
            a.start = self.action_points.index(a.start)

        for t in self.tasks:
            t.start = self.action_points.index(t.start)
            t.end = self.action_points.index(t.end)

    def run_solver(self, agents, tasks_stream, num_aps, room_graph=None):
        if room_graph is None:
            room_graph = self.create_test_room_graph()

        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV',
            agents=agents,
            tasks_stream=tasks_stream,
            room_graph=room_graph,
            capacity=1,
            num_aps=num_aps, # number of pickup/dropoff locations
            aps_list=[num_aps],
            fidelity=1,
        )
        # MRTASolver output: sol = {'agt': [{'t': [], 'c': [], 'id': []} for i in range(num_agents)]}
        # t: Start time of action, c: Capacity of agent, id: Action/Task ID
        if solver.sol is None:
            print("No solution found!")
            return None
        else:
            return solver.sol
        
    def create_test_room_graph(self):
        print("Maze Layout (4x4 Grid):")
        print("0---1---2---3")
        print("|   |   |   |")
        print("4---5---6---7")
        print("|   |   |   |")
        print("8---9--10--11")
        print("|   |   |   |")
        print("12-13--14--15")
        
        room_count = len(self.intersection_locations)
        intersection_matrix = np.ones((room_count, room_count)) * 10000
        direct_paths = []

        for i in range(room_count):
            intersection_matrix[i][i] = 0

        grid_size = 4
        for row in range(grid_size):
            for col in range(grid_size):
                i = row*grid_size + col
                # Connect right
                if col < grid_size-1:
                    direct_paths.append((i, i+1, 100))
                # Connect below
                if row < grid_size-1:
                    direct_paths.append((i, i+grid_size, 100))

        for start, end, dist in direct_paths:
            intersection_matrix[start][end] = dist
            intersection_matrix[end][start] = dist

        # Build graph for shortest paths
        graph = Graph()
        for start, end, dist in direct_paths:
            graph.add_edge(start, end, dist)
            graph.add_edge(end, start, dist)
        self.intersection_graph = graph # save for path planning to generate instructions

        solver_size = len(self.action_points)
        solver_graph = np.ones((solver_size, solver_size), dtype=int) * 10000
        for i in range(solver_size):
            for j in range(solver_size):
                if i == j:
                    solver_graph[i][j] = 0
                else:
                    try:
                        path = find_path(graph, self.action_points[i], self.action_points[j])
                        solver_graph[i][j] = int(path.total_cost)
                        solver_graph[j][i] = int(path.total_cost)
                    except:
                        solver_graph[i][j] = 10000
                        solver_graph[j][i] = 10000
        print(f"\nAction points: {self.action_points}")
        print(f"\nSolver Graph (Travel times Between Action Points): \n {solver_graph}")
        return solver_graph.tolist()

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

            for i in range(len(rschedule)-1):
                src = rschedule[i]['location']
                dest = rschedule[i+1]['location']
                next_action = rschedule[i+1]['action']

                # Compute full path between src and dest
                path = find_path(self.intersection_graph, src, dest)
                path = path.nodes
                print(path)

                if len(path) > 1:
                    step = 0
                    while step < len(path)-1:
                        direction = self.direction_to_turn(self.get_coordinates_from_id(path[step]), self.get_coordinates_from_id(path[step + 1]))
                        if prev_direction is not None and prev_direction != direction:
                            direction_angles = {
                                'N': 0,
                                'E': 90,
                                'S': 180,
                                'W': 270
                            }
                            angle = direction_angles[direction] - direction_angles[prev_direction]
                            if angle > 180:
                                angle -= 360
                            elif angle <= -180:
                                angle += 360

                            duration = int(abs(angle) / 90 * TURN_DURATION)
                            if angle > 0:
                                instructions.append(f"{TURN_RIGHT_CMD}:{duration}")
                            elif angle < 0:
                                instructions.append(f"{TURN_LEFT_CMD}:{duration}")

                        i = 1
                        while (step + i < len(path)-1):
                            if self.direction_to_turn(self.get_coordinates_from_id(path[step + i]), self.get_coordinates_from_id(path[step + i + 1])) == direction:
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
            print(f"Robot {robot_id} Instructions:")
            print(instructions_str)
        return instructions
    
    def get_coordinates_from_id(self, id):
        # Convert integer id of an intersection to (x, y) coordinates
        grid_size = 4
        x = id % grid_size
        y = id // grid_size
        return (DecisionPoint(id, x, y, "intersection"))
    
    def direction_to_turn(self, src, dest):
        if dest.y == src.y and dest.x < src.x:
            return 'N'
        elif dest.y == src.y and dest.x > src.x:
            return 'S'
        elif dest.x == src.x and dest.y > src.y:
            return 'E'
        elif dest.x == src.x and dest.y < src.y:
            return 'W'
        
    def send_instructions_to_robot(self, robot_id, instructions):
        pass

def main():
    coordinator = TaskCoordinator()
    coordinator.setup_solver_inputs()        
    tasks_stream = [[coordinator.tasks, 0]]
    num_aps = len(coordinator.action_points)
    solution = coordinator.run_solver(coordinator.agents, tasks_stream, num_aps)
    robot_schedules = coordinator.convert_solution_to_schedules(solution)
    coordinator.generate_point_to_point_movement_instructions(robot_schedules)
