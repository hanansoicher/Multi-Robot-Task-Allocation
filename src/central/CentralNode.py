# main driver code

#initialize the central node
# initialize camera
# initialize robots
# get graph


# run MRTA solver 
# get the solution
# send instructions to robots

import math
from central import SMTSolver
from central import GraphConstructor
from structs import *
from dijkstar import find_path

class CentralCoordinator:
    def __init__(self):
        self.graph_constructor = GraphConstructor()
        self.smt_solver = SMTSolver()

    def run_solver(self):
        agents = [
            Agent(id=0, start=QRLocation(id=0, x=0, y=0).id),
            Agent(id=1, start=QRLocation(id=1, x=1, y=1).id)
        ]
        tasks = [
            Task(id=0, start=1, end=4, deadline=100), # use actual IDs of waypoint QRLocations
            Task(id=1, start=2, end=5, deadline=100),
            Task(id=2, start=3, end=6, deadline=100),
        ]
        room_graph = self.graph_constructor.build_room_graph()
        solution = self.smt_solver.solve(room_graph=room_graph, num_aps=len(self.graph_constructor.intersections), agents=agents, tasks=tasks)
        return self.get_full_schedule(solution)
    
    def get_full_schedule(self, solution):
        schedule = []
        for robot_id in range(len(solution['agt'])):
            robot_schedule = self.get_schedule_for_robot(solution, robot_id)
            schedule.append(robot_schedule)
        return schedule
    
    def get_schedule_for_robot(self, solution, robot_id):
        agent_data = solution['agt'][robot_id]
        robot_schedule = []

        for i in range(len(agent_data['t'])):
            start = agent_data['t'][i]
            action_id = agent_data['id'][i]
            capacity = agent_data['c'][i]

            # ID 2M + N represents robot N picking up task M
            # ID 2M + N + 1 represents robot N dropping off task M
            if action_id == robot_id:
                action_type = "WAIT_AT_START"
            elif action_id > len(solution['agt']):
                task_num = (action_id - len(solution['agt'])) // 2
                action_type = f"{'PICKUP' if  (action_id - len(solution['agt'])) % 2 == 0 else 'DROPOFF'}_TASK_{task_num}"
            
            robot_schedule.append({
                'time': start,
                'action': action_type,
                'capacity': capacity
            })
        return robot_schedule
    
    def get_location_from_action(self, action_id):
        """Get QRLocation for an action ID"""
        if action_id < len(self.graph_constructor.intersections):  # Robot start position
            return self.intersections[action_id]
        else:  # Task action
            task_num = (action_id - len(self.intersections)) // 2
            is_pickup = (action_id - len(self.intersections)) % 2 == 0
            task_qr = task_num.start if is_pickup else task_num.end
            return self.intersections[task_qr]

    def convert_schedule_to_instructions_for_robot(self, solution, robot_id):
        schedule = self.get_full_schedule(solution)
        schedule = schedule[robot_id]
        instructions = []
        
        for i in range(len(schedule)-1):
            current = schedule[i]
            next = schedule[i+1]
            current_id = self.get_intersection_id_for_action(current['action'])
            next_id = self.get_intersection_id_for_action(next['action'])

            path = find_path(self.graph_constructor.roomGraph, current_id, next_id)
            path_intersections = path.nodes
            
            for idx in range(len(path_intersections)-1):
                cur_loc = self.graph_constructor.intersections[path_intersections[idx]]
                next_loc = self.graph_constructor.intersections[path_intersections[idx+1]]
                
                # Only generate movement instructions for direct paths
                if self.graph_constructor.adjacencyMatrix[path_intersections[idx]][path_intersections[idx+1]] != 10000:
                    dx = next_loc.x - cur_loc.x 
                    dy = next_loc.y - cur_loc.y
                    angle = math.degrees(math.atan2(dy, dx))
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    if abs(angle) > 5:
                        instructions.append({
                            'command': 'TURN',
                            'angle': angle
                        })
                        
                    if distance > 0:
                        instructions.append({
                            'command': 'MOVE',
                            'distance': distance
                        })
            
            if 'PICKUP' in next['action']:
                instructions.append({'command': 'PICKUP'})
            elif 'DROPOFF' in next['action']:
                instructions.append({'command': 'DROPOFF'})
        return instructions