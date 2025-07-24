import os
import subprocess
import tempfile
import json
from typing import List, Dict, Tuple, Optional
import numpy as np
from collections import defaultdict
import re

# All grid coordinates (row, col) refer to the center of the grid cell, i.e., (row + 0.5, col + 0.5) in world units.

class MAPF_Wrapper:    
    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.mapf_pc_path = os.path.join(os.path.dirname(current_dir), "MAPF-PC")
        self.cbs_executable = os.path.join(self.mapf_pc_path, "bin", "cbs.exe")
        
        if not os.path.exists(self.cbs_executable):
            raise FileNotFoundError(f"CBS executable not found at {self.cbs_executable}")
    
    def create_map_file(self, grid_size, obstacles = []):
        map_file = tempfile.NamedTemporaryFile(mode='w', suffix='.map', delete=False)
        
        rows, cols = grid_size
        map_file.write(f"{rows},{cols}\n")
        
        for r in range(rows):
            row = ""
            for c in range(cols):
                if (r, c) in obstacles:
                    row += "@"  # Obstacle
                else:
                    row += "."  # Reachable cell
            map_file.write(row + "\n")
        
        map_file.close()
        return map_file.name
    
    def convert_schedule_to_agents_file(self, schedules):
        agents_file = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False)

        agents_file.write(f"{len(schedules)} # num of agents\n")
        agents_file.write("# Format: num_of_goals sx sy g1x g1y g2x g2y ...\n")

        for robot_id in sorted(schedules.keys(), key=lambda x: int(x)):
            schedule = schedules[robot_id]
            waypoints = []
            # last_dropoff_idx = -1
            # for idx, step in enumerate(schedule):
            #     if step['action'] == 'DROPOFF':
            #         last_dropoff_idx = idx
            # Only include waypoints up to and including the last DROPOFF (Don't return to start location) (Handled by coordinator.run_solver)
            for i, step in enumerate(schedule):
                if step['location'] is not None:
                    waypoints.append(step['location'])

            # Remove duplicates while preserving order
            unique_waypoints = []
            for wp in waypoints:
                if wp not in unique_waypoints:
                    unique_waypoints.append(wp)
            if unique_waypoints:
                start = unique_waypoints[0]
                goals = unique_waypoints[1:]
                line = f"{len(goals)}\t{int(start[1])}\t{int(start[0])}"
                for g in goals:
                    line += f"\t{int(g[1])}\t{int(g[0])}"
                agents_file.write(line + "\n")
            else:
                agents_file.write("0\t0\t0\n")
        agents_file.close()
        # with open(agents_file.name, 'r') as f:
        #     print("Generated agents file:")
        #     print(f.read())
        return agents_file.name
    
    def parse_cbs_stdout(self, stdout):
        paths = {}
        
        lines = stdout.split('\n')
        cleaned_lines = []
        for line in lines:
            line = line.strip()
            if line.startswith('Agent '):
                cleaned_lines.append(line)
            elif line and cleaned_lines and not line.startswith('CG with'):
                cleaned_lines[-1] += ' ' + line
        # if not cleaned_lines:
            # print("CLEANED LINES: (empty)")
        # else:
            # print("CLEANED LINES:")
            # for cl in cleaned_lines:
            #     print(cl)
        for line in cleaned_lines:
            if line.startswith('Agent '):
                agent_match = re.match(r'Agent (\d+)', line)
                if agent_match:
                    agent_id = agent_match.group(1)
                    current_path = []
                    path_part = line.split(': ', 1)
                    if len(path_part) > 1:
                        path_str = path_part[1]
                        for step in path_str.split('->'):
                            step = step.strip()
                            if not step:
                                continue
                            # Match (x, y)@t or (x, y)@t*
                            m = re.match(r'\((\d+), (\d+)\)@(\d+)\*?', step)
                            if m:
                                row, col, t = int(m.group(1)), int(m.group(2)), int(m.group(3))
                                # MAPF-PC uses (col, row)/(x, y) format, convert to (time, row, col)
                                current_path.append((t, row, col))
                    
                    # Remove everything after the first step where time decreases
                    if current_path:
                        final_path = []
                        prev_time = -1
                        for step in current_path:
                            time = step[0]
                            if time < prev_time:
                                break
                            final_path.append(step)
                            prev_time = time
                        paths[agent_id] = final_path
        return paths

    def run_cbs_solver(self, map_file, agents_file, output_file = None, timeout = 300, num_agents = 1):
        if output_file is None:
            output_file = tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False).name
        
        cmd = [
            self.cbs_executable,
            "-m", map_file,
            "-a", agents_file,
            "-o", output_file,
            "-k", str(num_agents),
            "-t", str(timeout),
            "-s", "1"
        ]
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout + 10
            )
            
            # print("\n[CBS STDOUT]\n" + result.stdout)
            # print("\n[CBS STDERR]\n" + result.stderr)
            
            paths = self.parse_cbs_stdout(result.stdout)
            # print("CBS PATHS:", paths)
            
            success = result.returncode == 0
            output = result.stdout
            error = result.stderr
            
            return {
                'success': success,
                'output': output,
                'error': error,
                'paths': paths,
                'output_file': output_file
            }
            
        except subprocess.TimeoutExpired:
            return {
                'success': False,
                'output': '',
                'error': f'Solver timed out after {timeout} seconds',
                'paths': {},
                'output_file': output_file
            }
        except Exception as e:
            return {
                'success': False,
                'output': '',
                'error': str(e),
                'paths': {},
                'output_file': output_file
            }

    
    def generate_collision_free_paths(self, schedules, grid_size, obstacles = []):
        # print(f"[MAPF_Wrapper] Grid size: {grid_size}")
        # print(f"[MAPF_Wrapper] Obstacles: {obstacles}")

        map_file = self.create_map_file(grid_size, obstacles)
        agents_file = self.convert_schedule_to_agents_file(schedules)

        # try:
        #     with open(map_file, 'r') as f:
                # print("=== MAPF-PC Map File ===")
                # print(f.read())
        # except Exception as e:
        #     print(f"[DEBUG][ERROR] Could not read map file: {e}")
        # try:
        #     with open(agents_file, 'r') as f:
                # print("=== MAPF-PC Agents File ===")
                # print(f.read())
        # except Exception as e:
        #     print(f"[DEBUG][ERROR] Could not read agents file: {e}")


        try:
            result = self.run_cbs_solver(map_file, agents_file, num_agents=len(schedules))

            if result['success']:
                print("CBS solver completed successfully!")
                print(f"Generated paths for {len(result['paths'])} agents")
                collision_free_paths = {int(robot_id): path for robot_id, path in result['paths'].items()}
                return collision_free_paths
            else:
                print(f"CBS solver failed: {result['error']}")
                return {}
        except Exception as e:
            print(f"[DEBUG][ERROR] Could not run CBS solver: {e}")
            return {}
        finally:
            try:
                os.unlink(map_file)
                os.unlink(agents_file)
            except:
                pass


    # def postprocess_paths_replanning(self, collision_free_paths, _schedules, grid_size, obstacles):
    #     """
    #         collision_free_paths: {robot_id: [(time, row, col), ...]}
    #         schedules: Original robot schedules from MRTASolver {robot_id: [{'time': time, 'action': action, 'location': (row, col), 'task_id': task_id}, ...]}
    #     """
    #     schedules = _schedules.copy()
    #     # Convert paths to include orientation: (time, row, col, orientation)
    #     validated_paths = {}
    #     for robot_id, path in collision_free_paths.items():
    #         validated_paths[robot_id] = [(step[0], step[1], step[2], None) for step in path]
    #         validated_paths[robot_id][0][3] = (1, 0)
        
    #     unprocessed_paths = validated_paths.copy() # paths starting from the location each robot occupied at the start of the next turn (the robot has turned but hasn't moved yet. The first entry should be at last_validated_time)
    #     robot_orientations_before_turn = {robot_id: (1,0) for robot_id in collision_free_paths.keys()} # directions of each robot after the last validated time (after turns inserted). 
    #     latest_validated_time = 0 # time at which the last inserted turn was completed (and robot is in new orientation) after the shifted path was checked for collisions and replanned if collisions were found
    #     completed_tasks = {robot_id: set() for robot_id in collision_free_paths.keys()} # {robot_id: set of (location, task_id)} of tasks reached
    #     iteration = 0
    #     while iteration < 50:
    #         print(f"========================= Iteration {iteration} =========================\n")
    #         print(latest_validated_time)
    #         iteration += 1
    #         iteration_needed_replanning = False


    #         print("\nValidated paths before this iteration:")
    #         for robot_id, path in validated_paths.items():
    #             path = validated_paths[robot_id]
    #             print(f"  Robot {robot_id}: {[(step[0], step[1], step[2], step[3]) for step in path]}")

    #         print(f"\nUnprocessed paths before this iteration: ")
    #         for robot_id, path in unprocessed_paths.items():
    #             print(f"  Robot {robot_id}: {[(step[0], step[1], step[2], step[3]) for step in path]}")



    #         # remap task times to the time the robot would have reached the task location using CBS solver's discrete time steps
    #         for robot_id, schedule in schedules.items():
    #             path = validated_paths[robot_id]
    #             last_matched_idx = 0
    #             for i, task in enumerate(schedule[1:]):
    #                 for j in range(last_matched_idx, len(path)):
    #                     step = path[j]
    #                     if (step[1], step[2]) == task['location']:
    #                         task['time'] = step[0]
    #                         last_matched_idx = j
    #                         break

    #         next_turn_time, next_turns = self.find_next_turn_time(unprocessed_paths, robot_orientations_before_turn) # time at which the robot reached the coordinate it would need to turn at
    #         print(f"Next turns: {next_turns}")
    #         print(f"Next turn time: {next_turn_time}")

    #         if not next_turns: #stop condition when no more turns need to be checked
    #             print("No more turns to check")
    #             break

    #         #add waypoints reached between last_validated_time and the turn to completed_tasks.
    #         for robot_id, path in validated_paths.items():
    #             ntt = next_turn_time
    #             if ntt > len(path):
    #                 ntt = len(path)
    #             for i, step in enumerate(path[latest_validated_time:ntt]):
    #                 for task in schedules[robot_id]:
    #                     if (task['location'], task['time']) == ((step[1], step[2]), latest_validated_time + i):
    #                         completed_tasks[robot_id].add((task['location'], task['task_id']))
    #                         break
    #         # print(f"Completed tasks: {[(task['location'], task['task_id']) for robot_id in completed_tasks for task in schedules[robot_id] if (task['location'], task['task_id']) in completed_tasks[robot_id]]}")
            
    #         robot_orientations_after_turn = {robot_id: robot_orientations_before_turn[robot_id] for robot_id in robot_orientations_before_turn.keys()}

    #         for robot_id, turn in next_turns.items():
    #             coordinate, angle, new_orientation = turn['position'], turn['angle'], turn['new_orientation']
    #             robot_orientations_after_turn[robot_id] = new_orientation
    #             num_steps = int(abs(angle) // 90)
    #             turn_index = next((i for i, step in enumerate(validated_paths[robot_id]) if step[0] == next_turn_time), None)
                
    #             # Calculate intermediate orientations for each 90° step
    #             current_orientation = robot_orientations_before_turn[robot_id]
    #             turning_steps = []
                
    #             dir_to_angle = {
    #                 (0, 1): 0,    # Right (increasing col)
    #                 (1, 0): 90,   # Down (increasing row)
    #                 (0, -1): 180, # Left (decreasing col)
    #                 (-1, 0): 270  # Up (decreasing row)
    #             }
    #             angle_to_dir = {v: k for k, v in dir_to_angle.items()}
                
    #             for i in range(num_steps + 1):
    #                 if i == 0:
    #                     # First step: still at original orientation
    #                     turning_steps.append((next_turn_time + i, coordinate[0], coordinate[1], current_orientation))
    #                 else:
    #                     # Calculate intermediate orientation after i*90 degrees
    #                     curr_angle = dir_to_angle.get(current_orientation, 0)
    #                     intermediate_angle = (curr_angle + (90 if angle > 0 else -90) * i) % 360
    #                     intermediate_orientation = angle_to_dir.get(intermediate_angle, current_orientation)
    #                     turning_steps.append((next_turn_time + i, coordinate[0], coordinate[1], intermediate_orientation))
                
    #             # Shift remaining steps and update their orientations
    #             remaining_steps = [(step[0] + num_steps, step[1], step[2], new_orientation) for step in validated_paths[robot_id][turn_index+1:]]
    #             unprocessed_paths[robot_id] = turning_steps + remaining_steps
    #             print(f"\nInserted {num_steps} turning steps for Robot {robot_id} at {coordinate} (angle={angle})")


    #         next_next_turn_time, next_next_turns = self.find_next_turn_time(unprocessed_paths, robot_orientations_after_turn)

    #         if next_next_turns: # Only check times from next_turn_time to next_next_turn_time
    #             time_window = set(range(next_turn_time, next_next_turn_time + 1))
    #         else:
    #             # If no next turn, check from next_turn_time to end of all unprocessed paths
    #             max_time = max((max([step[0] for step in path], default=next_turn_time) for path in unprocessed_paths.values()), default=next_turn_time)
    #             time_window = set(range(next_turn_time, max_time + 1))

    #         collision_check_dict = {} # {robot_id: {time: (row, col)}}
    #         for robot_id, path in unprocessed_paths.items():
    #             collision_check_dict[robot_id] = {step[0]: (step[1], step[2]) for step in path}
    #         earliest_collision_time = float('inf')
    #         for t in sorted(time_window):
    #             positions = {}
    #             print_positions = {}
    #             for robot_id, time_pos in collision_check_dict.items():
    #                 if t in time_pos:
    #                     pos = time_pos[t]
    #                     print_positions[robot_id] = pos
    #             print(f"Collision check at time {t}: {print_positions}")
    #             for robot_id, time_pos in collision_check_dict.items():
    #                 if t in time_pos:
    #                     pos = time_pos[t]
    #                     if pos in positions.values():
    #                         earliest_collision_time = min(earliest_collision_time, t)
    #                     positions[robot_id] = pos

    #         # Collision handling
    #         if earliest_collision_time < float('inf'):
    #             iteration_needed_replanning = True
    #             print(f"Collision detected at time {earliest_collision_time}")

    #             # could update last_validated_time to collision_time-1, but this might not give enough space for replanning to find a solution. Instead, replan from the start of the turn
    #             schedules_to_be_replanned = {}
    #             for robot_id, schedule in schedules.items():
    #                 # Find the step at next_turn_time
    #                 step = next((s for s in validated_paths[robot_id] if s[0] == next_turn_time), None)
    #                 if step is None:
    #                     step = validated_paths[robot_id][-1]
    #                 location_at_next_turn_time = (step[1], step[2])
    #                 start_task = {'time': next_turn_time, 'action': 'WAIT', 'location': location_at_next_turn_time, 'task_id': None}
    #                 rest_of_schedule = []
    #                 for task in schedule:
    #                     if (task['location'], task['task_id']) not in completed_tasks[robot_id]:
    #                         rest_of_schedule.append(task)

    #                 schedules_to_be_replanned[robot_id] = [start_task] + rest_of_schedule
    #             if iteration_needed_replanning:
    #                 print("\nReplanning required due to collision. Schedules to be replanned:")
    #                 for robot_id, schedule in schedules_to_be_replanned.items():
    #                     print(f"  Robot {robot_id}: {[ (task['location'], task['time']) for task in schedule ]}")

    #             replanned_paths = self.generate_collision_free_paths(schedules_to_be_replanned, grid_size, obstacles)
    #             for robot_id, path in replanned_paths.items():
    #                     replanned_paths[robot_id] = [(step[0] + next_turn_time, step[1], step[2]) for step in path]
    #             unprocessed_paths = replanned_paths
    #         if iteration_needed_replanning:
    #             continue # don't update orientations or latest_validated_time, begin new iteration from same time with new replanned paths (robots in same orientation as they were when they arrived at the next turn)
    #         else:
    #             for robot_id, path in unprocessed_paths.items():
    #                 idx = next((i for i, step in enumerate(path) if step[0] == next_turn_time), None)
    #                 unprocessed_paths[robot_id] = path[idx:]
    #             for robot_id, path in validated_paths.items():
    #                 idx = next((i for i, step in enumerate(path) if step[0] == next_turn_time), None) # idx is the time the robot reached the next turn coordinate
    #                 validated_paths[robot_id] = path[:idx] + unprocessed_paths[robot_id] # unprocessed paths are either shifted if robot was part of next_turns or just the path after the turn

                
    #             robot_orientations_before_turn = robot_orientations_after_turn # for next iteration
    #             # latest_validated_time = int(next_turn_time + min(abs(turn['angle']) for turn in next_turns.values()) // 90)
    #             latest_validated_time = next_turn_time
    #             print(f"Latest validated time: {latest_validated_time}")  


    #         print(f"\nUnprocessed paths after this iteration: ")
    #         for robot_id, path in unprocessed_paths.items():
    #             print(f"  Robot {robot_id}: {[(step[0], step[1], step[2], step[3]) for step in path]}")
            
    #         print("\nUpdated validated paths after this iteration:")
    #         for robot_id, path in validated_paths.items():
    #             path = validated_paths[robot_id]
    #             print(f"  Robot {robot_id}: {[(step[0], step[1], step[2], step[3]) for step in path]}")

    #     print(f"Final validated paths: {validated_paths}")
    #     return validated_paths


    # def find_next_turn_time(self, paths, initial_orientations):
    #     """Pass in paths, ensure the path starts after the last validated time. initial_orientations is a dict of robot_id to orientation after inserting turns at last validated time
    #         Returns dict {robot_id: (time, position, angle, curr_orientation)} of next turn in schedule. Time is when the robot reached the coordinate that it would have started turning at.
    #         If multiple robots turn at the same time, return all of them and shift the paths of all turning robots at the next_turn_time
    #     """
    #     next_turns = {}
    #     next_turn_time = 100000
    #     for robot_id, path in paths.items():
    #         curr_orientation = initial_orientations[robot_id]
    #         for i in range(len(path)-1):
    #             curr_location = (path[i][1], path[i][2])
    #             next_location = (path[i+1][1], path[i+1][2])

    #             drow = next_location[0] - curr_location[0]
    #             dcol = next_location[1] - curr_location[1]

    #             if drow == 0 and dcol == 0:
    #                 continue
    #             elif drow == 0:
    #                 next_orientation = (0, 1 if dcol > 0 else -1)
    #             elif dcol == 0:
    #                 next_orientation = (1 if drow > 0 else -1, 0)
    #             else: # Diagonal movement not allowed
    #                 next_orientation = (drow / abs(drow), dcol / abs(dcol))

    #             if next_orientation != curr_orientation:
    #                 # Map directions to angles (0=Up, 90=Right, 180=Down, 270=Left) based on absolute view of the environment (top left coordinate is (0, 0))
    #                 dir_to_angle = {
    #                     (0, 1): 0,    # Right (increasing col)
    #                     (1, 0): 90,   # Down (increasing row)
    #                     (0, -1): 180, # Left (decreasing col)
    #                     (-1, 0): 270  # Up (decreasing row)
    #                 }
    #                 curr_angle = dir_to_angle.get(curr_orientation, 0)
    #                 next_angle = dir_to_angle.get(next_orientation, 0)
                    
    #                 # Calculate the shortest angle difference
    #                 angle = next_angle - curr_angle
    #                 if angle > 180:
    #                     angle -= 360
    #                 elif angle < -180:
    #                     angle += 360

    #                 if angle != 0:
    #                     if path[i][0] <= next_turn_time:
    #                         next_turn_time = path[i][0]
    #                         next_turns[robot_id] = {
    #                             'time': int(path[i][0]),
    #                             'position': curr_location,
    #                             'angle': int(angle),
    #                             'new_orientation': next_orientation
    #                         }
    #                         # break # find at most one turn per robot
    #             curr_orientation = next_orientation
    #     to_delete = [robot_id for robot_id, turn in next_turns.items() if turn['time'] > next_turn_time]
    #     for robot_id in to_delete:
    #         del next_turns[robot_id]
    #     return int(next_turn_time), next_turns
    
