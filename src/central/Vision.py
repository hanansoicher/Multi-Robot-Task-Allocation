import sys
import cv2
import numpy as np
import networkx as nx
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication
from UI import UI

class Vision:
    def __init__(self, coordinator, camera_input=0):
        self.cap = cv2.VideoCapture(camera_input)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera/video")

        self.corners = {}
        self.homography = None
        self.grid = None
        self.graph = None
        self.shortest_paths = {}
        
        self.solver_ran = False
        self.schedules = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.MAZE_WIDTH_CM = 72
        self.MAZE_HEIGHT_CM = 45
        self.GRID_SIZE_CM = 1
        self.MOVE_DURATION_MS_PER_CM = 1
        self.TURN_DURATION_MS_PER_DEG = 1

        self.app = QApplication(sys.argv)
        self.ui = UI(self, coordinator)
        self.ui.show()

        self.timer = QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(30)

    def run(self):                    
        ret, frame = self.cap.read()
        if not ret or frame is None:
            print("Cannot read frame, aborting...")
            return
        if not self.corners:
            self.corners = self.find_corners(frame)
            if self.corners and len(self.corners) == 4:
                src_pts = np.float32([
                    self.corners['top_left'], 
                    self.corners['top_right'], 
                    self.corners['bottom_left'], 
                    self.corners['bottom_right']
                ])
                dest_pts = np.float32([
                    [0, 0], 
                    [self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, 0],
                    [0, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM], 
                    [self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM]])
                self.homography = cv2.getPerspectiveTransform(src_pts, dest_pts)
                self.inv_homography = cv2.invert(self.homography)[1]
        
        if self.homography is not None:
            self.grid = self.update_obstacle_grid(frame)
            self.graph = self.update_graph(self.grid)

        viz_frame = self.draw_grid_overlay(frame.copy())
        if self.solver_ran and self.schedules is not None:
            viz_frame = self.draw_assigned_routes(viz_frame, self.schedules)
        else:
            viz_frame = self.draw_selected_waypoints(viz_frame)

        if viz_frame is not None:
            height, width, _ = viz_frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(viz_frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.ui.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def find_corners(self, frame):
        """Find ArUco markers for the corners"""
        corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        corner_pixels = {}
        corner_ids = {96: 'top_left', 97: 'top_right', 98: 'bottom_left', 99: 'bottom_right'}

        if ids is None:
            print("No ArUco markers found")
            return {}

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in corner_ids:
                marker_corners = corners[i][0]
                center = tuple(map(int, np.mean(marker_corners, axis=0)))
                corner_pixels[corner_ids[marker_id]] = center

        if len(corner_pixels) < 4:
            print(f"Only found {len(corner_pixels)} corners")
            return {}

        print("Found all 4 corners")
        return corner_pixels

    def pixel_to_grid(self, x, y):
        if self.homography is not None:
            grid_pos = cv2.perspectiveTransform(np.array([[[x, y]]], dtype='float32'), self.homography)[0][0]
            grid_x = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), self.MAZE_WIDTH_CM - 1))
            grid_y = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), self.MAZE_HEIGHT_CM - 1))
            return (grid_y, grid_x)  # (row, col) for grid indexing
        return None

    def find_robots(self, frame):
        marker_corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        robots = {}
        if ids is not None and self.homography is not None:
            rows, cols = int(self.MAZE_HEIGHT_CM), int(self.MAZE_WIDTH_CM)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                corners = marker_corners[i][0]
                center = np.mean(corners, axis=0)
                if marker_id in [0, 1]:
                        grid_pos = cv2.perspectiveTransform(np.array([[center]]), self.homography)[0][0]
                        grid_x = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), cols - 1))
                        grid_y = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), rows - 1))
                        robots[marker_id] = (grid_y, grid_x) # (row, col) for grid indexing
        return robots

    # def create_grid_with_obstacle_mask(self, frame):
    #     if self.homography is None:
    #         print("Homography not found, find corners and transform perspective before calling this function")
    #         return None

    #     warped = cv2.warpPerspective(frame, self.homography, (int(self.MAZE_WIDTH_CM * self.GRID_SIZE_CM), int(self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM)))
    #     _, black_tape_mask = cv2.threshold(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY), 60, 255, cv2.THRESH_BINARY_INV)
    #     kernel_close = np.ones((3, 3), np.uint8)
    #     black_tape_mask = cv2.morphologyEx(black_tape_mask, cv2.MORPH_CLOSE, kernel_close)
        
    #     rows = int(self.MAZE_HEIGHT_CM)
    #     cols = int(self.MAZE_WIDTH_CM)
        
    #     grid = np.ones((rows, cols), dtype=np.uint8)
    #     robot_coords = self.find_robots(frame)
        
    #     robot_mask = np.zeros((rows, cols), dtype=np.uint8)
    #     for robot_id, (robot_row, robot_col) in robot_coords.items():
    #         robot_radius = int(5 / self.GRID_SIZE_CM)
    #         cv2.circle(robot_mask, (robot_col, robot_row), robot_radius, 1, -1)
        
    #     try:
    #         black_path_mask = cv2.resize(black_tape_mask, (cols, rows))
    #         grid[black_path_mask > 0] = 0
    #         robot_path_overlap = robot_mask & black_path_mask
    #         grid[robot_path_overlap > 0] = 0
            
    #         print(f"Found {np.sum(black_path_mask > 0)} reachable cells on black tape paths")
    #         print(f"Found {np.sum(robot_path_overlap > 0)} cells with robots on paths")
    #     except cv2.error as e:
    #         print(f"Error processing masks: {e}")
    #     return grid

    def update_obstacle_grid(self, frame):
        if self.homography is None:
            print("Homography not found, find corners and transform perspective before calling this function")
            return None

        warped = cv2.warpPerspective(frame, self.homography, (int(self.MAZE_WIDTH_CM * self.GRID_SIZE_CM), int(self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM)))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        # Obstacle color ranges
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel_close = np.ones((5,5), np.uint8)
        kernel_dilate = np.ones((3,3), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        obstacle_mask = cv2.dilate(mask, kernel_dilate, iterations=1)
        rows = int(self.MAZE_HEIGHT_CM)
        cols = int(self.MAZE_WIDTH_CM)
        grid = np.zeros((rows, cols), dtype=np.uint8)
        if obstacle_mask is None:
            return grid
        try:
            obstacle_mask = cv2.resize(obstacle_mask, (cols, rows))
            grid[obstacle_mask > 0] = 1
            dilate_kernel = np.ones((11, 11), np.uint8)  # 5 cm buffer on each side
            dilated_obstacle_mask = cv2.dilate(obstacle_mask, dilate_kernel, iterations=1)
            grid[dilated_obstacle_mask > 0] = 1
        except cv2.error as e:
            print(f"Error resizing obstacle mask: {e}")
        return grid

    def update_graph(self):
        if self.grid is None:
            return None
        graph = nx.Graph()
        rows, cols = self.grid.shape
        for i in range(rows):
            for j in range(cols):
                graph.add_node((i,j))
                for di, dj in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < rows and 0 <= nj < cols and self.grid[ni,nj] == 0 and self.grid[i,j] == 0:
                        # Weight is sqrt(2) for diagonal, 1 for adjacent, multiplied by grid size in cm
                        weight = np.sqrt(2) if abs(di) + abs(dj) == 2 else 1
                        graph.add_edge((i,j), (ni,nj), weight=weight*self.GRID_SIZE_CM)
                    else:
                        graph.add_edge((i,j), (ni,nj), weight=float('inf'))
        return graph
    

    def draw_grid_overlay(self, frame):
        if self.homography is None:
            print("Homography not found, find corners and transform perspective before calling this function")
            return frame
        
        # Pink color in BGR for grid lines, lime boxes for unreachable
        pink = (200, 20, 255)
        lime = (33, 190, 120)
        
        obstacle_img = np.zeros_like(frame)
        grid_img = np.zeros_like(frame)

        if self.grid is not None:
            for i in range(self.grid.shape[0]):
                for j in range(self.grid.shape[1]):
                    if self.grid[i,j] == 1:
                        rect_corners = np.array([
                            [[j * self.GRID_SIZE_CM, i * self.GRID_SIZE_CM]],
                            [[(j+1) * self.GRID_SIZE_CM, i * self.GRID_SIZE_CM]],
                            [[(j+1) * self.GRID_SIZE_CM, (i+1) * self.GRID_SIZE_CM]],
                            [[j * self.GRID_SIZE_CM, (i+1) * self.GRID_SIZE_CM]]
                        ], dtype=np.float32)
                        
                        transformed_corners = cv2.perspectiveTransform(rect_corners, self.inv_homography)
                        points = transformed_corners.reshape((-1,1,2)).astype(np.int32)
                        cv2.fillPoly(obstacle_img, [points], lime)
        
        for y in range(0, int(self.MAZE_HEIGHT_CM) + 1, self.GRID_SIZE_CM):
            pt1 = np.array([[[0, y * self.GRID_SIZE_CM]]], dtype=np.float32)
            pt2 = np.array([[[self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, y * self.GRID_SIZE_CM]]], dtype=np.float32)
            
            pt1_transformed = tuple(map(int, cv2.perspectiveTransform(pt1, self.inv_homography)[0][0]))
            pt2_transformed = tuple(map(int, cv2.perspectiveTransform(pt2, self.inv_homography)[0][0]))
            cv2.line(grid_img, pt1_transformed, pt2_transformed, pink, 1)
        
        for x in range(0, int(self.MAZE_WIDTH_CM) + 1, self.GRID_SIZE_CM):
            pt1 = np.array([[[x * self.GRID_SIZE_CM, 0]]], dtype=np.float32)
            pt2 = np.array([[[x * self.GRID_SIZE_CM, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM]]], dtype=np.float32)
            
            pt1_transformed = tuple(map(int, cv2.perspectiveTransform(pt1, self.inv_homography)[0][0]))
            pt2_transformed = tuple(map(int, cv2.perspectiveTransform(pt2, self.inv_homography)[0][0]))
            cv2.line(grid_img, pt1_transformed, pt2_transformed, pink, 1)

        grid_mask = cv2.cvtColor(grid_img, cv2.COLOR_BGR2GRAY) > 0
        
        grid = frame.copy()
        grid[obstacle_img > 0] = obstacle_img[obstacle_img > 0]
        grid[grid_mask] = cv2.addWeighted(grid[grid_mask], 0.3, grid_img[grid_mask], 0.7, 0)
        return grid

    def draw_selected_waypoints(self, frame):
        viz_frame = frame.copy()
        if self.homography is None:
            return viz_frame
        
        for i, task in self.ui.task_coords.items():
            if task['start']:
                start_pixel = cv2.perspectiveTransform(np.array([[[task['start'][1] * self.GRID_SIZE_CM, task['start'][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                cv2.circle(viz_frame, tuple(map(int, start_pixel)), 5, (0,255,0), -1)
                cv2.putText(viz_frame, f"T{i}S", (int(start_pixel[0])+10, int(start_pixel[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            if task['end']:
                end_pixel = cv2.perspectiveTransform(np.array([[[task['end'][1] * self.GRID_SIZE_CM, task['end'][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                cv2.circle(viz_frame, tuple(map(int, end_pixel)), 5, (0,0,255), -1)
                cv2.putText(viz_frame, f"T{i}E", (int(end_pixel[0])+10, int(end_pixel[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        return viz_frame

    def draw_assigned_routes(self, frame, schedules):
        if schedules is None or self.graph is None:
            return frame
        
        base_colors = [(0, 255, 0), (255, 165, 0)]  # Green and orange for each robot
        viz_frame = frame.copy()
        
        for robot_id, schedule in enumerate(schedules):
            base_color = base_colors[robot_id]

            steps = len(schedule) - 1
            shading_step = 0.7 / steps if steps > 0 else 0  
            
            for i in range(len(schedule) - 1):
                darkness = 1.0 - (i * shading_step)
                color = tuple(int(c * darkness) for c in base_color)
                try:
                    if schedule[i]['location'] == schedule[i + 1]['location']:
                        pt = cv2.perspectiveTransform(np.array([[[schedule[i]['location'][1] * self.GRID_SIZE_CM, schedule[i]['location'][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                        cv2.circle(viz_frame, tuple(map(int, pt)), 5, color, -1)
                        if schedule[i + 1]['task_id'] is not None:
                            label = f"{schedule[i + 1]['action']} T {schedule[i + 1]['task_id']}"
                        else:
                            label = schedule[i + 1]['action']
                        cv2.putText(viz_frame, label, (int(pt[0] + 10), int(pt[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        continue

                    path = self.shortest_paths.get((schedule[i]['location'], schedule[i + 1]['location']))
                    if not path:
                        path = nx.shortest_path(self.graph, schedule[i]['location'], schedule[i + 1]['location'], weight='weight')
                    for k in range(len(path) - 1):
                        pt1 = cv2.perspectiveTransform(np.array([[[path[k][1] * self.GRID_SIZE_CM, path[k][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                        pt2 = cv2.perspectiveTransform(np.array([[[path[k + 1][1] * self.GRID_SIZE_CM, path[k + 1][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                        cv2.line(viz_frame, tuple(map(int, pt1)), tuple(map(int, pt2)), color, 3)
                        
                    pt = cv2.perspectiveTransform(np.array([[[path[-1][1] * self.GRID_SIZE_CM, path[-1][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                    cv2.circle(viz_frame, tuple(map(int, pt)), 5, color, -1)

                    if schedule[i + 1]['task_id'] is not None:
                        label = f"{schedule[i + 1]['action']} T {schedule[i + 1]['task_id']}"
                    else:
                        label = schedule[i + 1]['action']
                    cv2.putText(viz_frame, label, (int(pt[0] + 10), int(pt[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                except nx.NetworkXNoPath:
                    print(f"No path found between {schedule[i]['location']} and {schedule[i + 1]['location']}")
                    continue
                except Exception as e:
                    print(f"Error drawing path: {e}")
                    continue
        return viz_frame
    
    def create_travel_time_matrix(self, locations):
        if self.graph is None or not locations:
            return None
        n = len(locations)
        matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                try:
                    path = nx.shortest_path(self.graph, locations[i], locations[j], weight='weight')
                    print(path)
                    self.shortest_paths[locations[i], locations[j]] = path
                    self.shortest_paths[locations[j], locations[i]] = list(reversed(path))
                    base_time = nx.shortest_path_length(self.graph, locations[i], locations[j], weight='weight') * self.MOVE_DURATION_MS_PER_CM
                    prev_dir = None
                    # Include worst case initial turn (180 degrees) in case robot needs to fully turn around to start path
                    total_turn_time = 180 * self.TURN_DURATION_MS_PER_DEG
                    for k in range(len(path)-1):
                        dx = path[k+1][1] - path[k][1]
                        dy = path[k+1][0] - path[k][0]
                        magnitude = max(abs(dx), abs(dy))
                        curr_dir = (dx/magnitude, dy/magnitude) if magnitude > 0 else (0, 0)
                        if prev_dir:
                            turn_angle = self.calculate_turn_angle(prev_dir, curr_dir)
                            total_turn_time += abs(turn_angle) * self.TURN_DURATION_MS_PER_DEG
                        prev_dir = curr_dir
                    print(base_time, total_turn_time)
                    matrix[i][j] = matrix[j][i] = int((base_time + total_turn_time) // 100)
                    print(f"Travel time between {locations[i]} and {locations[j]}: {matrix[i][j]} ms")
                except nx.NetworkXNoPath:
                    matrix[i][j] = matrix[j][i] = 10000
        return matrix

    def calculate_turn_angle(self, prev_dir, curr_dir):
        def dir_to_angle(direction):
            if direction == (0, 1): return 0
            if direction == (1, 1): return 45
            if direction == (1, 0): return 90
            if direction == (1, -1): return 135
            if direction == (0, -1): return 180
            if direction == (-1, -1): return 225
            if direction == (-1, 0): return 270
            if direction == (-1, 1): return 315
            return 0
        if prev_dir is None or curr_dir is None:
            return 0
        angle_diff = ((dir_to_angle(curr_dir) - dir_to_angle(prev_dir) + 180) % 360) - 180
        return angle_diff