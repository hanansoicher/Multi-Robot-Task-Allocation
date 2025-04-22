import sys
from time import sleep
import time
import cv2
import numpy as np
import networkx as nx
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication
from UI import UI

class Vision:
    def __init__(self, coordinator, camera_input=0):
        self.ENV_WIDTH_CM = 116
        self.ENV_HEIGHT_CM = 52
        self.GRID_SIZE_CM = 1
        self.MOVE_DURATION_MS_PER_CM = 41
        self.FAST_TURN_DURATION_MS_PER_DEG = 3.6
        self.SLOW_TURN_DURATION_MS_PER_DEG = 10
        self.coordinator = coordinator
        self.corners = {}
        self.homography = None
        self.grid = None
        self._pending_grid = None
        self._pending_since = None
        self.graph = None
        self.robot_coords = {}
        self.shortest_paths = {}
        
        self.solver_ran = False

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.cap = cv2.VideoCapture(camera_input)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera/video")

        self.app = QApplication(sys.argv)
        self.ui = UI(self, coordinator)
        self.ui.show()

        self.timer = QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(50)

    def run(self):                    
        ret, frame = self.cap.read()
        if not ret or frame is None:
            # print("Cannot read frame, shutting down...")
            # self.cap.release()
            # self.app.quit()
            return
        if not self.corners:
            self.corners = self.find_corners(frame)
            if not self.corners or self.homography is None:
                sleep(1)
                return
        
        new_grid = self.get_obstacle_grid(frame)
        # If first assignment
        if self.grid is None:
            self.grid = new_grid
            self.graph = self.update_graph()
        else:
            if not np.array_equal(self.grid, new_grid):
                if self._pending_grid is None or not np.array_equal(self._pending_grid, new_grid):
                    self._pending_grid = new_grid.copy()
                    self._pending_since = time.time()
                else:
                    if time.time() - self._pending_since >= 1.0:
                        self.grid = self._pending_grid
                        self.graph = self.update_graph()
                        self._pending_grid = None
                        self._pending_since = None
            else:
                self._pending_grid = None
                self._pending_since = None
            
        viz_frame = self.update_overlay(frame.copy())

        if viz_frame is not None:
            height, width, _ = viz_frame.shape
            bytes_per_line = 3 * width
            qt_image = QImage(viz_frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.ui.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def find_corners(self, frame):
        corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is None:
            print("No ArUco markers found")
            return None

        corner_ids = {97: 'top_left', 99: 'top_right', 96: 'bottom_left', 98: 'bottom_right'} # real environment
        # corner_ids = {96: 'top_left', 97: 'top_right', 98: 'bottom_left', 99: 'bottom_right'} # sim
        corner_pixels = {}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in corner_ids:
                corner_pixels[corner_ids[marker_id]] = tuple(map(int, np.mean(corners[i][0], axis=0))) # center of marker

        if len(corner_pixels) < 4:
            print(f"Only found {len(corner_pixels)} corners")
            return None
        
        src_pts = np.float32([
            corner_pixels['top_left'], 
            corner_pixels['top_right'], 
            corner_pixels['bottom_left'], 
            corner_pixels['bottom_right']
        ])
        dest_pts = np.float32([
            [0, 0], 
            [self.ENV_WIDTH_CM / self.GRID_SIZE_CM, 0],
            [0, self.ENV_HEIGHT_CM / self.GRID_SIZE_CM], 
            [self.ENV_WIDTH_CM / self.GRID_SIZE_CM, self.ENV_HEIGHT_CM / self.GRID_SIZE_CM]
        ])
        try:
            self.homography = cv2.getPerspectiveTransform(src_pts, dest_pts)
            self.inv_homography = cv2.invert(self.homography)[1]
        except cv2.error as e:
            print(f"Error calculating homography: {e}")
            return None
        print("Found all 4 corners and transformed perspective")
        return corner_pixels

    def pixel_to_grid(self, x, y):
        grid_pos = cv2.perspectiveTransform(np.array([[[x, y]]], dtype='float32'), self.homography)[0][0]
        grid_x = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), self.ENV_WIDTH_CM - 1))
        grid_y = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), self.ENV_HEIGHT_CM - 1))
        return (grid_y, grid_x)  # (row, col) for grid indexing

    def find_robots(self, frame):
        marker_corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is not None:
            rows, cols = int(self.ENV_HEIGHT_CM / self.GRID_SIZE_CM), int(self.ENV_WIDTH_CM / self.GRID_SIZE_CM)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                center = np.mean(marker_corners[i][0], axis=0)
                if marker_id in [0, 1, 2, 3]:
                        grid_pos = cv2.perspectiveTransform(np.array([[center]]), self.homography)[0][0]
                        grid_x = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), cols - 1))
                        grid_y = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), rows - 1))
                        self.robot_coords[marker_id] = (grid_y, grid_x) # (row, col) for grid indexing
        return self.robot_coords

    def get_obstacle_grid(self, frame, with_buffer=True):
        # Obstacle color ranges
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        warped = cv2.warpPerspective(frame, self.homography, (int(self.ENV_WIDTH_CM * self.GRID_SIZE_CM), int(self.ENV_HEIGHT_CM / self.GRID_SIZE_CM)))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        obstacle_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        obstacle_mask = cv2.dilate(obstacle_mask, np.ones((3,3), np.uint8), iterations=1)
        rows = int(self.ENV_HEIGHT_CM/self.GRID_SIZE_CM)
        cols = int(self.ENV_WIDTH_CM/self.GRID_SIZE_CM)
        grid = np.zeros((rows, cols), dtype=np.uint8)
        if obstacle_mask is None:
            return grid
        try:
            obstacle_mask = cv2.resize(obstacle_mask, (cols, rows))
            grid[obstacle_mask > 0] = 1
            if not with_buffer:
                return grid
            grid[cv2.dilate(obstacle_mask, np.ones((int(5/self.GRID_SIZE_CM), int(5/self.GRID_SIZE_CM)), np.uint8), iterations=1) > 0] = 1  # 5 cm buffer on each side
        except cv2.error as e:
            print(f"Error resizing obstacle mask: {e}")
        return grid

    def update_graph(self):
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

    def update_graph_8d(self):
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

    def update_overlay(self, frame):
        pink = (200, 20, 255) # grid lines
        red = (0, 0, 255) # obstacles
        
        overlay = frame.copy()
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i, j] == 1:  # Unreachable grid square with buffer
                    rect_corners = np.array([
                        [[j * self.GRID_SIZE_CM, i * self.GRID_SIZE_CM]],
                        [[(j + 1) * self.GRID_SIZE_CM, i * self.GRID_SIZE_CM]],
                        [[(j + 1) * self.GRID_SIZE_CM, (i + 1) * self.GRID_SIZE_CM]],
                        [[j * self.GRID_SIZE_CM, (i + 1) * self.GRID_SIZE_CM]]
                    ], dtype=np.float32)
                    
                    transformed_corners = cv2.perspectiveTransform(rect_corners, self.inv_homography)
                    points = transformed_corners.reshape((-1, 1, 2)).astype(np.int32)
                    obstacle_overlay = overlay.copy()
                    cv2.fillPoly(obstacle_overlay, [points], red)
                    alpha = 0.5  # Transparency
                    cv2.addWeighted(obstacle_overlay, alpha, overlay, 1 - alpha, 0, overlay)
        for y in range(0, int(self.ENV_HEIGHT_CM) + 1, self.GRID_SIZE_CM):
            pt1 = np.array([[[0, y * self.GRID_SIZE_CM]]], dtype=np.float32)
            pt2 = np.array([[[self.ENV_WIDTH_CM * self.GRID_SIZE_CM, y * self.GRID_SIZE_CM]]], dtype=np.float32)
            cv2.line(overlay, tuple(map(int, cv2.perspectiveTransform(pt1, self.inv_homography)[0][0])), tuple(map(int, cv2.perspectiveTransform(pt2, self.inv_homography)[0][0])), pink, 1)
        for x in range(0, int(self.ENV_WIDTH_CM) + 1, self.GRID_SIZE_CM):
            pt1 = np.array([[[x * self.GRID_SIZE_CM, 0]]], dtype=np.float32)
            pt2 = np.array([[[x * self.GRID_SIZE_CM, self.ENV_HEIGHT_CM * self.GRID_SIZE_CM]]], dtype=np.float32)
            cv2.line(overlay, tuple(map(int, cv2.perspectiveTransform(pt1, self.inv_homography)[0][0])), tuple(map(int, cv2.perspectiveTransform(pt2, self.inv_homography)[0][0])), pink, 1)
        self.find_robots(frame)
        for robot_id, (row, col) in self.robot_coords.items():
            pt = cv2.perspectiveTransform(np.array([[[col * self.GRID_SIZE_CM, row * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
            cv2.circle(overlay, tuple(map(int, pt)), 8, (255, 0, 0), -1)
            cv2.putText(overlay, f"R{robot_id}", (int(pt[0] + 10), int(pt[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        if self.solver_ran and self.coordinator.schedules is not None:
                overlay = self.draw_assigned_routes(overlay)
        else:
            overlay = self.draw_selected_waypoints(overlay)
        return overlay

    def draw_selected_waypoints(self, frame):
        viz_frame = frame.copy()        
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

    # def draw_assigned_routes(self, frame):
    #     base_colors = [(0, 255, 0), (255, 165, 0), (0, 0, 255), (255, 0, 255)]  # Green, Orange, Blue, Magenta
    #     viz_frame = frame.copy()
        
    #     for robot_id, schedule in enumerate(self.coordinator.schedules):
    #         waypoints = len(schedule) - 1
    #         if waypoints < 1: continue
    #         base_color = base_colors[robot_id % len(base_colors)]
    #         shading_step = 0.7 / waypoints
    #         alpha = 0.6
            
    #         for i in range(waypoints):
    #             start_loc = schedule[i]['location']
    #             end_loc = schedule[i + 1]['location']
    #             if start_loc == end_loc: continue
                
    #             path = self.shortest_paths.get((start_loc, end_loc))
    #             if not path:
    #                 try:
    #                     self.shortest_paths[(start_loc, end_loc)] = nx.shortest_path(self.graph, start_loc, end_loc, weight='weight')
    #                 except nx.NetworkXNoPath:
    #                     print(f"No path found between {start_loc} and {end_loc}")
    #                     continue
                        
    #             for step in range(len(path) - 1):
    #                 pt1 = cv2.perspectiveTransform(np.array([[[path[step][1] * self.GRID_SIZE_CM, path[step][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
    #                 pt2 = cv2.perspectiveTransform(np.array([[[path[step+1][1] * self.GRID_SIZE_CM, path[step+1][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
    #                 overlay = viz_frame.copy()
    #                 cv2.line(overlay, tuple(map(int, pt1)), tuple(map(int, pt2)), tuple(int(c * (1.0 - (i * shading_step))) for c in base_color), 5)
    #                 cv2.addWeighted(overlay, alpha, viz_frame, 1-alpha, 0, viz_frame)

    #     for robot_id, schedule in enumerate(self.schedules):
    #         color = base_colors[robot_id % len(base_colors)]            
    #         for i, waypoint in enumerate(schedule):
    #             center = cv2.perspectiveTransform(np.array([[[waypoint['location'][1] * self.GRID_SIZE_CM, waypoint['location'][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
    #             cv2.circle(viz_frame, tuple(map(int, center)), 5, color, -1)
    #             if waypoint['action'] in ['PICKUP', 'DROPOFF']:
    #                 label = f"R{robot_id} {waypoint['action']} T {waypoint['task_id']}" if waypoint['task_id'] is not None else f"R{robot_id} {waypoint['action']}"
    #                 cv2.putText(viz_frame, label, (int(center[0] + 10), int(center[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    #             elif waypoint['action'] == 'MOVE':
    #                 cv2.putText(viz_frame, f"R{robot_id}", (int(center[0] + 5), int(center[1] + 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    #     return viz_frame
    
    def draw_assigned_routes(self, frame):
        viz_frame = frame.copy()
        base_colors = [(0, 255, 0), (255, 165, 0), (0, 0, 255), (255, 0, 255)]
        thickness = 3
        # for each stage and each robot, draw the saved path
        for stage, paths_by_robot in self.coordinator.collision_free_paths.items():
            for robot_id, path in paths_by_robot.items():
                if not path or len(path) < 2:
                    continue
                color = base_colors[robot_id % len(base_colors)]
                pts = []
                for (r, c) in path:
                    px, py = cv2.perspectiveTransform(np.array([[[c * self.GRID_SIZE_CM, r * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                    pts.append((int(px), int(py)))
                for p0, p1 in zip(pts, pts[1:]):
                    cv2.line(viz_frame, p0, p1, color, thickness)
        for robot_id, schedule in self.coordinator.schedules.items():
            color = base_colors[robot_id % len(base_colors)]            
            for i, waypoint in enumerate(schedule):
                center = cv2.perspectiveTransform(np.array([[[waypoint['location'][1] * self.GRID_SIZE_CM, waypoint['location'][0] * self.GRID_SIZE_CM]]], dtype='float32'), self.inv_homography)[0][0]
                cv2.circle(viz_frame, tuple(map(int, center)), 5, color, -1)
                if waypoint['action'] in ['PICKUP', 'DROPOFF']:
                    label = f"R{robot_id} {waypoint['action']} T {waypoint['task_id']}" if waypoint['task_id'] is not None else f"R{robot_id} {waypoint['action']}"
                    cv2.putText(viz_frame, label, (int(center[0] + 10), int(center[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                elif waypoint['action'] == 'MOVE':
                    cv2.putText(viz_frame, f"R{robot_id}", (int(center[0] + 5), int(center[1] + 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
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
                    self.shortest_paths[locations[i], locations[j]] = path
                    self.shortest_paths[locations[j], locations[i]] = list(reversed(path))
                    base_time = nx.shortest_path_length(self.graph, locations[i], locations[j], weight='weight') * self.MOVE_DURATION_MS_PER_CM
                    prev_dir = None
                    # Include worst case initial turn (180 degrees) in case robot needs to fully turn around to start path
                    total_turn_time = 135 * self.FAST_TURN_DURATION_MS_PER_DEG + 45 * self.SLOW_TURN_DURATION_MS_PER_DEG
                    for k in range(len(path)-1):
                        dx = path[k+1][1] - path[k][1]
                        dy = path[k+1][0] - path[k][0]
                        magnitude = max(abs(dx), abs(dy))
                        curr_dir = (dx/magnitude, dy/magnitude) if magnitude > 0 else (0, 0)
                        if prev_dir:
                            turn_angle = abs(self.calculate_turn_angle(prev_dir, curr_dir))
                            if turn_angle > 45:
                                turn_time = (turn_angle - 45) * self.FAST_TURN_DURATION_MS_PER_DEG + 45 * self.SLOW_TURN_DURATION_MS_PER_DEG
                            else:
                                turn_time = turn_angle * self.SLOW_TURN_DURATION_MS_PER_DEG
                            total_turn_time += turn_time
                        prev_dir = curr_dir
                    matrix[i][j] = matrix[j][i] = int(base_time + total_turn_time)
                    print(f"Travel time between {locations[i]} and {locations[j]}: {matrix[i][j]} ms")
                except nx.NetworkXNoPath:
                    matrix[i][j] = matrix[j][i] = 99999
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