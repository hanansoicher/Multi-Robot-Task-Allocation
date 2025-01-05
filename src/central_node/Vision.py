import cv2
import numpy as np
import networkx as nx

class Vision:
    def __init__(self, camera_input=0):
        self.cap = cv2.VideoCapture(camera_input)
        # self.cap = cv2.VideoCapture(camera_input, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera/video")
        self.frame = None
        self.corners = {}
        self.grid = None
        self.graph = None
        self.homography = None
        self.running = True
        self.solver_ran = False
        self.solver_callback = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.MAZE_WIDTH_CM = 72
        self.MAZE_HEIGHT_CM = 45
        self.GRID_SIZE_CM = 1

    def set_solver_callback(self, callback):
        self.solver_callback = callback

    def run(self):
        frame_num = 0
        corners_found = False
        while self.running:
            if not self.solver_ran:
                print(f"Frame {frame_num} processing...")
            frame_num += 1
            ret, frame = self.cap.read()
            self.frame = frame
            if not ret or frame is None:
                self.running = False
                print("Cannot read frame, aborting...")
            if not corners_found:
                self.corners = self.find_corners(frame)
                corners_found = True
            self.grid = self.create_grid(self.detect_unreachable_areas(frame))
            self.graph = self.create_graph()

            viz_frame = frame.copy()

            if self.corners:
                for corner in self.corners.values():
                    cv2.circle(viz_frame, corner, 5, (0,255,0), -1)

            if self.grid is not None:
                for i in range(self.grid.shape[0]):
                    for j in range(self.grid.shape[1]):
                        if self.grid[i,j] == 1:
                            pt = cv2.perspectiveTransform(
                                np.array([[[j*self.GRID_SIZE_CM, i*self.GRID_SIZE_CM]]], dtype='float32'), 
                                cv2.invert(self.homography)[1]
                            )[0][0]
                            cv2.circle(viz_frame, tuple(map(int, pt)), 2, (0,0,255), -1)
            
            robots, waypoints = self.detect_markers(frame)
            if robots and waypoints:
                for robot_id, robot_pos in robots.items():
                    for waypoint_id, waypoint_pos in waypoints.items():
                        try:
                            path = nx.shortest_path(self.graph, robot_pos, waypoint_pos, weight='weight')
                            for k in range(len(path) - 1):
                                pt1 = cv2.perspectiveTransform(np.array([[[path[k][1] * self.GRID_SIZE_CM, path[k][0] * self.GRID_SIZE_CM]]], dtype='float32'), cv2.invert(self.homography)[1])[0][0]
                                pt2 = cv2.perspectiveTransform(np.array([[[path[k + 1][1] * self.GRID_SIZE_CM, path[k + 1][0] * self.GRID_SIZE_CM]]], dtype='float32'), cv2.invert(self.homography)[1])[0][0]
                                cv2.line(viz_frame, tuple(map(int, pt1)), tuple(map(int, pt2)), (255, 0, 0), 2)
                        except nx.NetworkXNoPath:
                            continue
            viz_frame = self.build_grid_overlay(viz_frame)

            cv2.imshow('Maze View', viz_frame)
            if not self.solver_ran:
                print(f"Frame {frame_num} displayed")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("q pressed, exiting...")
                self.running = False
            elif cv2.waitKey(1) & 0xFF == ord('b'):
                print("b pressed, running solver...")
                if self.solver_callback:
                    self.solver_ran = True
                    self.solver_callback()

        print("Cleaning up...")
        self.cap.release()
        cv2.destroyAllWindows()

    def find_corners(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        corner_positions = {}
        corner_ids = {96: 'top_left', 97: 'top_right', 98: 'bottom_left', 99: 'bottom_right'}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in corner_ids:
                marker_corners = corners[i][0]
                center = tuple(map(int, np.mean(marker_corners, axis=0)))
                corner_positions[corner_ids[marker_id]] = center

        if len(corner_positions) != 4:
            print("Couldn't find corners")

        # # Ensure corners are on the same vertical/horizontal axes
        # top_left = corner_positions['top_left']
        # top_right = corner_positions['top_right']
        # bottom_left = corner_positions['bottom_left']
        # bottom_right = corner_positions['bottom_right']

        # top_left = (bottom_left[0], top_right[1])
        # bottom_right = (top_right[0], bottom_left[1])

        src_pts = np.float32([corner_positions['top_left'], corner_positions['top_right'], 
                              corner_positions['bottom_left'], corner_positions['bottom_right']])
        dest_pts = np.float32([[0, 0], [self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, 0],
                               [0, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM],
                               [self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM]])
        self.homography = cv2.getPerspectiveTransform(src_pts, dest_pts)
        return corner_positions

    def detect_unreachable_areas(self, frame):
        if self.homography is None:
            print("Homography not found")

        warped = cv2.warpPerspective(frame, self.homography, 
            (int(self.MAZE_WIDTH_CM * self.GRID_SIZE_CM), 
            int(self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM)))
        
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel_close = np.ones((5,5), np.uint8)
        kernel_dilate = np.ones((3,3), np.uint8)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.dilate(mask, kernel_dilate, iterations=1)
        
        return mask

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        robots = {}
        waypoints = {}

        if ids is not None:
            rows, cols = int(self.MAZE_HEIGHT_CM), int(self.MAZE_WIDTH_CM)
            waypoint_i = 0
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)

                if self.homography is not None:
                    grid_pos = cv2.perspectiveTransform(np.array([[center]]), self.homography)[0][0]
                    grid_x = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), cols - 1))
                    grid_y = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), rows - 1))
                    grid_pos = (grid_y, grid_x)  # (row, col) for grid indexing

                    if 0 <= marker_id <= 1:
                        robots[marker_id] = grid_pos
                    elif marker_id not in [0, 1, 96, 97, 98, 99]:
                        if marker_id % 2 == 0:
                            waypoints[2*waypoint_i] = grid_pos
                        else:
                            waypoints[2*waypoint_i+1] = grid_pos
                            waypoint_i += 1
        if not self.solver_ran:
            print(f"Detected {len(robots)} robots and {len(waypoints)} waypoints")
        return robots, waypoints

    def build_grid_overlay(self, frame):
        if self.homography is None:
            return frame
                    
        grid_img = np.zeros_like(frame)
        obstacle_img = np.zeros_like(frame)
        
        # Pink color in BGR for grid, lime for unreachable
        pink = (200, 20, 255)
        lime = (33, 190, 120)
        
        inv_homography = cv2.invert(self.homography)[1]
        
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
                        
                        transformed_corners = cv2.perspectiveTransform(rect_corners, inv_homography)
                        points = transformed_corners.reshape((-1,1,2)).astype(np.int32)
                        cv2.fillPoly(obstacle_img, [points], lime)
        
        spacing = self.GRID_SIZE_CM
        for y in range(0, int(self.MAZE_HEIGHT_CM) + 1, spacing):
            pt1 = np.array([[[0, y * self.GRID_SIZE_CM]]], dtype=np.float32)
            pt2 = np.array([[[self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, y * self.GRID_SIZE_CM]]], dtype=np.float32)
            
            pt1_transformed = cv2.perspectiveTransform(pt1, inv_homography)[0][0]
            pt2_transformed = cv2.perspectiveTransform(pt2, inv_homography)[0][0]
            
            pt1_transformed = tuple(map(int, pt1_transformed))
            pt2_transformed = tuple(map(int, pt2_transformed))
            cv2.line(grid_img, pt1_transformed, pt2_transformed, pink, 1)
        
        for x in range(0, int(self.MAZE_WIDTH_CM) + 1, spacing):
            pt1 = np.array([[[x * self.GRID_SIZE_CM, 0]]], dtype=np.float32)
            pt2 = np.array([[[x * self.GRID_SIZE_CM, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM]]], dtype=np.float32)
            
            pt1_transformed = cv2.perspectiveTransform(pt1, inv_homography)[0][0]
            pt2_transformed = cv2.perspectiveTransform(pt2, inv_homography)[0][0]
            
            pt1_transformed = tuple(map(int, pt1_transformed))
            pt2_transformed = tuple(map(int, pt2_transformed))
            cv2.line(grid_img, pt1_transformed, pt2_transformed, pink, 1)
        
        grid_mask = cv2.cvtColor(grid_img, cv2.COLOR_BGR2GRAY) > 0
        
        result = frame.copy()
        result[obstacle_img > 0] = obstacle_img[obstacle_img > 0]
        result[grid_mask] = cv2.addWeighted(result[grid_mask], 0.3, grid_img[grid_mask], 0.7, 0)
        
        return result

    def create_grid(self, obstacle_mask):
        rows = int(self.MAZE_HEIGHT_CM)
        cols = int(self.MAZE_WIDTH_CM)
        grid = np.zeros((rows, cols), dtype=np.uint8)
        
        if obstacle_mask is None:
            return grid
            
        try:
            obstacle_mask = cv2.resize(obstacle_mask, (cols, rows))
            grid[obstacle_mask > 0] = 1
        except cv2.error as e:
            print(f"Error resizing obstacle mask: {e}")
            return grid

        return grid

    def create_graph(self):
        if self.grid is None:
            return None

        graph = nx.Graph()
        rows, cols = self.grid.shape
        
        for i in range(rows):
            for j in range(cols):
                graph.add_node((i,j))
                
                for di, dj in [(-1,0), (1,0), (0,-1), (0,1), 
                                (-1,-1), (-1,1), (1,-1), (1,1)]:
                    ni, nj = i + di, j + dj
                    if (0 <= ni < rows and 0 <= nj < cols and self.grid[ni,nj] == 0 and (ni,nj) in graph) and self.grid[i,j] == 0:
                        # Weight is sqrt(2) for diagonal, 1 for adjacent
                        weight = np.sqrt(2) if abs(di) + abs(dj) == 2 else 1
                        graph.add_edge((i,j), (ni,nj), weight=weight)
                    else:
                        graph.add_edge((i,j), (ni,nj), weight=float('inf'))
        return graph

    def create_travel_time_matrix(self, locations):
        if self.graph is None or not locations:
            return None

        n = len(locations)
        matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                try:
                    if not self.solver_ran:
                        print(locations[i], locations[j])
                    time = nx.shortest_path_length(self.graph, locations[i], locations[j], weight='weight')
                    matrix[i][j] = matrix[j][i] = time
                except nx.NetworkXNoPath:
                    matrix[i][j] = matrix[j][i] = float('inf')
        return matrix

def main():
    # video = "img/mazewmarkers.png"
    # vg = Vision(video)
    vg = Vision()
    vg.run()

if __name__ == '__main__':
    main()
