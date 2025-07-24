import cv2
import numpy as np
import networkx as nx


class Vision:
    """Handles computer vision tasks: ArUco detection, homography computation, coordinate transformations, and obstacle detection."""

    GRID_SIZE_CM = 15
    ENV_WIDTH_CM = 60
    ENV_HEIGHT_CM = 45
    # ENV_WIDTH_CM = 90
    # ENV_HEIGHT_CM = 90
    ROWS, COLS = ENV_HEIGHT_CM // GRID_SIZE_CM, ENV_WIDTH_CM // GRID_SIZE_CM

    MOVE_DURATION_MS_PER_CM = 126.67 # 1900ms / 15cm 
    TURN_DURATION_MS_PER_DEG = 5.28 # 1900ms / 360deg


    def __init__(self, camera_input=0):
        self.cap = cv2.VideoCapture(camera_input)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {camera_input}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.homography = None
        self.inverse_homography = None
        self.robot_coords = {} # {id: (row, col)}

        self.solver_matrix_paths = {} # {(start coord, end coord): path}
        self.obstacle_grid = None # ROWS x COLS binary grid, 1 = square contains >=1 tape pixel
        self.graph = None # nodes are (row, col) grid coordinates, edges to adjacent cells with weight=1
        

    def get_frame(self):
        """Capture and return a frame from the camera."""
        return self.cap.read()


    def process_frame(self, frame):
        """Process a frame: compute homography if needed, update obstacle grid and graph if needed."""
        if self.homography is None:
            self.compute_homography(frame.copy())
        elif self.obstacle_grid is None:
            # TODO: Update grid and graph periodically for dynamic obstacles
            self.update_obstacle_grid(frame.copy())
            self.update_graph()


    def compute_homography(self, frame):
        corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

        if ids is None:
            print("[Vision] No markers detected.")
            return

        # Map marker ID to its corners
        marker_corners = {mid: c[0] for c, mid in zip(corners, ids.flatten())}
        required = {96, 97, 98, 99}
        if not required.issubset(marker_corners):
            detected = set(marker_corners.keys())
            missing = required - detected
            print(f"[Vision] Missing markers: {missing}. Detected: {sorted(detected)}")
            return

        # Uses the inner corner of each marker
        # OpenCV ArUco corner order: 0=top-left, 1=top-right, 2=bottom-right, 3=bottom-left

        src = np.float32([
            marker_corners[96][2],  # Top Left marker, bottom-right corner 
            marker_corners[97][3],  # Top Right marker, bottom-left corner
            marker_corners[98][1],  # Bottom Left marker, top-right corner
            marker_corners[99][0],  # Bottom Right marker, top-left corner
        ])
        dst = np.float32([
            [0, 0],
            [self.ENV_WIDTH_CM, 0],
            [0, self.ENV_HEIGHT_CM],
            [self.ENV_WIDTH_CM, self.ENV_HEIGHT_CM],
        ])

        # print(f"Homography src points: {src}")
        # print(f"Homography dst points: {dst}")
        self.homography = cv2.getPerspectiveTransform(src, dst)
        self.inverse_homography = np.linalg.inv(self.homography)
        print("Corners detected, homography established.")
        

    def find_robots(self, frame):
        marker_corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is not None:
            rows, cols = int(self.ENV_HEIGHT_CM / self.GRID_SIZE_CM), int(self.ENV_WIDTH_CM / self.GRID_SIZE_CM)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                center = np.mean(marker_corners[i][0], axis=0)
                if marker_id < 96:
                    # Store as center of the cell
                    grid_pos = cv2.perspectiveTransform(np.array([[center]]), self.homography)[0][0]
                    col = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), cols - 1))
                    row = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), rows - 1))
                    
                    if marker_id == 3: # Temporary, accidentally skipped marker 2
                        self.robot_coords[marker_id-1] = (row, col)
                    else:
                        self.robot_coords[marker_id] = (row, col)  # (row, col)
        return self.robot_coords


    def update_obstacle_grid(self, frame):
        """ROWS x COLS binary grid, 1 if square contains >=1 tape pixel."""

        lower_obstacle_range = np.array([72, 30, 110], np.uint8)   # teal tape, OpenCV HSV
        upper_obstacle_range = np.array([92, 255, 255], np.uint8)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_obstacle_range, upper_obstacle_range)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        warped = cv2.warpPerspective(mask, self.homography, (self.ENV_WIDTH_CM, self.ENV_HEIGHT_CM), flags=cv2.INTER_NEAREST)
        grid = warped.reshape(self.ROWS, self.GRID_SIZE_CM, self.COLS, self.GRID_SIZE_CM).max(1).max(2)

        self.obstacle_grid = (grid > 0).astype(np.uint8)
        print("number of unreachable cells:", np.sum(self.obstacle_grid))
        return self.obstacle_grid


    def update_graph(self):
        g = nx.Graph()
        for r in range(self.ROWS):
            for c in range(self.COLS):
                if self.obstacle_grid[r, c] > 0:
                    continue
                g.add_node((r, c))
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    neighbor_r, neighbor_c = r+dr, c+dc
                    if 0 <= neighbor_r < self.ROWS and 0 <= neighbor_c < self.COLS and self.obstacle_grid[neighbor_r, neighbor_c] == 0:
                        g.add_edge((r,c), (neighbor_r,neighbor_c), weight=1)
        self.graph = g


    def create_travel_time_matrix(self, locations): # locations is a list of (row, col) tuples representing task waypoints and robot start positions
        n = len(locations)
        matrix = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                try:
                    path = nx.shortest_path(self.graph, (int(locations[i][0]), int(locations[i][1])), (int(locations[j][0]), int(locations[j][1])), weight='weight')
                    self.solver_matrix_paths[locations[i], locations[j]] = path
                    self.solver_matrix_paths[locations[j], locations[i]] = list(reversed(path))
                    num_moves = len(path) - 1
                    prev_orientation = None
                    num_turns = 2 # 2 for potential 180 degree turn to begin the path
                    for k in range(len(path)-1):
                        dx = path[k+1][1] - path[k][1]
                        dy = path[k+1][0] - path[k][0]
                        curr_orientation = (dy, dx)
                        if prev_orientation is not None:
                            dir_to_angle = {
                                (0, 1): 0,    # Right (increasing col)
                                (1, 0): 90,   # Down (increasing row)
                                (0, -1): 180, # Left (decreasing col)
                                (-1, 0): 270  # Up (decreasing row)
                            }
                            angle = dir_to_angle.get(curr_orientation, 0) - dir_to_angle.get(prev_orientation, 0)
                            if angle > 180:
                                angle -= 360
                            elif angle < -180:
                                angle += 360
                            num_turns += int(abs(angle) // 90)
                        prev_orientation = curr_orientation
                    travel_time = num_moves + num_turns
                    matrix[i][j] = matrix[j][i] = travel_time
                except nx.NetworkXNoPath:
                    matrix[i][j] = matrix[j][i] = 999999
        return matrix
    
    def get_obstacle_coordinates(self):
        obstacles = []
        if self.obstacle_grid is not None:
            for row in range(self.obstacle_grid.shape[0]):
                for col in range(self.obstacle_grid.shape[1]):
                    if self.obstacle_grid[row, col]:
                        obstacles.append((row, col))
        return obstacles

    def close(self):
        if self.cap:
            self.cap.release()