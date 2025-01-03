import cv2
import numpy as np
import networkx as nx

class Vision:
    def __init__(self, camera_input=0):
        self.cap = cv2.VideoCapture(camera_input)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera/video")
        
        self.corners = {}
        self.grid = None
        self.graph = None
        self.homography = None
        self.running = True

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Constants
        self.MAZE_WIDTH_CM = 200
        self.MAZE_HEIGHT_CM = 100
        self.GRID_SIZE_CM = 1

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.running = False
            return None, None, None, None

        corners = self.find_corners(frame)
        if corners is None:
            return frame, None, None, None

        robots, waypoints = self.detect_markers(frame)

        obstacle_mask = self.detect_unreachable_areas(frame)
        if obstacle_mask is None:
            return frame, None, None, None

        self.grid = self.create_grid(obstacle_mask)
        self.graph = self.create_graph()
        locations = list(robots.values()) + list(waypoints.values())
        travel_times = self.create_travel_time_matrix(locations)

        return frame, robots, waypoints, travel_times

    def run(self):
        while self.running:
            frame, _, _, _ = self.process_frame()
            if frame is None:
                break

            if self.corners:
                for corner in self.corners.values():
                    cv2.circle(frame, corner, 5, (0,255,0), -1)

            if self.grid is not None:
                for i in range(self.grid.shape[0]):
                    for j in range(self.grid.shape[1]):
                        if self.grid[i,j] == 1:
                            pt = cv2.perspectiveTransform(np.array([[[j*self.GRID_SIZE_CM, i*self.GRID_SIZE_CM]]], dtype='float32'), cv2.invert(self.homography)[1])[0][0]
                            cv2.circle(frame, tuple(map(int, pt)), 2, (0,0,255), -1)

            self.overlay_grid(frame)
            cv2.imshow('Maze View', frame)

            if cv2.waitKey(1) == ord('q'):
                self.running = False

        self.cap.release()
        cv2.destroyAllWindows()

    def find_corners(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None:
            return None

        corner_positions = {}
        corner_ids = {96: 'top_left', 97: 'top_right', 98: 'bottom_left', 99: 'bottom_right'}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in corner_ids:
                marker_corners = corners[i][0]
                center = tuple(map(int, np.mean(marker_corners, axis=0)))
                corner_positions[corner_ids[marker_id]] = center

        if len(corner_positions) != 4:
            return None

        self.corners = corner_positions
        src_pts = np.float32([corner_positions['top_left'], corner_positions['top_right'], 
                              corner_positions['bottom_left'], corner_positions['bottom_right']])
        dest_pts = np.float32([[0, 0], [self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, 0],
                               [0, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM],
                               [self.MAZE_WIDTH_CM * self.GRID_SIZE_CM, self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM]])
        self.homography = cv2.getPerspectiveTransform(src_pts, dest_pts)
        return self.corners

    def detect_unreachable_areas(self, frame):
        if self.homography is None:
            return None

        warped = cv2.warpPerspective(frame, self.homography, 
            (int(self.MAZE_WIDTH_CM * self.GRID_SIZE_CM), 
            int(self.MAZE_HEIGHT_CM * self.GRID_SIZE_CM)))
        
        # Convert to HSV color space for better red detection
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        
        # Define range for red color
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red regions
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Clean up mask
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask


    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        robots = {}
        waypoints = {}

        if ids is not None:
            rows, cols = int(self.MAZE_HEIGHT_CM), int(self.MAZE_WIDTH_CM)
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)

                if self.homography is not None:
                    grid_pos = cv2.perspectiveTransform(np.array([[center]]), self.homography)[0][0]
                    # Convert to integer tuple and ensure within bounds
                    grid_x = int(min(max(grid_pos[0] / self.GRID_SIZE_CM, 0), cols - 1))
                    grid_y = int(min(max(grid_pos[1] / self.GRID_SIZE_CM, 0), rows - 1))
                    grid_pos = (grid_y, grid_x)  # Note: Using (row, col) format for grid indexing

                    if 0 <= marker_id <= 1:
                        robots[f"robot_{marker_id+1}"] = grid_pos
                    else:
                        waypoints[f"waypoint_{marker_id-1}"] = grid_pos
        print(robots, waypoints)
        return robots, waypoints

    def overlay_grid(self, frame):
        height, width, _ = frame.shape
        for i in range(0, int(self.MAZE_HEIGHT_CM) + 1):
            y = i * self.GRID_SIZE_CM
            cv2.line(frame, (0, y), (width, y), (0, 255, 0), 1)
        for j in range(0, int(self.MAZE_WIDTH_CM) + 1):
            x = j * self.GRID_SIZE_CM
            cv2.line(frame, (x, 0), (x, height), (0, 255, 0), 1)
        return frame

    def create_grid(self, obstacle_mask):
        rows = int(self.MAZE_HEIGHT_CM)
        cols = int(self.MAZE_WIDTH_CM)
        grid = np.zeros((rows, cols), dtype=np.uint8)
        obstacle_mask = cv2.resize(obstacle_mask, (cols, rows))
        grid[obstacle_mask > 0] = 1
        return grid

    def create_graph(self):
        """Create navigation graph from grid."""
        if self.grid is None:
            return None
            
        graph = nx.Graph()
        rows, cols = self.grid.shape
        
        for i in range(rows):
            for j in range(cols):
                if self.grid[i,j] == 0:  # Reachable cell
                    graph.add_node((i,j))
                    
                    # Add edges to neighbors (8-connected)
                    for di, dj in [(-1,0), (1,0), (0,-1), (0,1), 
                                 (-1,-1), (-1,1), (1,-1), (1,1)]:
                        ni, nj = i + di, j + dj
                        if (0 <= ni < rows and 0 <= nj < cols and 
                            self.grid[ni,nj] == 0 and 
                            (ni,nj) in graph):
                            # Weight is sqrt(2) for diagonal, 1 for adjacent
                            weight = np.sqrt(2) if abs(di) + abs(dj) == 2 else 1
                            graph.add_edge((i,j), (ni,nj), weight=weight)
        return graph


    def create_travel_time_matrix(self, locations):
        if self.graph is None or not locations:
            return None

        n = len(locations)
        matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i+1, n):
                try:
                    print(locations[i], locations[j])
                    time = nx.shortest_path_length(self.graph, locations[i], locations[j], weight='weight')
                    matrix[i][j] = matrix[j][i] = time
                except nx.NetworkXNoPath:
                    matrix[i][j] = matrix[j][i] = float('inf')
        return matrix

def main():
    video = "img/mazewmarkers.png"
    vg = Vision(video)
    vg.run()

if __name__ == '__main__':
    main()
