import numpy as np
from math import sqrt
from typing import Dict, Set
import cv2
from dijkstar import Graph, find_path
from structs import *

class GraphConstructor:
    def __init__(self):
        self.intersections: Dict[int, QRLocation] = {} # Intersections may be + shaped, L shaped, or T shaped
        self.adjacencyMatrix = None # direct paths between neighboring intersections
        self.roomGraph = None # complete graph with shortest paths between all intersections for input to SMT solver
        self.camera = cv2.VideoCapture()
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def capture_camera_frame(self):
        _, frame = self.camera.read()
        return frame

    # QR Code format: "id:type"
    def identify_intersections(self, frame: np.ndarray):
        detections = cv2.QRCodeDetector().decode(frame)

        for detection in detections:
            data = detection.data.decode("utf-8").split(":")
            corners = np.array([[p.x, p.y] for p in detection.polygon], dtype=np.float32)
            center = corners.mean(axis=0)
            self.intersections[data[0]] = QRLocation(id=int(data[0]), x=center[0], y=center[1], type=data[1])
    
    def detect_direct_path(self, frame: np.ndarray, id1: QRLocation, id2: QRLocation) -> bool:
        """
        Detect if there is a direct black tape path between two QR code locations
        """

        start_point = np.array([id1.x, id1.y])
        end_point = np.array([id2.x, id2.y])
        
        # Determine path direction
        path_vector = end_point - start_point

        is_vertical = abs(path_vector[1]) < abs(path_vector[0]) * 0.1
        is_horizontal = abs(path_vector[0]) < abs(path_vector[1]) * 0.1

        for loc in self.intersections.values():
            if is_vertical:
                # Check for intermediate intersections at same x-level and between the y coordinates
                if (abs(loc.x - id1.x) == 0 and min(id1.y, id1.y) < loc.y < max(id1.y, id2.y)):
                    return False
            if is_horizontal:
                # Check for intermediate intersections at same y-level and between the x coordinates
                if ((abs(loc.y - id1.y) == 0) and (min(id1.x, id2.x) < loc.x < max(id1.x, id2.x))):
                    return False
        
        # Mask for the expected path region
        mask = np.zeros_like(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        TAPE_WIDTH = 30  # pixels
        if is_vertical: # Vertical path
            y1 = int(min(id1.y, id2.y) - (TAPE_WIDTH)//2)
            y2 = int(max(id1.y, id2.y) + (TAPE_WIDTH)//2)
            x1 = int(min(id1.x, id2.x))
            x2 = int(max(id1.x, id2.x))
        elif is_horizontal: # Horizontal path
            x1 = int(min(id1.x, id2.x) - (TAPE_WIDTH)//2)
            x2 = int(max(id1.x, id2.x) + (TAPE_WIDTH)//2)
            y1 = int(min(id1.y, id2.y))
            y2 = int(max(id1.y, id2.y))
        else: # Diagonal paths not supported
            return False
        mask[y1:y2, x1:x2] = 255

        # Apply adaptive threshold to account for lighting conditions
        binary = cv2.adaptiveThreshold(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Apply mask
        path_region = cv2.bitwise_and(binary, mask)
        black_pixels = np.count_nonzero(path_region)
        path_length = np.linalg.norm(path_vector)
        min_expected_pixels = (TAPE_WIDTH * path_length) * 0.7  # At least 70% of path region should be black
        
        # Check for path continuity
        if black_pixels >= min_expected_pixels:
            contours, _ = cv2.findContours(path_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 1:  # Should be one continuous path
                contour = contours[0]
                contour_points = contour[:, 0, :]
                min_x = np.min(contour_points[:, 0])
                max_x = np.max(contour_points[:, 0])
                min_y = np.min(contour_points[:, 1])
                max_y = np.max(contour_points[:, 1])
                if is_horizontal:
                    return (abs(min_x - x1) < TAPE_WIDTH and abs(max_x - x2) < TAPE_WIDTH)
                else:
                    return (abs(min_y - y1) < TAPE_WIDTH and abs(max_y - y2) < TAPE_WIDTH)    
        return False
    
    def calculate_travel_time(self, id1: int, id2: int, speed: float) -> float:
        """Calculate travel time between two intersections based on Euclidean distance"""
        i = self.intersections[id1]
        j = self.intersections[id2]
        distance = sqrt((i.x - j.x)**2 + (i.y - j.y)**2)
        return distance / speed

    def identify_obstacle(self, frame: np.ndarray):
        # Identify obstacle in the maze and update the adjacency matrix to remove intersecting paths
        pass

    def build_room_graph(self):
        frame = self.capture_camera_frame()
        self.identify_intersections(frame)
        num_intersections = len(self.intersections)

        self.adjacencyMatrix = np.full((num_intersections, num_intersections), 10000) # Max travel time by default
        np.fill_diagonal(self.adjacencyMatrix, 0)

        for i in self.intersections.values():
            for j in self.intersections.values():
                if i.id != j.id:
                    if self.detect_direct_path(frame, i, j):
                        time = self.calculate_travel_time(i.id, j.id, 1)
                        self.adjacencyMatrix[i.id][j.id] = time
                        self.adjacencyMatrix[j.id][i.id] = time

        graph = Graph(undirected=True)
        for i in range(num_intersections):
            for j in range(i + 1, num_intersections):
                # Add edge if there's a direct path
                if self.adjacencyMatrix[i][j] != 10000:
                    graph.add_edge(i, j, self.adjacencyMatrix[i][j])

        complete_matrix = np.full((num_intersections, num_intersections), 10000)
        np.fill_diagonal(complete_matrix, 0)
        
        for i in range(num_intersections):
            for j in range(i + 1, num_intersections):
                try:
                    # Find shortest path through intermediate intersections
                    path = find_path(graph, i, j)
                    path_cost = int(np.ceil(path.total_cost))
                    complete_matrix[i][j] = path_cost
                    complete_matrix[j][i] = path_cost
                except:
                    # If no path exists, keep default max value
                    pass

        self.roomGraph = complete_matrix.tolist()
        return complete_matrix.tolist()
        
