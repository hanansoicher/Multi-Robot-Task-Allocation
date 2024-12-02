import numpy as np
import cv2 as cv
import threading as th 
import networkx as nx
from typing import Optional
from util import UtilityFunctions as uf
import math


def main():
    vg = VideoToGraph(75, 150)
    image = cv.imread("img/maze_crop_modified_black.png")
    vg.convert_image_to_graph(image)

class VideoToGraph:

    #initilaize
    def __init__(self, width, length, metric = True):
        #self.cap = self.initialize_camera()
        self.maze_height = width if metric else width * 2.54
        self.maze_length = length if metric else length * 2.54
        self.grid_height = 0
        self.grid_width = 0

        if False:
            # Create a thread for updating the graph
            self.graph_thread = th.Thread(target=self.CreateStreamOfGraph, args=(self.cap,))
            self.graph_thread.daemon = True  # Set as daemon thread so it exits when the main program ends
            self.graph_thread.start()

    # Video input
    def initialize_camera(self, camera = int(0)):
        capture = cv.VideoCapture(camera) # 0 is the default camera, can also take a file

        if not capture.isOpened():
            print("Cannot open camera")
            input("Press Enter to exit... ")
            exit()

        return capture
    
    # Release the camera 
    def tear_down(self):
        self.cap.release()
        cv.destroyAllWindows()

    # Create and update graph from the video input
    def create_graph(self, cap):
        pass
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
        
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # Our operations on the frame come here
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            
            # Display the resulting frame
            cv.imshow('frame', gray)
            if cv.waitKey(1) == ord('q'):
                break

    def convert_image_to_graph(self, image):
        corners = uf.create_bounding_box(image)
        self.set_dimensions(corners)
        graph = nx.grid_2d_graph(self.grid_width, self.grid_height)
        
        matrix = uf.compute_affine_transformation(corners, self.grid_width, self.grid_height)

        pos = {node: node for node in graph.nodes()}
        real_pos = {node: uf.apply_affine_transform(node, matrix) for node in graph.nodes()}
        nx.set_node_attributes(graph, pos, "pos")
        nx.set_node_attributes(graph, real_pos, "real_pos")

        overlay_image = image.copy()
        self.detect_black_areas(image, graph)

        # Apply the affine transformation to each node in the graph
        for node in graph.nodes():
            transformed_node = graph.nodes[node]["real_pos"]
            near_black = self.near_black_area(graph, node)
            color = (0, 0, 255) if near_black else (0, 255, 0)
            cv.circle(overlay_image, (int(transformed_node[0]), int(transformed_node[1])), radius=5, color=color, thickness=-1)

        # Optionally, draw the edges between connected nodes
        for edge in graph.edges():
            node1_x, node1_y,_ = uf.apply_affine_transform(edge[0], matrix)
            node2_x, node2_y,_ = uf.apply_affine_transform(edge[1], matrix)
            graph[edge[0]][edge[1]]['weight'] = math.sqrt((node2_x - node1_x) ** 2 + (node2_y - node1_y) ** 2)
            color =  (0, 0, 255) if self.near_black_area(graph, edge[0], edge[1]) else (255, 0, 0)
            cv.line(overlay_image, 
                    (int(node1_x), int(node1_y)), 
                    (int(node2_x), int(node2_y)), 
                    color=color, thickness=1)
        uf.show_image(overlay_image)
        self.penalize_near_black_areas(graph)
        path = self.safe_astar_path(graph, (0, 0), (self.grid_width - 1, self.grid_height - 4), heuristic=self.heuristic)
        if path:
            overlay_image = self.draw_transformed_path(overlay_image, graph, path)

        uf.show_image(overlay_image)

    def near_black_area(self, graph, node_a, node_b: Optional[any] = None):
        if node_b is None:
            return graph.nodes[node_a].get("is_near_black")
        return graph.nodes[node_a].get("is_near_black") or graph.nodes[node_b].get("is_near_black")

    def penalize_near_black_areas(self, graph):
        for node in graph.nodes:
            # Check if the current node is near a black area
            if graph.nodes[node].get("is_near_black", False):
                for neighbor in graph.neighbors(node):
                    # Assign a very high weight to edges leading to this node
                    graph[node][neighbor]['weight'] = float('inf')
                    graph[neighbor][node]['weight'] = float('inf')  # For undirected graphs

    def set_dimensions(self, corners, block_size_cm=4):
        # Compute grid dimensions based on the block size and image size
        image_width_px = corners['top_right'][0] - corners['top_left'][0]
        image_height_px = corners['bottom_left'][1] - corners['top_left'][1]

        pixel_block_height_px  = (block_size_cm / self.maze_height) * image_height_px
        pixel_block_width_px = (block_size_cm / self.maze_length) * image_width_px

        self.grid_width = int(image_width_px / pixel_block_width_px)
        self.grid_height = int(image_height_px / pixel_block_height_px)         
 
    def detect_black_areas(self, image, graph, proximity_threshold=65):

        image_copy = image.copy()
        gray_scale =  cv.cvtColor(image_copy, cv.COLOR_BGRA2BGR) if image_copy.shape[-1] == 4 else cv.cvtColor(image_copy, cv.COLOR_BGR2GRAY)

        # Threshold the image to isolate black areas
        _, thresholded = cv.threshold(gray_scale, 5, 255, cv.THRESH_BINARY_INV)

        # Find contours of the black areas
        contours, _ = cv.findContours(thresholded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        overlay_image = cv.cvtColor(gray_scale, cv.COLOR_GRAY2BGR)

        # Filter contours based on area
        min_contour_area = 1000 
        filtered_contours = [cnt for cnt in contours if cv.contourArea(cnt) > min_contour_area]

        for contour in filtered_contours:
            cv.drawContours(overlay_image, [contour], -1, (0, 0, 255), 2)
    
            for node in graph.nodes:
                node_x, node_y,_ = graph.nodes[node]["real_pos"]
                distance = abs(cv.pointPolygonTest(contour, (node_x,node_y), True)) 
                if distance <= proximity_threshold:
                    graph.nodes[node]["is_near_black"] = True
                else:
                    graph.nodes[node].setdefault("is_near_black", False)
                
        uf.show_image(overlay_image)

    def draw_transformed_path(self, image, graph, path):
        overlay_image = image.copy()
        for i in range(len(path) - 1):
            node_a = path[i]
            node_b = path[i + 1]
            
            pos_a = graph.nodes[node_a]['real_pos']
            pos_b = graph.nodes[node_b]['real_pos']
 
            cv.line(overlay_image, (int(pos_a[0]), int(pos_a[1])), 
                    (int(pos_b[0]), int(pos_b[1])), (0, 255, 0), 2)  # Blue line
            
            cv.circle(overlay_image, (int(pos_a[0]), int(pos_a[1])), 5, (255,255,153), -1)  # Yellow circle
            cv.circle(overlay_image, (int(pos_b[0]), int(pos_b[1])), 5, (255,255,153), -1)  

        return overlay_image

    def safe_astar_path(self,graph, start_node, goal_node, heuristic):
        
        # Ensure the nodes are reachable
        if not nx.has_path(graph, start_node, goal_node):
            return None

        path = nx.astar_path(graph, source=start_node, target=goal_node, weight="weight", heuristic=heuristic)
        # Check if any edge in the path has infinite weight
        for u, v in zip(path[:-1], path[1:]):
            if graph[u][v]['weight'] == float('inf'):
                print(f"No path exists: Infinite weight edge encountered between {u} and {v}.")
                return None
        self.print_path_weights(graph, path)

        return path

    def print_path_weights(self,graph, path):
        print("Edge weights along the A* path:")
        total_weight = 0
        for i in range(len(path) - 1):
            node1 = path[i]
            node2 = path[i + 1]
            weight = graph[node1][node2].get('weight', None)  # Access the weight of the edge
            total_weight += weight if weight is not None else 0
            print(f"Edge {node1} -> {node2}: weight = {weight}")
        print(f"Total path weight: {total_weight}")

    def heuristic(self, node, goal):
        (x1, y1) = node
        (x2, y2) = goal
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Detecting the objects
    def detect_objects():
        pass
        # qcd = cv.QRCodeDetector()
        # outputs multiple QR codes

if __name__ == "__main__":
    main()