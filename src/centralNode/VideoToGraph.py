import numpy as np
import cv2 as cv
import threading as th 
import networkx as nx
from typing import Optional
from util import UtilityFunctions as uf


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
        nx.set_node_attributes(graph, pos, "pos")

        real_pos = {node: uf.apply_affine_transform(node, matrix) for node in graph.nodes()}
        nx.set_node_attributes(graph, real_pos, "real_pos")

        overlay_image = image.copy()
        self.detect_black_areas(image, graph)

        # Apply the affine transformation to each node in the graph
        for node in graph.nodes():
            transformed_node = uf.apply_affine_transform(node, matrix)
            near_black = self.near_black_area(graph, node)
            color = (0, 0, 255) if near_black else (0, 255, 0)
            cv.circle(overlay_image, (int(transformed_node[0]), int(transformed_node[1])), radius=5, color=color, thickness=-1)

        # Optionally, draw the edges between connected nodes
        for edge in graph.edges():
            node1_x, node1_y,_ = uf.apply_affine_transform(edge[0], matrix)
            node2_x, node2_y,_ = uf.apply_affine_transform(edge[1], matrix)
            color =  (0, 0, 255) if self.near_black_area(graph, edge[0], edge[1]) else (255, 0, 0)
            cv.line(overlay_image, 
                    (int(node1_x), int(node1_y)), 
                    (int(node2_x), int(node2_y)), 
                    color=color, thickness=1)
        uf.show_image(overlay_image)

    def near_black_area(self, graph, node_a, node_b: Optional[any] = None):
        if node_b is None:
            return graph.nodes[node_a].get("is_near_black")
        return graph.nodes[node_a].get("is_near_black") or graph.nodes[node_b].get("is_near_black")

    
    def set_dimensions(self, corners, block_size_cm=4):
        # Compute grid dimensions based on the block size and image size
        image_width_px = corners['top_right'][0] - corners['top_left'][0]
        image_height_px = corners['bottom_left'][1] - corners['top_left'][1]

        pixel_block_height_px  = (block_size_cm / self.maze_height) * image_height_px
        pixel_block_width_px = (block_size_cm / self.maze_length) * image_width_px

        self.grid_width = int(image_width_px / pixel_block_width_px)
        self.grid_height = int(image_height_px / pixel_block_height_px)         
 
    def detect_black_areas(self, image, graph, proximity_threshold=65):
        # Load the image
        image_copy = image.copy()

        # Convert the copied image to grayscale
        gray_scale =  cv.cvtColor(image_copy, cv.COLOR_BGRA2BGR)  if image_copy.shape[-1] == 4 else cv.cvtColor(image_copy, cv.COLOR_BGR2GRAY)

        # Threshold the image to isolate black areas
        _, thresholded = cv.threshold(gray_scale, 5, 255, cv.THRESH_BINARY_INV)

        # Find contours of the black areas
        contours, _ = cv.findContours(thresholded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        overlay_image = cv.cvtColor(gray_scale, cv.COLOR_GRAY2BGR)


        # Filter contours based on area
        min_contour_area = 1000 
        filtered_contours = [cnt for cnt in contours if cv.contourArea(cnt) > min_contour_area]

        # Process each contour
        for contour in filtered_contours:
            cv.drawContours(overlay_image, [contour], -1, (0, 0, 255), 2)
    
            for node in graph.nodes:
                node_x, node_y,_ = graph.nodes[node]["real_pos"]
                distance = abs(cv.pointPolygonTest(contour, (node_x,node_y), True)) 
                if distance <= proximity_threshold:
                    graph.nodes[node]["is_near_black"] = True
                    print(f"Distance between node {node} and black area: {distance}")
                else:
                    graph.nodes[node].setdefault("is_near_black", False)
                


        uf.show_image(overlay_image)

    # Define a pixel-to-node function
    def pixel_to_node_func(self, pixel_x, pixel_y, graph):
        for node, data in graph.nodes(data=True):
            if abs(data["pos"][0] - pixel_x) < 25 and abs(data["pos"][1] - pixel_y) < 25:
                return node
        return None
    
    # Detecting the objects
    def detect_objects():
        pass
        # qcd = cv.QRCodeDetector()
        # outputs multiple QR codes

if __name__ == "__main__":
    main()