import numpy as np
import cv2 as cv
import threading as th 
import networkx as nx
from util import UtilityFunctions as uf


def main():
    vg = VideoToGraph(75, 150)
    image = cv.imread("img/maze_crop_modified.png")
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
        overlay_image = image.copy()

        # Apply the affine transformation to each node in the graph
        for node in graph.nodes():
            transformed_node = uf.apply_affine_transform(node, matrix)
            cv.circle(overlay_image, (int(transformed_node[0]), int(transformed_node[1])), radius=5, color=(0, 255, 0), thickness=-1)

        # Optionally, draw the edges between connected nodes
        for edge in graph.edges():
            node1_transformed = uf.apply_affine_transform(edge[0], matrix)
            node2_transformed = uf.apply_affine_transform(edge[1], matrix)
            cv.line(overlay_image, 
                    (int(node1_transformed[0]), int(node1_transformed[1])), 
                    (int(node2_transformed[0]), int(node2_transformed[1])), 
                    color=(255, 0, 0), thickness=1)
        uf.show_image(overlay_image)
    
    def set_dimensions(self, corners, block_size_cm=4):
        # Compute grid dimensions based on the block size and image size
        image_width_px = corners['top_right'][0] - corners['top_left'][0]
        image_height_px = corners['bottom_left'][1] - corners['top_left'][1]

        pixel_block_height_px  = (block_size_cm / self.maze_height) * image_height_px
        pixel_block_width_px = (block_size_cm / self.maze_length) * image_width_px

        self.grid_width = int(image_width_px / pixel_block_width_px)
        self.grid_height = int(image_height_px / pixel_block_height_px)         

# Detecting the objects
def detect_objects():
    pass
    # qcd = cv.QRCodeDetector()
    # outputs multiple QR codes

if __name__ == "__main__":
    main()