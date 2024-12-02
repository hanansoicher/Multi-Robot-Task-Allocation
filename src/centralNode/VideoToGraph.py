import numpy as np
import cv2 as cv
import threading as th 
import networkx as nx
from networkx.algorithms.shortest_paths.astar import astar_path

import matplotlib.pyplot as plt

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
        self.pixel_block_height_px = 0
        self.pixel_block_width_px = 0

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
        corners = self.create_bounding_box(image)
        grid, grid_width, grid_height = self.create_grid(corners)

        
        # Compute the affine transformation
        matrix = self.compute_affine_transformation(corners, grid_width, grid_height)
        overlay_image = image.copy()

        print(f"pixel block size: {self.pixel_block_width_px}x{self.pixel_block_height_px}")
        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                transformed_block = self.apply_affine_transform((col, row), matrix)
        
                (x, y,_) = transformed_block
                # Create a centered bounding box for the grid cell
                top_left_x = int(x - self.pixel_block_width_px / 2)
                top_left_y = int(y - self.pixel_block_height_px / 2)

                bottom_right_x = int(x + self.pixel_block_width_px / 2)
                bottom_right_y = int(y + self.pixel_block_height_px / 2)

                # Draw the rectangle
                cv.rectangle(
                    overlay_image,
                    (top_left_x, top_left_y),
                    (bottom_right_x, bottom_right_y),
                    (0, 255, 0),  # Green color
                    1  # Line thickness
                )

        # Visualize the graph
        plt.figure(figsize=(8, 8))
        plt.imshow(cv.cvtColor(overlay_image, cv.COLOR_BGR2RGB))  # Convert from BGR to RGB
        plt.title("Grid Overlay on Image")
        plt.axis('off')  # Hide axes
        plt.show()
# Detecting the objects

