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

    # Function to apply affine transformation to a point
    def apply_affine_transform(self, point, matrix):
        point_homogeneous = np.array([point[0], point[1], 1], dtype=np.float32)
        transformed = np.dot(matrix, point_homogeneous)
        x, y, w = transformed[0], transformed[1], transformed[2]
        if w != 0:  # Normalize by w for perspective transformations
            x /= w
            y /= w
        return x, y, w
    
    def calculate_scale_factor(self,point1, point2, real_world_distance):
        # Calculate pixel distance between two points
        pixel_distance = np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        # Calculate the scale factor
        scale_factor = real_world_distance / pixel_distance
        return scale_factor

    def compute_affine_transformation(self, corners, grid_width, grid_height):
        # Define the source (grid coordinates) and destination (image coordinates) points for affine transformation
        source_points = np.float32([
            [0, 0],  # Top-left of grid
            [grid_width-1, 0],  # Top-right of grid
            [0, grid_height-1],  # Bottom-left of grid 
            [grid_width-1, grid_height-1],  # Bottom-right of grid 
        ])

        dest_points = np.float32([
            corners["top_left"], 
            corners["top_right"],
            corners["bottom_left"],
            corners["bottom_right"],
        ])
        matrix,_ = cv.findHomography(source_points, dest_points)
    
        return matrix
    
    def create_grid(self, corners, block_size_cm=4):
        # Define the corner points
        top_left = corners['top_left']
        top_right = corners['top_right']
        bottom_left = corners['bottom_left']

        image_width_px = top_right[0] - top_left[0]
        image_height_px = bottom_left[1] - top_left[1]

        # Compute grid dimensions based on the block size and image size
        self.pixel_block_height_px  = (block_size_cm / self.maze_height) * image_height_px
        self.pixel_block_width_px = (block_size_cm / self.maze_length) * image_width_px

        grid_width = int(image_width_px / self.pixel_block_width_px)
        grid_height = int(image_height_px / self.pixel_block_height_px)
        print(f"Grid Size: {grid_width}x{grid_height}")
        
        # Initialize grid values with IDs,can be used to store information about the grid cell
        index = 0
        grid = np.zeros((grid_height, grid_width), dtype=int) 
        for row in range(grid_height):
            for col in range(grid_width):
                grid[row, col] = index
                index += 1
        return grid, grid_width, grid_height
    
    def create_bounding_box(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # define range of color in HSV to detect the corners
        color_ranges = {
            "green": ((35, 50, 50), (85, 255, 255)),  # Green range
            "red": ((0, 200, 200), (10, 255, 255)),    # Red range (low range)
            "blue": ((100, 50, 50), (140, 255, 255)), # Blue range
            "orange": ((10, 100, 100), (25, 255, 255)), # Orange range
        }
        
        # Find the corners of the maze
        corners = list(self.find_points(image, color_ranges).values())
        top_left = min(corners, key=lambda p: (p[0], p[1]))  
        bottom_left = min([p for p in corners if p != top_left], key=lambda p: (p[0], -p[1]))  
        top_right = min([p for p in corners if p != bottom_left and p != top_left], key=lambda p: (p[0], -p[1]))  
        bottom_right = max([p for p in corners if p != top_left and p != top_right and p != bottom_left], key=lambda p: (p[0], p[1]))  

        corners = {
            "top_left": top_left,
            "top_right": top_right,
            "bottom_left": bottom_left,
            "bottom_right": bottom_right,
        }

        return corners

    def find_points(self, image, color_ranges):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        points = {}
        # Draw the corners to confirm
        for color_name, (lower, upper) in color_ranges.items():
            # Create mask for the color
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            mask = cv.inRange(hsv, lower, upper)
            
            # Find contours for the color
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            # Find the centroids of the contours
            centroids = []
            for contour in contours:
                M = cv.moments(contour)
                if M["m00"] != 0:  # Avoid division by zero
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
            
            # Store centroids in the dictionary
            points[color_name] = centroids[0] if len(centroids) == 1 else centroids 
        
        return points            

# Detecting the objects
def detect_objects():
    pass
    # qcd = cv.QRCodeDetector()
    # outputs multiple QR codes

# Display image close window when q is pressed
def show_image(opencv_image):
    while True:
        cv.imshow("Threshold", opencv_image)
        if cv.waitKey(1) == ord('q'):
            break
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()

   