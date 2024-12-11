import cv2 as cv
import numpy as np
import math


class UtilityFunctions:

    YELLOW=(255,255,153)
    RED=(0,0,255)
    GREEN=(0,255,0)
    BLUE=(255,0,0)

    @staticmethod
    def euclidean_distance(point1, point2):
        return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1]-point1[1]) ** 2)    
    
    @staticmethod
    # Function to apply affine transformation to a point
    def apply_affine_transform(point, matrix):
        point_homogeneous = np.array([point[0], point[1], 1], dtype=np.float32)
        transformed = np.dot(matrix, point_homogeneous)
        x, y, w = transformed[0], transformed[1], transformed[2]
        if w != 0:  # Normalize by w for perspective transformations
            x /= w
            y /= w
        return x, y, w
    
    @staticmethod
    def apply_inverse_affine_transform(pixel_pos, matrix):
        # Invert the affine matrix
        inv_matrix, _ = cv.invert(matrix)
        
        # Apply the inverse transformation
        pixel_homogeneous = np.array([pixel_pos[0], pixel_pos[1], 1], dtype=np.float32)
        transformed = np.dot(inv_matrix, pixel_homogeneous)
        x, y, w = transformed[0], transformed[1], transformed[2]
        
        if w != 0:  # Normalize by w for perspective transformations
            x /= w
            y /= w
        return int(x), int(y)

    
    TOP_LEFT = "top_left" 
    TOP_RIGHT = "top_right"
    BOTTOM_LEFT = "bottom_left"
    BOTTOM_RIGHT = "bottom_right"
    
    @staticmethod
    def compute_affine_transformation(corners, grid_width, grid_height):
        # Define the source (grid coordinates) and destination (image coordinates) points for affine transformation
        source_points = np.float32([
            [0, 0],  # Top-left of grid
            [grid_width-1, 0],  # Top-right of grid
            [0, grid_height-1],  # Bottom-left of grid 
            [grid_width-1, grid_height-1],  # Bottom-right of grid 
        ])
        dest_points = np.float32([
            corners[UtilityFunctions.TOP_LEFT], 
            corners[UtilityFunctions.TOP_RIGHT],
            corners[UtilityFunctions.BOTTOM_LEFT],
            corners[UtilityFunctions.BOTTOM_RIGHT],
        ])
        matrix,_ = cv.findHomography(source_points, dest_points)

        return matrix
    
    GREEN_RANGE = ((35, 50, 50), (85, 255, 255))
    RED_RANGE = ((0, 200, 200), (10, 255, 255))
    BLUE_RANGE = ((115, 100, 50), (137, 200, 200)) 
    ORANGE_RANGE = ((10, 100, 100), (25, 255, 255))

    @staticmethod
    def find_corners(image):
        color_ranges = {
            "green": UtilityFunctions.GREEN_RANGE, 
            "red": UtilityFunctions.RED_RANGE,
            "blue": UtilityFunctions.BLUE_RANGE,
            "orange": UtilityFunctions.ORANGE_RANGE
        }
        
        # Find the corners of the maze
        corners = list(UtilityFunctions.find_points(image, color_ranges).values())
        top_left = min(corners, key=lambda p: (p[0], p[1]))  
        bottom_left = min([p for p in corners if p != top_left], key=lambda p: (p[0], -p[1]))  
        top_right = min([p for p in corners if p != bottom_left and p != top_left], key=lambda p: (p[0], -p[1]))  
        bottom_right = max([p for p in corners if p != top_left and p != top_right and p != bottom_left], key=lambda p: (p[0], p[1]))  

        corners = {
            UtilityFunctions.TOP_LEFT: top_left,
            UtilityFunctions.TOP_RIGHT: top_right,
            UtilityFunctions.BOTTOM_LEFT: bottom_left,
            UtilityFunctions.BOTTOM_RIGHT: bottom_right,
        }

        return corners
       
    @staticmethod
    def find_points(image, color_ranges):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        points = {}
        for color_name, (lower, upper) in color_ranges.items():
            contours = UtilityFunctions.find_contours(hsv, lower, upper)

            # Find the centroids of the contours
            centroids = []
            for contour in contours:
                M = cv.moments(contour)
                if M["m00"] != 0:  # Avoid division by zero
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
            points[color_name] = centroids[0] 
        return points   
    
    @staticmethod
    def find_contours(hsv, lower, upper):
        # Create mask for the color
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv.inRange(hsv, lower, upper)
        
        # Find contours for the color
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        return contours
    
    @staticmethod
    def find_center_of_rectangle(positions):
        # (x,y) - (topleft,topright,bottomright,bottomleft)
        top_left = positions[0]
        bottom_right = positions[2]

        center_x = (top_left[0] + bottom_right[0]) // 2 
        center_y = (top_left[1] + bottom_right[1]) // 2 
        
        return (center_x,center_y)

    @staticmethod
    def show_node_positions(matrix, point = (0,0) ):
        affine = UtilityFunctions.apply_affine_transform(point, matrix)
        inverse = UtilityFunctions.apply_inverse_affine_transform(point, matrix)
        print(f"point: {point}, affine: {affine}, inverse: {inverse}")
