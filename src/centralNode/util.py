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
        return x, y

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
            corners["top_left"], 
            corners["top_right"],
            corners["bottom_left"],
            corners["bottom_right"],
        ])
        matrix,_ = cv.findHomography(source_points, dest_points)

        return matrix
    
    @staticmethod
    def find_corners(image):
        color_ranges = {
            "green": ((35, 50, 50), (85, 255, 255)),  # Green range
            "red": ((0, 200, 200), (10, 255, 255)),    # Red range (low range)
            "blue": ((115, 100, 50), (137, 200, 200)), # Blue range
            "orange": ((10, 100, 100), (25, 255, 255)), # Orange range
        }
        
        # Find the corners of the maze
        corners = list(UtilityFunctions.find_points(image, color_ranges).values())
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
            
            # Store centroids in the dictionary
            points[color_name] = centroids[0] #if len(centroids) == 1 else centroids 
        
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
    # Display image close window when q is pressed
    def show_image(opencv_image):
        while True:
            cv.imshow("Threshold", opencv_image)
            if cv.waitKey(1) == ord('q'):
                break
        cv.destroyAllWindows()