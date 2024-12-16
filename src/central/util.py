import time
import cv2 as cv
import numpy as np
import math


class UtilityFunctions:

    YELLOW=(255,255,153)
    RED=(0,0,255)
    GREEN=(0,255,0)
    BLUE=(255,0,0)
    ROBOT_ONE='robot 1'
    ROBOT_TWO='robot 2'

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
    RED_RANGE_1 = ((0, 70, 50), (10, 255, 255))
    RED_RANGE_2 = ((170, 70, 50), (180, 255, 255))

    @staticmethod
    def find_corners(image):
        color_ranges = {
            "green": UtilityFunctions.GREEN_RANGE, 
            "red": UtilityFunctions.RED_RANGE_1,
        }

        # Find the corners of the maze
        corners_found = False
        while not corners_found:
            try:
                points = UtilityFunctions.find_points(image, color_ranges)
                corners = list(points.values())
                rectangles = UtilityFunctions.make_rectangle(corners) 
                corners_found = True
            except:
                print("Corners not found, trying again...")
                time.sleep(3)  # Try again if the corners are not found,
                continue

        corners = {
            UtilityFunctions.TOP_LEFT: rectangles[0], 
            UtilityFunctions.TOP_RIGHT: rectangles[1],
            UtilityFunctions.BOTTOM_LEFT: rectangles[2],
            UtilityFunctions.BOTTOM_RIGHT: rectangles[3],
        }
        return corners
    
    @staticmethod
    def make_rectangle(points):
        if len(points) != 4:
            raise ValueError("You must provide exactly 4 coordinates.")
        
        sorted_coords = sorted(points, key=lambda p: (p[1], p[0]))  # Sort by y first (top-to-bottom), then by x (left-to-right)
        
        # First two are top points (smallest y values)
        top_left = min(sorted_coords[:2], key=lambda p: p[0])  # Left-most 
        top_right = max(sorted_coords[:2], key=lambda p: p[0])  # Right-most 
        
        # Last two are bottom points (largest y values)
        bottom_left = min(sorted_coords[2:], key=lambda p: p[0])  # Left-most 
        bottom_right = max(sorted_coords[2:], key=lambda p: p[0])  # Right-most

        return top_left, top_right, bottom_left, bottom_right 

    @staticmethod
    def find_points(image, color_ranges):
        image = cv.GaussianBlur(image, (5, 5), 0)
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        points = {}
        for color_name, (lower, upper) in color_ranges.items():
            # lower, upper = UtilityFunctions.adjust_hsv_range(hsv, lower, upper)
            contours = UtilityFunctions.find_contours(hsv, lower, upper, color_name == "red")

            centroids = []
            for contour in contours:
                M = cv.moments(contour)
                if M["m00"] != 0:  # Avoid division by zero
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    area = cv.contourArea(contour)  # Calculate the area of the contour
                    centroids.append((cx, cy, area))

            centroids = sorted(centroids, key=lambda x: x[2], reverse=True)
            best_centroid = centroids[0] if len(centroids) > 1 else None
            if best_centroid is None:
                print(f"Couldn't find 2 {color_name}: centroid/corner")
            points[f"{color_name}_1"] = centroids[0][:2]
            points[f"{color_name}_2"] = centroids[1][:2]
        return points   
    
    
    @staticmethod
    def adjust_hsv_range(hsv, lower, upper):
        v_mean = np.mean(hsv[:, :, 2])  # Compute the mean brightness (value channel)
        adjustment = max(0, 255 - v_mean) // 10  # Scale the adjustment
        lower = np.array(lower) - [0, 0, adjustment]  # Lower the value range
        upper = np.array(upper) + [0, 0, adjustment]  # Increase the value range
        return np.clip(lower, 0, 255), np.clip(upper, 0, 255)

    @staticmethod
    def find_contours(hsv, lower, upper, red=False):
        # Create mask for the color
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv.inRange(hsv, lower, upper)
        if red:
            mask = cv.inRange(hsv, UtilityFunctions.RED_RANGE_1[0], 
                              UtilityFunctions.RED_RANGE_1[1]) | cv.inRange(hsv, UtilityFunctions.RED_RANGE_2[0], UtilityFunctions.RED_RANGE_2[1])

        # Find contours for the color
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        return contours
    
    @staticmethod
    def find_center_of_rectangle(positions):
        top_left = positions[0]
        bottom_right = positions[2]

        center_x = (top_left[0] + bottom_right[0]) // 2 
        center_y = (top_left[1] + bottom_right[1]) // 2 
        
        return (center_x,center_y)

    @staticmethod
    def show_node_positions(matrix, point = (0,0)):
        affine = UtilityFunctions.apply_affine_transform(point, matrix)
        inverse = UtilityFunctions.apply_inverse_affine_transform(point, matrix)
        print(f"point: {point}, affine: {affine}, inverse: {inverse}")

    @staticmethod
    def visualize_mask(hsv, lower, upper):
        mask = cv.inRange(hsv, lower, upper)
        cv.imshow("Mask", mask)

    @staticmethod
    def kahan_sum(numbers):
        total = 0.0     
        c = 0.0         

        for num in numbers:
            y = num - c     
            t = total + y   
            c = (t - total) - y  
            total = t      

        return total    
