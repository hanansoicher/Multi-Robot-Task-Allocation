import time
import cv2 as cv
import numpy as np
import math
from collections import deque

def main():
    image = cv.imread("img/test_screen-kopi.png")
    rgb = [66, 56, 47]
    UtilityFunctions.get_color_range(image, rgb)

# Environment dimensions in centimeters
dimensions = (92, 62.5)
environment_width_cm = dimensions[0]
environment_height_cm = dimensions[1]

# Actor class to manage individual bounding boxes and their properties
class Actor:
    def __init__(self, name):
        self.name = name
        self.tracker = cv.TrackerCSRT_create()
        self.bbox = None
        self.history = deque(maxlen=300)
        self.origin = None
        self.orientation = None  # Angle in degrees
        self.orientation_point = None  # Point indicating orientation

    def initialize_tracker(self, frame, bbox):
        self.bbox = bbox
        self.tracker.init(frame, bbox)

    def update(self, frame):
        success, new_bbox = self.tracker.update(frame)
        if success:
            self.bbox = tuple(map(int, new_bbox))
            self.history.append(self.get_location())
            if self.orientation_point:
                self.update_orientation(frame)
        return success

    def get_location(self):
        if not self.bbox:
            return None
        x, y, w, h = self.bbox
        center_x = x + w / 2
        center_y = y + h / 2
        return center_x, center_y

    def set_orientation_point(self, point):
        self.orientation_point = point
        self.update_orientation_from_point()

    def update_orientation_from_point(self):
        if self.bbox and self.orientation_point:
            center = self.get_location()
            if center:
                dx = self.orientation_point[0] - center[0]
                dy = self.orientation_point[1] - center[1]
                self.orientation = np.degrees(np.arctan2(dy, dx))

    def update_orientation(self, frame):
        if self.bbox:
            x, y, w, h = map(int, self.bbox)
            roi = frame[y:y+h, x:x+w]
            if roi.size > 0:
                gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
                _, binary = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
                contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv.contourArea)
                    if len(largest_contour) >= 5:  # PCA requires at least 5 points
                        data = largest_contour.reshape(-1, 2).astype(np.float32)
                        mean, eigenvectors = cv.PCACompute(data, mean=None)
                        angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])
                        self.orientation = np.degrees(angle)

    def distance_from_origin_cm(self, frame, location):
        if not self.origin or not location:
            return 0
        origin_x, origin_y = self.origin
        loc_x, loc_y = location
        width_scale = environment_width_cm / frame.shape[1]
        height_scale = environment_height_cm / frame.shape[0]
        dx = (loc_x - origin_x) * width_scale
        dy = (loc_y - origin_y) * height_scale
        return np.sqrt(dx**2 + dy**2)

    def get_bbox(self):
        """Return bounding box coordinates in clockwise order starting from top-left."""
        if not self.bbox:
            return None
        x, y, w, h = self.bbox
        top_left = [x, y]
        top_right = [x + w, y]
        bottom_right = [x + w, y + h]
        bottom_left = [x, y + h]
        return [top_left, top_right, bottom_right, bottom_left]
    
    def intersects_with(self, other_bbox):
        """Check if this Actor's bounding box intersects with another bounding box."""
        if not self.bbox or not other_bbox:
            return False

        print("ours", self.bbox, "theirs", other_bbox)
        x1, y1, w1, h1 = self.bbox
        x2, y2, w2, h2 = other_bbox

        return not (x1 + w1 < x2 or x2 + w2 < x1 or y1 + h1 < y2 or y2 + h2 < y1)


# Environment class to manage actors and grid information
class Environment:
    def __init__(self, grid_size):
        self.actors = []
        self.grid_size = grid_size

    def add_actor(self, actor):
        self.actors.append(actor)

    def update_grid_size(self, new_grid_size):
        self.grid_size = new_grid_size

    def get_grid_size(self):
        return self.grid_size

    def reset_bounding_boxes(self):
        for actor in self.actors:
            actor.history.clear()
            actor.bbox = None
            actor.tracker.clear()


# Variables to store selected points and polygon
temp_frame = None
video_capture_mode = 1
points = []
grid_size = 50  # Initial grid size
polygon = []  # To store the polygon points for object tracking
tracking = False
polygon_complete = False
phase = 1  # Tracks the current phase

# Environment dimensions in centimeters
dimensions = (92, 62.5)
environment_width_cm = dimensions[0]
environment_height_cm = dimensions[1]
environment = Environment(grid_size)

def add_actor_to_environment(frame, polygon, name):
    x_coords = [p[0] for p in polygon]
    y_coords = [p[1] for p in polygon]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    bbox = (min_x, min_y, max_x - min_x, max_y - min_y)
    actor = Actor(name=name)
    actor.initialize_tracker(frame, bbox)
    environment.add_actor(actor)


class UtilityFunctions:

    YELLOW=(255,255,153)
    RED=(0,0,255)
    GREEN=(0,255,0)
    BLUE=(255,0,0)
    ROBOT_ONE="robot 1"
    ROBOT_TWO="robot 2"
    ROBOT_ONE_RANGE = ((100, 150, 0), (140, 255, 255))
    ROBOT_TWO_RANGE = ((4, 53, 50), (24, 93, 86))
    TEXT_DISTANCE = 65
    @staticmethod
    def get_color_range(image, rgb):

        # Convert RGB to HSV using OpenCV
        rgb_color = np.uint8([[rgb]])  # Input color in RGB
        hsv_color = cv.cvtColor(rgb_color, cv.COLOR_RGB2HSV)
        hue, sat, val = hsv_color[0][0]

        # Define the color range (you can tweak the ± values as needed)
        lower_bound = np.array([hue - 10, max(50, sat - 20), max(50, val - 20)], dtype=np.uint8)
        upper_bound = np.array([hue + 10, min(255, sat + 20), min(255, val + 20)], dtype=np.uint8)

        print("Lower HSV Bound:", lower_bound)
        print("Upper HSV Bound:", upper_bound)

        # Example of applying this to a mask
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Convert image to HSV
        mask = cv.inRange(hsv_image, lower_bound, upper_bound)  # Create a mask

        # Optional: Visualize the result
        result = cv.bitwise_and(image, image, mask=mask)
        cv.imshow("Original Image", image)
        cv.imshow("Mask", mask)
        cv.imshow("Result", result)
        cv.waitKey(0)
        cv.destroyAllWindows()

    @staticmethod
    def find_corners_feed(cap):
        
        points = []
        def click_event(event, x, y, flags, param):

            if event == cv.EVENT_LBUTTONDOWN:
                points.append((x, y))
                print(f"Point {len(points)}: {x}, {y}")
                cv.circle(temp_frame, (x, y), 5, (0, 0, 255), -1)
                cv.imshow("Video Feed", temp_frame)

        cv.namedWindow("Video Feed")
        cv.setMouseCallback("Video Feed", click_event)
        while len(points) < 4:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            temp_frame = frame.copy()
            cv.imshow("Video Feed", frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                cv.destroyAllWindows()
                exit()
        
        rectangles = UtilityFunctions.make_rectangle(points)
        corners = {
            UtilityFunctions.TOP_LEFT: rectangles[0], 
            UtilityFunctions.TOP_RIGHT: rectangles[1],
            UtilityFunctions.BOTTOM_LEFT: rectangles[2],
            UtilityFunctions.BOTTOM_RIGHT: rectangles[3],
        }

        points = sorted(points, key=lambda p: (p[1], p[0]))
        if points[0][0] > points[1][0]:
            points[0], points[1] = points[1], points[0]
        if points[2][0] > points[3][0]:
            points[2], points[3] = points[3], points[2]
            
        src_points = np.array(points, dtype=np.float32)
        dst_points = np.array([
            [0, 0],
            [frame.shape[1], 0],
            [0, frame.shape[0]],
            [frame.shape[1], frame.shape[0]]
        ], dtype=np.float32)

        H, _ = cv.findHomography(src_points, dst_points)
        return corners, H
    
    
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

    tracking = False 
    def click_event(event, x, y, flags, param):
        global points, polygon, tracking, polygon_complete

        if event == cv.EVENT_LBUTTONDOWN:
            if not tracking:
                points.append((x, y))
                print(f"Point {len(points)}: {x}, {y}")
                cv.circle(temp_frame, (x, y), 5, (0, 0, 255), -1)
                cv.imshow("Video Feed", temp_frame)

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
                try:
                    cx, cy, area = UtilityFunctions.find_center_of_contour(contour)
                    if cx is not None:
                        centroids.append((cx, cy, area))
                except:
                    continue
            centroids = sorted(centroids, key=lambda x: x[2], reverse=True)
            best_centroid = centroids[0] if len(centroids) > 1 else None
            if best_centroid is None:
                print(f"Couldn't find 2 {color_name}: centroid/corner")
            points[f"{color_name}_1"] = centroids[0][:2]
            points[f"{color_name}_2"] = centroids[1][:2]
        return points   
    
    @staticmethod
    def find_center_of_contour(contour):
        M = cv.moments(contour)
        if M["m00"] != 0:  # Avoid division by zero
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            area = cv.contourArea(contour)  # Calculate the area of the contour

            return cx, cy, area
        return None
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
    

    @staticmethod
    def get_all_objects(H, cap):
        global temp_frame
        global points, polygon, tracking, polygon_complete, phase

        def click_event(event, x, y, flags, param):
            global temp_frame
            global points, polygon, tracking, polygon_complete, phase

            if event == cv.EVENT_LBUTTONDOWN:
                if not tracking and phase == 0:
                    points.append((x, y))
                    print(f"Point {len(points)}: {x}, {y}")
                    cv.circle(temp_frame, (x, y), 5, (0, 0, 255), -1)
                    cv.imshow("Video Feed", temp_frame)
                elif phase == 1 and tracking:
                    if not polygon_complete:
                        polygon.append((x, y))
                        print(f"Polygon point {len(polygon)}: {x}, {y}")
                        if len(polygon) > 1:
                            cv.line(temp_frame, polygon[-2], polygon[-1], (255, 255, 0), 2)
                        cv.circle(temp_frame, (x, y), 5, (255, 255, 0), -1)

                        if len(polygon) > 2 and np.linalg.norm(np.array(polygon[0]) - np.array(polygon[-1])) < 10:
                            polygon_complete = True
                            cv.line(temp_frame, polygon[-1], polygon[0], (255, 255, 0), 2)
                            print("Polygon completed.")

                            name = str(input("\nEnter a name for the entity")).strip()
                            add_actor_to_environment(frame, polygon, name)
                            polygon.clear()
                            print("Added new actor", name)
                        cv.imshow("Original Video Feed", temp_frame)
                elif phase == 2:  # Orientation point selection
                    if environment.actors:
                        actor = environment.actors[-1]  # Assign orientation to the last added actor
                        actor.set_orientation_point((x, y))
                        print(f"Orientation point for {actor.name}: {x}, {y}")

        cv.namedWindow("Video Feed")
        cv.setMouseCallback("Video Feed", click_event)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            temp_frame = frame.copy()
            mapped_frame = frame.copy() #cv.warpPerspective(frame, H, (frame.shape[1], frame.shape[0]))
            for x in range(0, mapped_frame.shape[1], environment.get_grid_size()):
                cv.line(mapped_frame, (x, 0), (x, mapped_frame.shape[0]), (255, 0, 0), 1)
            for y in range(0, mapped_frame.shape[0], environment.get_grid_size()):
                cv.line(mapped_frame, (0, y), (mapped_frame.shape[1], y), (255, 0, 0), 1)
            for x in range(0, mapped_frame.shape[1], environment.get_grid_size()):
                for y in range(0, mapped_frame.shape[0], environment.get_grid_size()):
                    cv.circle(mapped_frame, (x, y), 2, (0, 255, 0), -1)
            for actor in environment.actors:
                if actor.update(frame):
                    top_left = (int(actor.bbox[0]), int(actor.bbox[1]))
                    bottom_right = (int(actor.bbox[0] + actor.bbox[2]), int(actor.bbox[1] + actor.bbox[3]))
                    cv.rectangle(mapped_frame, top_left, bottom_right, (0, 255, 255), 2)

                    # Draw orientation arrow
                    if actor.orientation is not None:
                        center = tuple(map(int, actor.get_location()))
                        arrow_length = 50
                        end_x = int(center[0] + arrow_length * np.cos(np.radians(actor.orientation)))
                        end_y = int(center[1] + arrow_length * np.sin(np.radians(actor.orientation)))
                        cv.arrowedLine(mapped_frame, center, (end_x, end_y), (0, 0, 255), 2)
                    
                    # Display actor name at the center of the bounding box
                    if actor.name:
                        text_size = cv.getTextSize(actor.name, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        text_x = int(actor.bbox[0] + actor.bbox[2] / 2 - text_size[0] / 2)
                        text_y = int(actor.bbox[1] + actor.bbox[3] / 2 + text_size[1] / 2)
                        cv.putText(mapped_frame, actor.name, (text_x, text_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


            cv.imshow("Original Video Feed", frame)
            cv.imshow("Mapped Video Feed", mapped_frame)
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                cv.destroyAllWindows()
                return { v.name : v for v in environment.actors}
            elif key == ord('n'):
                tracking = not tracking
                polygon_complete = False
                polygon = []
            elif key == ord('a'):
                environment.update_grid_size(environment.get_grid_size() + 10)
            elif key == ord('b'):
                environment.update_grid_size(max(10, environment.get_grid_size() - 10))
            elif key == ord('r'):
                environment.reset_bounding_boxes()
            elif key == ord('o'):
                phase = 2  # Switch to orientation point selection phase




if __name__ == '__main__':
    main()