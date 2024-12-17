import cv2
import numpy as np
from collections import deque

# Variables to store selected points and polygon
video_capture_mode = 1
points = []
grid_size = 50  # Initial grid size
polygon = []  # To store the polygon points for object tracking
tracking = False
polygon_complete = False
phase = 0  # Tracks the current phase

# Environment dimensions in centimeters
dimensions = (92, 62.5)
environment_width_cm = dimensions[0]
environment_height_cm = dimensions[1]

# Actor class to manage individual bounding boxes and their properties
class Actor:
    def __init__(self, name):
        self.name = name
        self.tracker = cv2.TrackerCSRT_create()
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
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    if len(largest_contour) >= 5:  # PCA requires at least 5 points
                        data = largest_contour.reshape(-1, 2).astype(np.float32)
                        mean, eigenvectors = cv2.PCACompute(data, mean=None)
                        angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])
                        self.orientation = np.degrees(angle)

    def distance_from_origin_cm(self, location):
        if not self.origin or not location:
            return 0
        origin_x, origin_y = self.origin
        loc_x, loc_y = location
        width_scale = environment_width_cm / frame.shape[1]
        height_scale = environment_height_cm / frame.shape[0]
        dx = (loc_x - origin_x) * width_scale
        dy = (loc_y - origin_y) * height_scale
        return np.sqrt(dx**2 + dy**2)

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

# Mouse callback for selecting points and polygon
def click_event(event, x, y, flags, param):
    global points, polygon, tracking, polygon_complete, phase

    if event == cv2.EVENT_LBUTTONDOWN:
        if not tracking and phase == 0:
            points.append((x, y))
            print(f"Point {len(points)}: {x}, {y}")
            cv2.circle(temp_frame, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow("Video Feed", temp_frame)
        elif phase == 1 and tracking:
            if not polygon_complete:
                polygon.append((x, y))
                print(f"Polygon point {len(polygon)}: {x}, {y}")
                if len(polygon) > 1:
                    cv2.line(temp_frame, polygon[-2], polygon[-1], (255, 255, 0), 2)
                cv2.circle(temp_frame, (x, y), 5, (255, 255, 0), -1)

                if len(polygon) > 2 and np.linalg.norm(np.array(polygon[0]) - np.array(polygon[-1])) < 10:
                    polygon_complete = True
                    cv2.line(temp_frame, polygon[-1], polygon[0], (255, 255, 0), 2)
                    print("Polygon completed.")
                    add_actor_to_environment(frame, polygon)
                    polygon.clear()
                cv2.imshow("Original Video Feed", temp_frame)
        elif phase == 2:  # Orientation point selection
            if environment.actors:
                actor = environment.actors[-1]  # Assign orientation to the last added actor
                actor.set_orientation_point((x, y))
                print(f"Orientation point for {actor.name}: {x}, {y}")


def hover_event(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        width_scale = environment_width_cm / mapped_frame.shape[1]
        height_scale = environment_height_cm / mapped_frame.shape[0]
        coord_x = x * width_scale
        coord_y = y * height_scale
        hover_frame = mapped_frame.copy()
        cv2.putText(hover_frame, f"X: {coord_x:.2f} cm, Y: {coord_y:.2f} cm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Mapped Video Feed", hover_frame)

def add_actor_to_environment(frame, polygon):
    x_coords = [p[0] for p in polygon]
    y_coords = [p[1] for p in polygon]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    bbox = (min_x, min_y, max_x - min_x, max_y - min_y)
    actor = Actor(name=f"Actor_{len(environment.actors) + 1}")
    actor.initialize_tracker(frame, bbox)
    environment.add_actor(actor)

# Open video capture
cap = cv2.VideoCapture(video_capture_mode)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

environment = Environment(grid_size)

cv2.namedWindow("Video Feed")
cv2.setMouseCallback("Video Feed", click_event)

while len(points) < 4:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    temp_frame = frame.copy()
    cv2.imshow("Video Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        exit()

print(f"Selected points: {points}")
phase = 1
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
H, _ = cv2.findHomography(src_points, dst_points)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    mapped_frame = cv2.warpPerspective(frame, H, (frame.shape[1], frame.shape[0]))
    for x in range(0, mapped_frame.shape[1], environment.get_grid_size()):
        cv2.line(mapped_frame, (x, 0), (x, mapped_frame.shape[0]), (255, 0, 0), 1)
    for y in range(0, mapped_frame.shape[0], environment.get_grid_size()):
        cv2.line(mapped_frame, (0, y), (mapped_frame.shape[1], y), (255, 0, 0), 1)
    for x in range(0, mapped_frame.shape[1], environment.get_grid_size()):
        for y in range(0, mapped_frame.shape[0], environment.get_grid_size()):
            cv2.circle(mapped_frame, (x, y), 2, (0, 255, 0), -1)
    for actor in environment.actors:
        if actor.update(frame):
            top_left = (int(actor.bbox[0]), int(actor.bbox[1]))
            bottom_right = (int(actor.bbox[0] + actor.bbox[2]), int(actor.bbox[1] + actor.bbox[3]))
            cv2.rectangle(mapped_frame, top_left, bottom_right, (0, 255, 255), 2)

            # Draw orientation arrow
            if actor.orientation is not None:
                center = tuple(map(int, actor.get_location()))
                arrow_length = 50
                end_x = int(center[0] + arrow_length * np.cos(np.radians(actor.orientation)))
                end_y = int(center[1] + arrow_length * np.sin(np.radians(actor.orientation)))
                cv2.arrowedLine(mapped_frame, center, (end_x, end_y), (0, 0, 255), 2)

    cv2.imshow("Original Video Feed", frame)
    cv2.imshow("Mapped Video Feed", mapped_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
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

cap.release()
cv2.destroyAllWindows()
