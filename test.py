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

# Initial values for the kernel size and standard deviation
kernel_size = 5  # Start with a 5x5 kernel
stddev = 0  # Start with a standard deviation of 0

# Environment dimensions in centimeters
dimensions = (92, 62.5)
environment_width_cm = dimensions[0]
environment_height_cm = dimensions[1]

# Actor class to manage individual bounding boxes and their properties
# Actor class to manage individual bounding boxes and their properties
class Actor:
    def __init__(self, name):
        self.name = name
        self.bbox = None
        self.history = deque(maxlen=300)
        self.template = None
        self.origin = None

    def update_bbox(self, new_bbox):
        if self.history and bbox_difference(new_bbox, self.history[-1]) > 50:
            print(f"Sharp change detected for {self.name}. Applying smoothing.")
            smoothed_bbox = compute_average_bbox(self.history)
            self.bbox = smoothed_bbox if smoothed_bbox else new_bbox
        else:
            self.bbox = new_bbox

        self.history.append(new_bbox)

        if self.origin is None:
            self.origin = self.get_location()

        current_location = self.get_location()
        if self.origin and self.distance_from_origin_cm(current_location) > 2:
            print(f"{self.name} moved to {current_location} cm")

    def get_latest_bbox(self):
        return self.bbox

    def get_location(self):
        if not self.bbox:
            return None
        top_left, bottom_right = self.bbox
        center_x = (top_left[0] + bottom_right[0]) / 2
        center_y = (top_left[1] + bottom_right[1]) / 2
        return center_x, center_y

    def distance_from_origin_cm(self, location):
        if not self.origin or not location:
            return 0
        origin_x, origin_y = self.origin
        loc_x, loc_y = location
        # Convert pixel distances to centimeters
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
            actor.origin = None

# Function to calculate the difference between two bounding boxes
def bbox_difference(bbox1, bbox2):
    x1, y1 = bbox1[0]
    x2, y2 = bbox2[0]
    return np.linalg.norm(np.array([x2 - x1, y2 - y1]))

# Function to compute the average bounding box over the last 50 frames
def compute_average_bbox(bbox_history, lookback_window=15):
    if len(bbox_history) < lookback_window:
        return None
    recent_bboxes = list(bbox_history)[-lookback_window:]
    x1_avg = np.mean([bbox[0][0] for bbox in recent_bboxes])
    y1_avg = np.mean([bbox[0][1] for bbox in recent_bboxes])
    x2_avg = np.mean([bbox[1][0] for bbox in recent_bboxes])
    y2_avg = np.mean([bbox[1][1] for bbox in recent_bboxes])
    return ((int(x1_avg), int(y1_avg)), (int(x2_avg), int(y2_avg)))

# Mouse callback for selecting points and polygon
def click_event(event, x, y, flags, param):
    global points, polygon, tracking, polygon_complete

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
                    save_template(frame, polygon)
                    polygon.clear()
                cv2.imshow("Original Video Feed", temp_frame)

# Function to handle mouse movement for coordinate display
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
        
def filter_white(img):
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 20, 255])
    white_mask = cv2.inRange(hsv_frame, lower_white, upper_white)
    inverse_white_mask = cv2.bitwise_not(white_mask)
    return cv2.bitwise_and(img, img, mask=inverse_white_mask)

def pre_process_for_template(img):
    fgbg = cv2.createBackgroundSubtractorMOG2()
    fgmask = fgbg.apply(img)
    frame_with_mask = cv2.bitwise_and(img, img, mask=fgmask)
    frame_with_mask = filter_white(frame_with_mask)
    gray_template = cv2.cvtColor(frame_with_mask, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    equalized_template = clahe.apply(gray_template)
    return np.float32(equalized_template)

def save_template(frame, polygon):
    mask = np.zeros_like(frame, dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(polygon, dtype=np.int32)], (255, 255, 255))
    masked_image = cv2.bitwise_and(frame, mask)
    x_coords = [p[0] for p in polygon]
    y_coords = [p[1] for p in polygon]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    template = masked_image[min_y:max_y, min_x:max_x]
    grad_template = pre_process_for_template(template)
    cv2.imwrite("template_gradient.png", grad_template)
    print("Template gradient saved to 'template_gradient.png'.")
    actor = Actor(name="Bob")
    actor.template = grad_template
    actor.bbox = ((min_x, min_y), (max_x, max_y))
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
        grad_frame = pre_process_for_template(frame)
        grad_frame = cv2.GaussianBlur(grad_frame, (kernel_size, kernel_size), stddev)
        result = cv2.matchTemplate(grad_frame, np.float32(actor.template), cv2.TM_SQDIFF)
        min_val, _, min_loc, _ = cv2.minMaxLoc(result)
        min_x, min_y = min_loc
        max_x = min_x + actor.template.shape[1]
        max_y = min_y + actor.template.shape[0]
        new_bbox = ((min_x, min_y), (max_x, max_y))
        actor.update_bbox(new_bbox)
        top_left = tuple(actor.get_latest_bbox()[0])
        bottom_right = tuple(actor.get_latest_bbox()[1])
        cv2.rectangle(mapped_frame, top_left, bottom_right, (0, 255, 255), 2)
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
    elif key == ord('j'):
        kernel_size += 2
    elif key == ord('u'):
        kernel_size -= 2
    elif key == ord('k'):
        stddev += 1
    elif key == ord('i'):
        stddev = max(0, stddev - 1)
    elif key == ord('r'):
        environment.reset_bounding_boxes()

cap.release()
cv2.destroyAllWindows()
