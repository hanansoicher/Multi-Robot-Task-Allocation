import cv2 as cv
import networkx as nx
import numpy as np
import threading
import queue
from util import UtilityFunctions as uf
from Graph import Graph as gr
import asyncio


def main():
    video = "img/video/test_red_close.mov"
    vg = VideoToGraph(75, 150, video)
    vg.start_environment()
    vg.tear_down()

class VideoToGraph:
    
    def initialize_tracker(self, cap):

        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            cap.release()
            cv.destroyAllWindows()
            exit()

        # Initialize a list for trackers and bounding boxes
        self.robot_trackers = []
        self.robot_bounding_boxes = []
        
        # Select multiple ROIs manually or programmatically
        while True:
            # Manually select bounding boxes
            bbox = cv.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
            self.robot_bounding_boxes.append(bbox)
            
            # Initialize a tracker for this bounding box
            tracker = cv.TrackerKCF.create()  # Change tracker type if needed
            tracker.init(frame, bbox)
            self.robot_trackers.append(tracker)
            
            # Ask user if they want to select more bounding boxes
            print("Press 'y' to select another object or any other key to continue")
            key = cv.waitKey(0)
            if key != ord('y'):
                break

            cv.destroyWindow("Frame")

    #initialize
    def __init__(self, height, length, video_file, robots, metric = True):

        # video feed
        self.cap = self.initialize_camera(video_file)

        # Display toggles
        self.display_grid = True
        self.display_paths = True
        self.display_obstacles = True
        self.display_qr_objects = True
        self.display_actions_points = True
        self.display_deadline = True
        self.display_HUD = False

        # Graph setup
        self.square_length_cm = length if metric else length * 2.54
        self.square_height_cm = height if metric else height * 2.54
        self.square_pixel_length = 0
        self.square_pixel_height = 0
        self.graph_x_nodes = 0
        self.graph_y_nodes = 0
        self.block_size_cm = 3
        self.pixel_conversion = []
        self.corners = {}
        self.matrix = any
        self.graph = nx.Graph()

        # shortest paths from robot to goal
        self.paths = {}
        self.robot_goals = {}
        self.deadline_threshold = 2000

        # QR Code tracking 
        self.qcd = None
        self.overlapping_nodes = set()
        self.tracked_qr_objects = {}

        # Robot tracking
        self.tracked_robots = {}
        self.robots_colors = {uf.ROBOT_ONE: (uf.ROBOT_ONE_RANGE), uf.ROBOT_TWO: (uf.ROBOT_TWO_RANGE)}
        self.robots = robots
        self.robot_trackers = []
        self.robot_bounding_boxes = []

        # relay processed video via thread to avoid creating a blocking call
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = True
        self.thread = threading.Thread(target=self.start_environment, daemon=True)
        self.thread.start()
        self.overlay_update_frame_interval = 1

    # Video input
    def initialize_camera(self, camera = int(0)):
        capture = cv.VideoCapture(camera) # 0 is the default camera, can also take a file

        if not capture.isOpened():
            print("Cannot open camera")
            exit()

        return capture
    
    # Release the camera 
    def tear_down(self):
        self.running = False
        self.cap.release()
        try:
            self.thread.join()
        except:
            print("Thread couldn't be joined")
        cv.destroyAllWindows()

    async def get_robot_positions(self, robot):
        future = asyncio.Future()
        try:
            bbox, center = self.tracked_robots[robot]
            future.set_result(center)
        except:
            future.set_result(None)
        return await future
    
    def get_action_point(self, action_point):
        action_point = self.tracked_qr_objects[action_point] if self.tracked_qr_objects.__contains__(action_point) else None
        return action_point
    
    def get_nearest_node_to_actionpoint(self, action_point):
    
        action_point = self.get_action_point(action_point)
        if action_point is None:
            return None
        print(action_point)
        center = uf.find_center_of_rectangle(action_point)
        return gr.find_nearest_node(self.graph, center)

    # Create and update graph from the video input
    def start_environment(self):
        frame_count = 0  # Count frames to update the overlay after a set number of frames
        refresh_graph = True  
        while self.running:

            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                self.running = False
                break

            # if self.corners == {}:
#                 self.corners, self.H = uf.find_corners_feed(self.cap)

            # frame = cv.warpPerspective(frame, self.H, (frame.shape[1], frame.shape[0]))
            refresh_graph = True if frame_count % self.overlay_update_frame_interval*3 == 0 else False
            update = frame_count % self.overlay_update_frame_interval == 0
            overlay_image = frame.copy()
            if update:
                self.convert_image_to_graph(overlay_image, refresh_graph)
                self.detect_qr_objects(overlay_image)
                refresh_graph = False

            self.update_robot_positions_from_trackers(frame)
            for i in range(len(self.robot_trackers)):
                self.draw_robot_position(overlay_image, i)

            # self.detect_robots(overlay_image, self.robots_colors)
            if self.display_grid:
                self.draw_grid(overlay_image, self.graph)

            if self.display_qr_objects:
                self.draw_qr_objects(overlay_image)

            if self.display_paths:
                pass
            
            if self.display_deadline:
                self.draw_deadline(overlay_image)            
            
            
            self.draw_HUD(overlay_image)

            try:
                no_robots = self.no_robots()
                if no_robots:
                    pass
                else:
                    robot_1_center = self.tracked_robots[uf.ROBOT_ONE][1]
                    robot_1 = gr.find_nearest_node(self.graph, robot_1_center)
                    robot_2_center = self.tracked_robots[uf.ROBOT_TWO][1]
                    robot_2 = gr.find_nearest_node(self.graph, robot_2_center)
                    path = gr.safe_astar_path(self.graph, robot_1, robot_2, gr.heuristic)
                    if path is not None and self.display_paths:
                        overlay_image = gr.draw_transformed_path(overlay_image, self.graph, path)
                        gr.print_path_weights(self.graph, path)
                    
            except:
                if update:
                    pass

            # Display the (frame + overlay)
            if not self.frame_queue.full():
                self.frame_queue.put(overlay_image)
            frame_count += 1

    def no_robots(self):
        return not self.tracked_robots.__contains__(uf.ROBOT_ONE) and not self.tracked_robots.__contains__(uf.ROBOT_TWO)

    
    def convert_image_to_graph(self, image, refresh_graph):
        if refresh_graph:
            corners = uf.find_corners(image)
            if self.corners != corners:
                self.corners = corners
                self.set_dimensions(self.corners)
                self.graph = nx.grid_2d_graph(self.graph_x_nodes, self.graph_y_nodes)
                gr.add_diagonal_edges(self.graph_x_nodes, self.graph_y_nodes, self.graph)
                self.refresh_matrix(self.corners)
                gr.set_node_positions(self.graph, self.matrix)
            self.update_robot_positions_from_trackers(image)
            self.detect_static_obstacles(image)
            self.detect_qr_objects(image)
            # self.detect_robots(image, self.robots_colors)
            self.compute_pixel_conversion()
            gr.adjust_graph_weights(self.graph, self.pixel_conversion)        

        return self.graph
    
    def refresh_matrix(self, corners):
        matrix = uf.compute_affine_transformation(corners, self.graph_x_nodes, self.graph_y_nodes)
        self.matrix = matrix

    def draw_grid(self, image, graph):
        overlay_image = gr.draw_nodes_overlay(graph, image)
        overlay_image = gr.draw_edges_overlay(graph, overlay_image)
        # overlay_image = self.draw_corners_overlay(overlay_image)
        return overlay_image

    def draw_qr_objects(self, overlay_image):
        for i, key in enumerate(self.tracked_qr_objects.keys()):
            pts = self.tracked_qr_objects[key].astype(int)
            cv.polylines(overlay_image, [pts], isClosed=True, color=uf.YELLOW, thickness=2)
            cv.putText(overlay_image, f"Action point {i+1}", (pts[0][0]+20, pts[0][1]-20), cv.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 3)

    def draw_corners_overlay(self, overlay_image):
        max_y = max(self.corners.values(), key=lambda p: p[1])[1]
        for corner_name, (x,y) in self.corners.items():
            if y < max_y / 2:
                x += 50 # text position adjustment
                y -= 100 
            cv.putText(overlay_image, corner_name, (x-150, y+50), cv.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,0), 3)
        return overlay_image
    
    def find_paths(self, robot_goal):
        paths = {}
        for robot, goal in robot_goal.items():
            try:
                _, center = self.tracked_qr_objects[robot]
            except:
                continue
            path = gr.a_star_from_pixel_pos(self.graph, center, goal)
            paths[robot] = path
        return paths

    def set_robot_goals(self, goals):
        for robot, goal in goals.items():
            self.robot_goals[robot] = goal

    def set_dimensions(self, corners):
        try:
            # Compute grid dimensions based on the block size and image size
            self.square_pixel_length = corners[uf.TOP_RIGHT][0] - corners[uf.TOP_LEFT][0]
            self.square_pixel_height = corners[uf.BOTTOM_RIGHT][1] - corners[uf.TOP_LEFT][1]

            pixel_block_height_px  = (self.block_size_cm / self.square_height_cm) * self.square_pixel_height
            pixel_block_length_px = (self.block_size_cm / self.square_length_cm) * self.square_pixel_length

            self.graph_x_nodes = int(self.square_pixel_length / pixel_block_length_px)
            self.graph_y_nodes = int(self.square_pixel_height / pixel_block_height_px)
        except:
            print("Couldn't set the dimensions")    

    def compute_pixel_conversion(self):
        try:
            self.pixel_conversion.append(self.square_length_cm / self.square_pixel_length)
            self.pixel_conversion.append(self.square_height_cm / self.square_pixel_height)
            self.pixel_conversion.append((self.square_length_cm**2 + self.square_height_cm**2) ** 0.5 / (self.square_pixel_length**2 + self.square_pixel_height**2) ** 0.5)
        except Exception as e:
            print(e)
            print("Couldn't compute pixel dimensions")

    def detect_static_obstacles(self, image, proximity_threshold=60):
        overlay_image = image.copy()
        hsv_image = cv.cvtColor(overlay_image, cv.COLOR_BGR2HSV)
        pink_lower = [140, 50, 50]
        pink_upper = [170, 255, 255]
        contours = uf.find_contours(hsv_image, pink_lower, pink_upper)

        MIN_CONTOUR_AREA = 3000
        filtered_contours = [cnt for cnt in contours if cv.contourArea(cnt) > MIN_CONTOUR_AREA]

        for contour in filtered_contours:
            cv.drawContours(overlay_image, [contour], -1, uf.RED, 2)
            gr.update_graph_based_on_obstacle(self.graph, contour, proximity_threshold)

    def update_robot_positions_from_trackers(self, image):
        for i, tracker in enumerate(self.robot_trackers):
            ok, bbox = tracker.update(image)
            if ok:
                # Draw bounding box
                # Tracking success
                (x, y, w, h) = [int(v) for v in bbox]
                top_left = (x, y)
                bottom_right = (x + w, y + h)
                center_x = (top_left[0] + bottom_right[0]) // 2 
                center_y = (top_left[1] + bottom_right[1]) // 2 
                center = (center_x, center_y)
                self.update_robot_position((bbox,center), i)
            else:
                # Tracking failure
                cv.putText(image, f"Tracking failed for Node {i}", (100, 50 + i * 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


    def detect_robots(self, image, color_ranges):
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        robots = {}

        for robot, (lower, upper) in color_ranges.items():
            contours = uf.find_contours(hsv_image, lower, upper)
            MIN_CONTOUR_AREA = 6000
            largest_contour = max(contours, key=cv.contourArea)
            for cnt in contours:
                if cv.contourArea(cnt) > MIN_CONTOUR_AREA:
                    center = uf.find_center_of_contour(largest_contour)
                    if center is not None:
                        cx, cy,_ = center    
            robots[robot] = (largest_contour, (cx, cy))
        
        for robot, (contours, (cx,cy)) in robots.items():
            self.update_robot_position((contours, (cx,cy)), robot)
            image = self.draw_robot_position(image, robot)        

    def draw_robot_position(self, image, robot, outline_color = uf.GREEN,center_color = uf.RED):
        if robot in self.tracked_robots.keys():
            (bbox, center) = self.tracked_robots[robot]
            # cv.drawContours(image, [contours], -1, outline_color, 2)
            (x, y, w, h) = [int(v) for v in bbox]
            cv.rectangle(image, (x, y), (x + w, y + h), outline_color, 2)
            cv.circle(image, center, 7, center_color, -1)
            self.outline_text(image, f"robot {robot}", (center[0]-65, center[1]-20), color=uf.GREEN, scale=1.2, outline=4)
        return image

    def update_robot_position(self, bbox, robot):
        robot = robot if robot else None
        if robot:
            try:
                previous_position = self.tracked_robots[robot]
                same_position = np.array_equal(bbox[1], previous_position[1])
                if not same_position:
                    self.tracked_robots[robot] = bbox
            except:
                self.tracked_robots[robot] = bbox



    def detect_qr_objects(self, image):
        # Initialize the QR code detector
        self.qcd = cv.QRCodeDetector() if self.qcd is None else self.qcd

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
        
        # Detect and decode QR codes
        retval, decoded_infos, points, _ = self.qcd.detectAndDecodeMulti(thresh)
        
        if retval:
            for i, qr_code_info in enumerate(decoded_infos):
                self.update_position(points[i], qr_code_info)

        for key in self.tracked_qr_objects.keys():
            try:
                if key[0] == "a":
                    qr_code_points = self.tracked_qr_objects[key].astype(int)
                    overlapping_nodes = self.check_qr_code_overlap(self.graph, qr_code_points)
                    gr.update_graph_based_on_qr_code(self.graph, overlapping_nodes, self.overlapping_nodes)
                    self.overlapping_nodes = overlapping_nodes
            except:
                print(f"Invalid QR code detected: {key}")
    
    def check_qr_code_overlap(self, graph, qr_code_points, proximity_threshold=25):
        # Get the bounding box of the QR code in graph space
        min_x = min([pt[0] for pt in qr_code_points])
        max_x = max([pt[0] for pt in qr_code_points])
        min_y = min([pt[1] for pt in qr_code_points])
        max_y = max([pt[1] for pt in qr_code_points])
        
        # Check for nodes within the bounding box
        overlapping_nodes = gr.find_nodes_within_bounding_box(graph, min_x, max_x, min_y, max_y, proximity_threshold)    
        return overlapping_nodes
    
    def update_position(self, position, qr_object):
        qr_object = qr_object if qr_object else self.get_nearest_object(position)
        if qr_object:
            try:
                previous_position = self.tracked_qr_objects[qr_object]
                same_position = np.array_equal(position, previous_position)
                if not same_position:
                    self.tracked_qr_objects[qr_object] = position
            except:
                self.tracked_qr_objects[qr_object] = position

    def get_nearest_object(self, points, threshold=400):
        closest_id = None
        min_distance = gr.INF
        center_position = uf.find_center_of_rectangle(points)

        for obj_id, last_positions in self.tracked_qr_objects.items():
            last_position_center = uf.find_center_of_rectangle(last_positions)
            distance = (uf.euclidean_distance(center_position, last_position_center))
            if distance < min_distance and distance < threshold:
                min_distance = distance
                closest_id = obj_id

        return closest_id
    
    def display_robot_instructions(self, overlay_image, instructions):
        pos_x, pos_y = self.get_corner_position_for_text(multiplier=9)
        for (robot, instruction) in instructions:
            overlay_image = self.outline_text(overlay_image, f"{robot}: {instruction}", (pos_x, pos_y), color=uf.GREEN, scale=1.2, outline=4)
            pos_y += uf.TEXT_DISTANCE

    def draw_deadline(self, overlay_image):
        if self.display_deadline:
            pos_x, pos_y = self.get_corner_position_for_text(multiplier=6)
            self.outline_text(overlay_image, f"Deadline threshold: {self.deadline_threshold}", (pos_x, pos_y), color=uf.GREEN, scale=1.2, outline=4)
    
    def draw_HUD(self, overlay_image):
        pos_x, pos_y = self.get_corner_position_for_text(multiplier=20)
        if self.display_HUD:
            overlay_image = self.outline_text(overlay_image, f"Grid: {self.display_grid}", (pos_x, pos_y), scale=1.2,outline=4)
            overlay_image = self.outline_text(overlay_image, f"Paths: {self.display_paths}", (pos_x, pos_y - 1*uf.TEXT_DISTANCE), scale=1.2,outline=4)
            overlay_image = self.outline_text(overlay_image, f"Action points: {self.display_actions_points}", (pos_x, pos_y - 2*uf.TEXT_DISTANCE), scale=1.2,outline=4)
            overlay_image = self.outline_text(overlay_image, f"Deadline threshold: {self.display_deadline}", (pos_x, pos_y - 3*uf.TEXT_DISTANCE), scale=1.2,outline=4)
            overlay_image = self.outline_text(overlay_image, f"QR objects: {self.display_qr_objects}", (pos_x, pos_y - 4*uf.TEXT_DISTANCE), scale=1.2,outline=4)
            overlay_image = self.outline_text(overlay_image, f"Obstacles: {self.display_obstacles}", (pos_x, pos_y - 5*uf.TEXT_DISTANCE), scale=1.2,outline=4)

        overlay_image = self.outline_text(overlay_image, f"Options: {self.display_HUD}", (pos_x, pos_y+uf.TEXT_DISTANCE), scale=1.2,outline=4)

    def get_corner_position_for_text(self, x = 10, y = 20, multiplier = 1):
        pos_x, pos_y = (self.corners[uf.TOP_LEFT][0]) // x, (self.square_pixel_height // y) * multiplier 
        return (pos_x, pos_y)

    def overlay_text(self, image, text, position, color=(0,0,0), scale=1.3):
        cv.putText(image, text, position, cv.FONT_HERSHEY_SIMPLEX, scale, (color), thickness=2, lineType=cv.LINE_AA)
        return image
    
    def outline_text(self, image, text, position, color=(255,255,255), scale=1.3, outline=2):
        font = cv.FONT_HERSHEY_SIMPLEX
        color_outline = (0, 0, 0)  # Black outline

        cv.putText(image, text, position, font, scale, color_outline, outline, lineType=cv.LINE_AA)
        cv.putText(image, text, position, font, scale, color, outline-2, lineType=cv.LINE_AA)
        return image

    def check_weights(self):
        length = gr.safe_astar_path(self.graph, (0,0), (self.graph_x_nodes-1,0), gr.heuristic)
        height = gr.safe_astar_path(self.graph, (0,0), (0, self.graph_y_nodes-1), gr.heuristic)
        diagonal = gr.safe_astar_path(self.graph, (0,0), (self.graph_x_nodes - 1 , self.graph_y_nodes-1), gr.heuristic)

        print("Length, height, diagonal")
        gr.print_path_weights(self.graph, length)
        gr.print_path_weights(self.graph, height)
        gr.print_path_weights(self.graph, diagonal)

if __name__ == "__main__":
    main()