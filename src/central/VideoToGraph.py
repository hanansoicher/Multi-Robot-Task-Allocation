import cv2 as cv
import networkx as nx
import numpy as np
import threading
import queue
from util import UtilityFunctions as uf
from Graph import Graph as gr

def main():
    video = "img/video/test_red_close.mov"
    vg = VideoToGraph(75, 150, video)
    vg.start_environment()
    vg.tear_down()

class VideoToGraph:

    #initialize
    def __init__(self, height, length, video_file, robots, metric = True):

        # video feed
        self.cap = self.initialize_camera(video_file)

        # Graph setup
        self.square_length_cm = length if metric else length * 2.54
        self.square_height_cm = height if metric else height * 2.54
        self.square_pixel_length = 0
        self.square_pixel_height = 0
        self.pixel_center_block_height_px  = 0
        self.pixel_center_block_length_px = 0
        self.pixel_diagonal_block_length_px = 0 
        self.graph_x_nodes = 0
        self.graph_y_nodes = 0
        self.block_size_cm = 3.5
        self.corners = {}
        self.matrix = any
        self.graph = nx.Graph()

        # shortest paths from robot to goal
        self.paths = {}
        self.robot_goals = {}

        # QR Code / robot tracking 
        self.qcd = None
        self.overlapping_nodes = set()
        self.tracked_objects = {}
        self.robots = robots

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

            refresh_graph = True if frame_count % self.overlay_update_frame_interval*3 == 0 else False
            update = frame_count % self.overlay_update_frame_interval == 0
            if update:
                graph = self.convert_image_to_graph(frame, refresh_graph)
                self.detect_objects(frame)
                refresh_graph = False

            overlay_image = self.draw_overlay(frame, graph)
            self.draw_objects_overlay(overlay_image)
            try:
                no_robots, overlay_image = self.no_robots(overlay_image)
                if no_robots:
                    pass
                else:
                    center_1 = uf.find_center_of_rectangle(self.tracked_objects['robot 1'])
                    self.robots['robot 1']['START'] = gr.find_nearest_node(self.graph, center_1)
                    paths = self.find_paths(self.robot_goals)
                    for robot_path in paths.keys():
                        if robot_path:
                            path = paths[robot_path]
                            overlay_image = gr.draw_transformed_path(overlay_image, graph, path)
                            self.paths[robot_path] = path
            except:
                if update:
                    print("Couldn't find path")

            # Display the (frame + overlay)
            if not self.frame_queue.full():
                self.frame_queue.put(overlay_image)
            frame_count += 1

    def no_robots(self, overlay_image):
        no_robots = not self.tracked_objects.__contains__('robot 1') and not self.tracked_objects.__contains__('robot 2')
        if no_robots:
            top_left = self.corners[uf.TOP_LEFT]
            bottom_right = gr.find_nearest_node(self.graph, self.corners[uf.BOTTOM_RIGHT])
            path = gr.a_star_from_pixel_pos(self.graph, top_left, bottom_right)
            if path:
                overlay_image = gr.draw_transformed_path(overlay_image, self.graph, path)
                self.paths['robot 1'] = path
                # gr.print_path_weights(self.graph, path)

            top_left = (0, 0)
            top_right = (self.graph_x_nodes - 1, 0)
            bottom_left = (0, self.graph_y_nodes - 1)
            bottom_right = (self.graph_x_nodes - 1, self.graph_y_nodes - 1)

            gr.print_path_weights(self.graph, gr.safe_astar_path(self.graph, top_left, top_right, gr.heuristic))
            gr.print_path_weights(self.graph, gr.safe_astar_path(self.graph, top_left, bottom_left, gr.heuristic))
            gr.print_path_weights(self.graph, gr.safe_astar_path(self.graph, top_left, bottom_right, gr.heuristic))
        return no_robots, overlay_image
    
    def convert_image_to_graph(self, image, refresh_graph):
        if refresh_graph:
            corners = uf.find_corners(image)
            if self.corners != corners:
                self.corners = corners
                self.set_dimensions(corners)
                self.graph = nx.grid_2d_graph(self.graph_x_nodes, self.graph_y_nodes)
                gr.add_diagonal_edges(self.graph_x_nodes, self.graph_y_nodes, self.graph)
                self.refresh_matrix(corners)
                gr.set_node_positions(self.graph, self.matrix)
                self.compute_pixel_dimensions()
                self.set_robot_goals({
                    'robot 1': (self.graph_x_nodes - 1, self.graph_y_nodes - 4),
                    'robot 2': (self.graph_x_nodes - 3, self.graph_y_nodes - 12),          
                    })
            # self.detect_static_obstacles(image, self.graph)
            self.detect_objects(image)

            gr.adjust_graph_weights(self.graph, self.pixel_center_block_length_px, self.pixel_center_block_height_px, 
                                    self.pixel_diagonal_block_length_px, self.block_size_cm)        

        return self.graph
    
    def refresh_matrix(self, corners):
        matrix = uf.compute_affine_transformation(corners, self.graph_x_nodes, self.graph_y_nodes)
        self.matrix = matrix

    def draw_overlay(self, image, graph):
        overlay_image = gr.draw_nodes_overlay(graph, image)
        overlay_image = gr.draw_edges_overlay(graph, overlay_image)
        overlay_image = self.draw_corners_overlay(overlay_image)
        return overlay_image

    def draw_objects_overlay(self, overlay_image):
        for key in self.tracked_objects.keys():
            pts = self.tracked_objects[key].astype(int)
            cv.polylines(overlay_image, [pts], isClosed=True, color=uf.GREEN, thickness=2)
            cv.putText(overlay_image, key, (pts[0][0]+20, pts[0][1]-20), cv.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 3)

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
                robot_position = self.tracked_objects[robot]
            except:
                continue
            center = uf.find_center_of_rectangle(robot_position)
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

    def compute_pixel_dimensions(self):
        try:
            center_coord_x = (int(self.graph_x_nodes/2))
            center_coord_y = (int(self.graph_y_nodes/2))
            center_node = self.graph.nodes[center_coord_x,center_coord_y][gr.PIXEL_POS]

            north_node = self.graph.nodes[center_coord_x,center_coord_y + 1][gr.PIXEL_POS]
            south_node = self.graph.nodes[center_coord_x,center_coord_y - 1][gr.PIXEL_POS]
            east_node = self.graph.nodes[center_coord_x + 1,center_coord_y][gr.PIXEL_POS]
            west_node = self.graph.nodes[center_coord_x - 1 ,center_coord_y][gr.PIXEL_POS]

            north_east_node = self.graph.nodes[center_coord_x + 1,center_coord_y + 1][gr.PIXEL_POS]
            south_east_node = self.graph.nodes[center_coord_x + 1,center_coord_y - 1][gr.PIXEL_POS]
            south_west_node = self.graph.nodes[center_coord_x - 1,center_coord_y - 1][gr.PIXEL_POS]
            north_west_node = self.graph.nodes[center_coord_x - 1,center_coord_y + 1][gr.PIXEL_POS]


            # Compute the pixel distance between the center node and its neighbors
            self.pixel_center_block_height_px = (uf.euclidean_distance(center_node, north_node) + uf.euclidean_distance(center_node, south_node)) / 2
            self.pixel_center_block_length_px = (uf.euclidean_distance(center_node, east_node) + uf.euclidean_distance(center_node, west_node)) / 2
            self.pixel_diagonal_block_length_px = (uf.euclidean_distance(center_node, north_east_node) 
                                                   + uf.euclidean_distance(center_node, south_east_node) 
                                                   + uf.euclidean_distance(center_node, south_west_node) 
                                                   + uf.euclidean_distance(center_node, north_west_node)) / 4
            
            top_left = (0, 0)
            top_right = (self.graph_x_nodes - 1, 0)
            bottom_left = (0, self.graph_y_nodes - 1)
            bottom_right = (self.graph_x_nodes - 1, self.graph_y_nodes - 1)

            length = gr.adjust_distance_based_on_correction(self.graph, top_left, top_right, 0, self.square_pixel_length, 0, self.square_length_cm)
            height = gr.adjust_distance_based_on_correction(self.graph, top_left, bottom_left, self.square_pixel_height, 0, 0, self.square_height_cm)
            diagonal = gr.adjust_distance_based_on_correction(self.graph, top_left, bottom_right, 0, 0, (self.square_pixel_height**2 + self.square_pixel_length**2)**0.5,(self.square_length_cm**2 + self.square_height_cm**2)**0.5)

            print(f"Length: {length} cm, Height: {height} cm, Diagonal: {diagonal} cm")

        except Exception as e:
            print(e)
            print("Couldn't compute pixel dimensions")

    def detect_static_obstacles(self, image, graph, proximity_threshold=60):
        overlay_image = image.copy()
        hsv_image = cv.cvtColor(overlay_image, cv.COLOR_BGR2HSV)
        pink_lower = [140, 50, 50]
        pink_upper = [170, 255, 255]
        contours = uf.find_contours(hsv_image, pink_lower, pink_upper)

        MIN_CONTOUR_AREA = 3000
        filtered_contours = [cnt for cnt in contours if cv.contourArea(cnt) > MIN_CONTOUR_AREA]

        for contour in filtered_contours:
            cv.drawContours(overlay_image, [contour], -1, uf.RED, 2)
            gr.update_graph_based_on_obstacle(graph, contour, proximity_threshold)

    def detect_objects(self, image):
        # Initialize the QR code detector
        self.qcd = cv.QRCodeDetector() if self.qcd is None else self.qcd

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
        
        # Detect and decode QR codes
        retval, decoded_infos, points, _ = self.qcd.detectAndDecodeMulti(thresh)
        
        if retval:
            for i, qr_code_info in enumerate(decoded_infos):
                self.update_position(points[i], qr_code_info)

        for key in self.tracked_objects.keys():
            try:
                if key[0] == "a":
                    qr_code_points = self.tracked_objects[key].astype(int)
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
    
    def update_position(self, position, robot):
        robot = robot if robot else self.get_nearest_object(position)
        if robot:
            try:
                previous_position = self.tracked_objects[robot]
                same_position = np.array_equal(position, previous_position)
                if not same_position:
                    self.tracked_objects[robot] = position
            except:
                self.tracked_objects[robot] = position


    def get_nearest_object(self, points, threshold=400):
        closest_id = None
        min_distance = gr.INF
        center_position = uf.find_center_of_rectangle(points)

        for obj_id, last_positions in self.tracked_objects.items():
            last_position_center = uf.find_center_of_rectangle(last_positions)
            distance = (uf.euclidean_distance(center_position, last_position_center))
            if distance < min_distance and distance < threshold:
                min_distance = distance
                closest_id = obj_id

        return closest_id
    
    def overlay_text(self, image, text, position, color=(0,0,0)):
        cv.putText(image, text, position, cv.FONT_HERSHEY_SIMPLEX, 1.3, color, 3)
        return image

if __name__ == "__main__":
    main()