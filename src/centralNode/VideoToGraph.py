import cv2 as cv
import networkx as nx
import numpy as np

from util import UtilityFunctions as uf
from graph import Graph as gr

def main():
    video = "img/video/test_with_block.mov"

    vg = VideoToGraph(75, 150, video)
    vg.start_environment(vg.cap)
    vg.tear_down()

class VideoToGraph:

    #initilaize
    def __init__(self, width, length, video_file, metric = True):
        self.cap = self.initialize_camera(video_file)
        self.maze_height = width if metric else width * 2.54
        self.maze_length = length if metric else length * 2.54
        self.grid_height = 0
        self.grid_width = 0
        self.corners = {}
        self.matrix = any
        self.graph = nx.Graph()
        self.qcd = None
        self.tracked_objects = {}


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
    def start_environment(self, cap, overlay_update_frame_interval=40):
        frame_count = 0  # Count frames to update the overlay after a set number of frames
        refresh = True  # Flag to refresh the graph
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            refresh = True if frame_count % overlay_update_frame_interval*3 == 0 else False
            if frame_count % overlay_update_frame_interval == 0:
                graph = self.convert_image_to_graph(frame, refresh)
                self.detect_objects(frame)
                refresh = False
     

            overlay_image = self.draw_overlay(frame, graph)
            self.draw_objects_overlay(overlay_image)
            try:
                robot_2 = self.tracked_objects['robot 2']
                center = uf.find_center_of_square(robot_2)

                # (41, 20): (np.float64(2131.0000000000005), np.float64(1044.0), np.float64(0.9063705759888968))
                print(center)

                nearest_point = gr.find_nearest_node(self.graph, center)
                print("neareast pont:", nearest_point)

                path = gr.safe_astar_path(graph, nearest_point, (self.grid_width - 1, self.grid_height - 4), gr.heuristic)
                if path:
                    overlay_image = gr.draw_transformed_path(overlay_image, graph, path)
            except:
                    print("Couldn't find robot_2")



            # Display the (frame + overlay)
            cv.imshow('frame_with_overlay', overlay_image)

            # Exit condition
            if cv.waitKey(1) == ord('q'):
                break
            frame_count += 1
    
    def convert_image_to_graph(self, image, refresh):
        if refresh:
            corners = uf.find_corners(image)
            if self.corners != corners:
                print("updating corners")
                self.corners = corners
                self.set_dimensions(corners)
                self.graph = nx.grid_2d_graph(self.grid_width, self.grid_height)
                gr.add_diagonal_edges(self.grid_width, self.grid_height, self.graph)
                self.refresh_matrix(corners)
                gr.set_node_positions(self.graph, self.matrix)
            gr.adjust_graph_weights(self.graph)        
            self.detect_static_obstacles(image, self.graph)
            gr.update_graph_weights_based_on_obstacles(self.graph)
            self.refresh_matrix(self.corners)

        self.detect_objects(image)
        for key in self.tracked_objects.keys():
            try:
                if key[0] == "a":
                    qr_code_points = self.tracked_objects[key].astype(int)
                    overlapping_nodes = self.check_qr_code_overlap(self.graph, qr_code_points)
                    gr.update_graph_based_on_qr_code(self.graph, overlapping_nodes)
            except :
                print(f"Invalid QR code detected: {key}")
        return self.graph
    
    def refresh_matrix(self, corners):
        matrix = uf.compute_affine_transformation(corners, self.grid_width, self.grid_height)
        self.matrix = matrix

    def draw_overlay(self, image, graph,):
        overlay_image = gr.draw_nodes_overlay(graph, image)
        overlay_image = gr.draw_edges_overlay(graph, overlay_image)
        return overlay_image

    def draw_objects_overlay(self, overlay_image):
        for key in self.tracked_objects.keys():
            pts = self.tracked_objects[key].astype(int)
            cv.polylines(overlay_image, [pts], isClosed=True, color=uf.GREEN, thickness=2)
            cv.putText(overlay_image, key, (pts[0][0]+20, pts[0][1]-20), cv.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 3)


    def set_dimensions(self, corners, block_size_cm=3.5):
        # Compute grid dimensions based on the block size and image size
        image_width_px = corners['top_right'][0] - corners['top_left'][0]
        image_height_px = corners['bottom_left'][1] - corners['top_left'][1]

        pixel_block_height_px  = (block_size_cm / self.maze_height) * image_height_px
        pixel_block_width_px = (block_size_cm / self.maze_length) * image_width_px

        self.grid_width = int(image_width_px / pixel_block_width_px)
        self.grid_height = int(image_height_px / pixel_block_height_px)         
    
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
        objects = {}
        
        # Detect and decode QR codes
        retval, decoded_infos, points, _ = self.qcd.detectAndDecodeMulti(image)
        
        if retval:
            for i, decoded_info in enumerate(decoded_infos):
                if decoded_info:
                    objects[decoded_info] = points[i]
                else:
                    nearest_id = self.get_nearest_object(points[i])
                    if nearest_id:
                        objects[nearest_id] = points[i]
                        print(f"Updating position of {nearest_id}")
        self.update_positions(objects)
    
    def check_qr_code_overlap(self, graph, qr_code_points, proximity_threshold=22):
        transformed_qr_code_points = []
        
        # Apply the affine transformation to the QR code points
        for point in qr_code_points:
            transformed_point = uf.apply_inverse_affine_transform(point, self.matrix)  
            transformed_qr_code_points.append(transformed_point[:2])  # Use x, y coordinates

        # Get the bounding box of the QR code in graph space
        min_x = min([pt[0] for pt in transformed_qr_code_points])
        max_x = max([pt[0] for pt in transformed_qr_code_points])
        min_y = min([pt[1] for pt in transformed_qr_code_points])
        max_y = max([pt[1] for pt in transformed_qr_code_points])
        
        # Check for nodes within the bounding box
        overlapping_nodes = gr.find_nodes_within_bounding_box(graph, min_x, max_x, min_y, max_y, proximity_threshold)    
        return overlapping_nodes
    
    def update_positions(self, objects):
        for obj in objects:
            position = objects[obj]  # (x, y) coordinates

            # Update the tracked objects
            self.tracked_objects[obj] = position

    def get_nearest_object(self, points, threshold=400):
        closest_id = None
        min_distance = float('inf')
        center_position = uf.find_center_of_square(points)

        for obj_id, last_positions in self.tracked_objects.items():
            last_position_center = uf.find_center_of_square(last_positions)
            distance = (uf.euclidean_distance(center_position, last_position_center))
            if distance < min_distance and distance < threshold:
                min_distance = distance
                closest_id = obj_id

        return closest_id

if __name__ == "__main__":
    main()