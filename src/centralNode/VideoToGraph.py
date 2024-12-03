import cv2 as cv
import threading as th 
import networkx as nx

from util import UtilityFunctions as uf
from graph import Graph as gr

def main():
    vg = VideoToGraph(75, 150)
    image = cv.imread("img/maze_crop_corners_obstacle_qr_codes.png")
    vg.convert_image_to_graph(image)

class VideoToGraph:

    #initilaize
    def __init__(self, width, length, metric = True):
        #self.cap = self.initialize_camera()
        self.maze_height = width if metric else width * 2.54
        self.maze_length = length if metric else length * 2.54
        self.grid_height = 0
        self.grid_width = 0

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
        corners = uf.find_corners(image)
        self.set_dimensions(corners)
        graph = nx.grid_2d_graph(self.grid_width, self.grid_height)
        gr.add_diagonal_edges(self.grid_width, self.grid_height, graph)
        matrix = uf.compute_affine_transformation(corners, self.grid_width, self.grid_height)
        gr.set_node_positions(graph, matrix)
        gr.adjust_graph_weights(graph)        

        self.detect_static_obstacles(image, graph)
        gr.update_graph_weights_based_on_obstacles(graph)

        overlay_image = self.draw_overlay(image, graph)
        uf.show_image(overlay_image)


        start = (0, 0)
        target = (self.grid_width - 5, self.grid_height - 15)

        path = gr.safe_astar_path(graph, start, target, heuristic=gr.heuristic)
        if path:
            overlay_image = gr.draw_transformed_path(overlay_image, graph, path)
            overlay_image = gr.draw_path_weights(overlay_image, graph, path)
        objects = self.detect_objects(image)

        for key in objects.keys():
            pts = objects[key].astype(int)
            cv.polylines(overlay_image, [pts], isClosed=True, color=uf.GREEN, thickness=2)

        uf.show_image(overlay_image)
    
    def draw_overlay(self, image, graph):
        overlay_image = image.copy()
        overlay_image = gr.draw_nodes_overlay(graph, overlay_image)
        overlay_image = gr.draw_edges_overlay(graph, overlay_image)
        return overlay_image

    def set_dimensions(self, corners, block_size_cm=4):
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
            uf.show_image(overlay_image)

    def detect_objects(self, image):
        # Initialize the QR code detector
        qcd = cv.QRCodeDetector()
        objects = {}
        
        # Detect and decode QR codes
        print("Detecting QR codes...")
        retval, decoded_infos, points, _ = qcd.detectAndDecodeMulti(image)
        
        if retval:
            print("QR codes detected.")
            for i, decoded_info in enumerate(decoded_infos):
                obj_type, obj_number = decoded_info.split(" ")
                decoded_info = obj_type+"_"+obj_number
                objects[decoded_info] = points[i]
                print(f"QR Code {i+1}: {decoded_info}")
        return objects

if __name__ == "__main__":
    main()