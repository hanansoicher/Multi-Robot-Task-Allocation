import math
import networkx as nx
import numpy as np
from util import UtilityFunctions as uf
from typing import Optional
import cv2 as cv

class Graph(nx.Graph):

    NEAR_OBSTACLE = "is_near_obstacle"
    PIXEL_POS = "pixel_pos"
    GRID_POS = "grid_pos"
    INF = float('inf')
    EDGE_WEIGHT = "distance_in_cm"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def set_node_positions(graph, matrix):
        pos = {node: node for node in graph.nodes()}
        nx.set_node_attributes(graph, pos, Graph.GRID_POS)

        pixel_pos = {node: uf.apply_affine_transform(node, matrix) for node in graph.nodes()}
        nx.set_node_attributes(graph, pixel_pos, Graph.PIXEL_POS)

    def draw_nodes_overlay(graph, overlay_image):
        for node in graph.nodes():
            transformed_node = graph.nodes[node][Graph.PIXEL_POS]
            near_obstacle = Graph.is_node_near_obstacle(graph, node)
            color = uf.RED if near_obstacle else uf.GREEN
            cv.circle(overlay_image, (int(transformed_node[0]), int(transformed_node[1])), radius=5, color=color, thickness=-1)
        return overlay_image
    
    def draw_edges_overlay(graph, overlay_image):
        for edge in graph.edges():
            node1_x, node1_y, _ = graph.nodes[edge[0]][Graph.PIXEL_POS]
            node2_x, node2_y, _ = graph.nodes[edge[1]][Graph.PIXEL_POS]
            color = uf.RED if Graph.is_node_near_obstacle(graph, edge[0], edge[1]) else uf.BLUE
            cv.line(overlay_image, 
                    (int(node1_x), int(node1_y)), 
                    (int(node2_x), int(node2_y)), 
                    color=color, thickness=1)
        return overlay_image

    def is_node_near_obstacle(graph, node_a, node_b: Optional[any] = None):
        if node_b is None:
            return graph.nodes[node_a].get(Graph.NEAR_OBSTACLE)
        return graph.nodes[node_a].get(Graph.NEAR_OBSTACLE) or graph.nodes[node_b].get(Graph.NEAR_OBSTACLE)

    def add_diagonal_edges(grid_width, grid_height, graph):
        for x, y in graph.nodes():
            # Define the diagonal neighbors
            diagonal_neighbors = [
                (x - 1, y - 1),  # Northwest
                (x - 1, y + 1),  # Northeast
                (x + 1, y - 1),  # Southwest
                (x + 1, y + 1)   # Southeast
            ]

            # Add edges to diagonal neighbors if they are within bounds
            for node_x, node_y in diagonal_neighbors:
                if 0 <= node_x < grid_width and 0 <= node_y < grid_height:
                    graph.add_edge((x, y), (node_x, node_y))
    
    def update_graph_weights_based_on_obstacles(graph):
        for node in graph.nodes:
            if graph.nodes[node].get(Graph.NEAR_OBSTACLE, True):
                for neighbor in graph.neighbors(node):
                    graph[node][neighbor][Graph.EDGE_WEIGHT] = Graph.INF
                    graph[neighbor][node][Graph.EDGE_WEIGHT] = Graph.INF 

    DIAGONAL = "diagonal"
    HORIZONTAL = "horizontal"
    VERTICAL = "vertical"

    def adjust_graph_weights(graph, conversion):
        for edge in graph.edges():
            node_a, node_b = edge
            if Graph.is_node_near_obstacle(graph, node_a, node_b):
                graph.edges[node_a, node_b][Graph.EDGE_WEIGHT] = Graph.INF
                graph.edges[node_b, node_a][Graph.EDGE_WEIGHT] = Graph.INF
            else:
                distance_in_cm = Graph.adjust_distance_based_on_correction(graph, node_a, node_b, conversion)
                graph.edges[node_a, node_b][Graph.EDGE_WEIGHT] = distance_in_cm
                graph.edges[node_b, node_a][Graph.EDGE_WEIGHT] = distance_in_cm
        return graph
    
    @staticmethod
    def adjust_distance_based_on_correction(graph, node_a, node_b, conversion):
            node_a_x, node_a_y, _ = graph.nodes[node_a][Graph.PIXEL_POS]
            node_b_x, node_b_y, _ = graph.nodes[node_b][Graph.PIXEL_POS]
            pixel_distance = uf.euclidean_distance((node_a_x,node_a_y), (node_b_x, node_b_y))

            # adjust the weight based on direction
            distance_in_cm = float('inf')
            direction = Graph.direction(node_a, node_b)
            if direction == Graph.DIAGONAL:
                distance_in_cm = pixel_distance * conversion[2] 
            
            elif direction == Graph.HORIZONTAL:
                distance_in_cm = pixel_distance * conversion[0]

            elif direction == Graph.VERTICAL: 
                distance_in_cm = pixel_distance * conversion[1]
            return distance_in_cm

    @staticmethod
    def direction(node_a, node_b):
        x1, y1 = node_a
        x2, y2 = node_b
        if (x1 < x2 or x2 > x1) and (y1 < y2 or y2 > y1):
            return Graph.DIAGONAL
        if (x1 < x2 or x2 < x1) and y1 == y2:
            return Graph.HORIZONTAL
        if (x1 == x2) and (y1 < y2 or y2 < y1):
            return Graph.VERTICAL
        else:
            return None
        
    def adjust_distance_based_on_correction_pixel(graph, pixel_pos_a, pixel_pos_b, conversion):
        # Extract real-world coordinate
        pos_a_x, pos_a_y = pixel_pos_a
        pos_b_x, pos_b_y = pixel_pos_b
        
        # Calculate pixel distance
        pixel_distance = uf.euclidean_distance((pos_a_x, pos_a_y), (pos_b_x, pos_b_y))
        
        # Determine the direction based on threshold
        direction = Graph.direction_pixel((pos_a_x, pos_a_y), (pos_b_x, pos_b_y), pixel_distance/3)
        
        # Adjust the weight based on the determined direction
        if direction == Graph.DIAGONAL:
            return pixel_distance * conversion[2]
        elif direction == Graph.HORIZONTAL:
            return pixel_distance * conversion[0]
        elif direction == Graph.VERTICAL:
            return pixel_distance * conversion[1]
        
        # Default case if no valid direction
        return float('inf')


    @staticmethod
    def direction_pixel(node_a_coords, node_b_coords, threshold):
        x1, y1 = node_a_coords
        x2, y2 = node_b_coords
        
        # Calculate the differences in x and y directions
        delta_x = abs(x2 - x1)
        delta_y = abs(y2 - y1)
        
        # Determine the movement direction based on the threshold
        if delta_x > threshold and delta_y > threshold:
            return Graph.DIAGONAL
        elif delta_x > threshold and delta_y <= threshold:
            return Graph.HORIZONTAL
        elif delta_y > threshold and delta_x <= threshold:
            return Graph.VERTICAL
        else:
            # If the movement is too small in all directions
            return None


    def update_graph_based_on_obstacle(graph, contour, proximity_threshold):
        for node in graph.nodes:
            node_x, node_y,_ = graph.nodes[node][Graph.PIXEL_POS]
            distance = abs(cv.pointPolygonTest(contour, (node_x,node_y), True)) 
            if distance <= proximity_threshold:
                graph.nodes[node][Graph.NEAR_OBSTACLE] = True
            else:
                graph.nodes[node].setdefault(Graph.NEAR_OBSTACLE, False)

    def update_graph_based_on_qr_code(graph, overlapping_nodes, previous_overlapping_nodes):
        subgraph = graph.subgraph(overlapping_nodes)
        for node in subgraph.nodes:
            graph.nodes[node][Graph.NEAR_OBSTACLE] = True

        non_overlapping_nodes = (previous_overlapping_nodes - overlapping_nodes)
        subgraph = graph.subgraph(non_overlapping_nodes)
        for node in subgraph.nodes:
            graph.nodes[node][Graph.NEAR_OBSTACLE] = False

    def find_nodes_within_bounding_box(graph, min_x, max_x, min_y, max_y, proximity_threshold):
        overlapping_nodes = set()
        for node in graph.nodes():
            node_pos = graph.nodes[node][Graph.PIXEL_POS]
            node_x, node_y = node_pos[0], node_pos[1]
            
            # Check if the node is within the bounding box of the QR code
            if min_x - proximity_threshold <= node_x <= max_x + proximity_threshold and \
            min_y - proximity_threshold <= node_y <= max_y + proximity_threshold:
                overlapping_nodes.add(node)
        return overlapping_nodes
    
    @staticmethod
    def safe_astar_path(graph, start_node, goal_node, heuristic):
        if not nx.has_path(graph, start_node, goal_node):
            return None

        path = nx.astar_path(graph, source=start_node, target=goal_node, 
                            weight= lambda u, v, d: Graph.INF if d[Graph.EDGE_WEIGHT] == Graph.INF else d[Graph.EDGE_WEIGHT],
                            heuristic=heuristic)
        # lambda u, v, d: Graph.INF if d[Graph.EDGE_WEIGHT] == Graph.INF else d[Graph.EDGE_WEIGHT]
        # Check if any edge in the path has infinite weight
        for u, v in zip(path[:-1], path[1:]):
            if graph.edges[u, v].get(Graph.EDGE_WEIGHT, None) == Graph.INF:
                print(f"No path exists: Infinite weight edge encountered between {u} and {v}.")
                return None
        return path
    

    def a_star_from_pixel_pos(graph, pixel_pos, goal):
        nearest_node = Graph.find_nearest_node(graph, pixel_pos)
        path = Graph.safe_astar_path(graph, nearest_node, goal, Graph.heuristic)
        return path

    # Feel free to improve try to use inverse_transformation instead
    @staticmethod
    def find_nearest_node(graph, query_point):
        # Extract node positions
        positions = nx.get_node_attributes(graph, Graph.PIXEL_POS)
        query_point = query_point + (np.float32(1.0),)
        distances = {
                node: np.linalg.norm(np.array(pos) - np.array(query_point))
                for node, pos in positions.items()
            }
        
        # Find the node with the minimum distance
        nearest_node = min(distances, key=distances.get)
        return nearest_node

    @staticmethod
    def print_path_weights(graph, path):
        total_weight = []
        pixel_distances = []
        for i in range(len(path) - 1):
            node1 = path[i]
            node2 = path[i + 1]
            weight = graph[node1][node2].get(Graph.EDGE_WEIGHT, None) 
            pixel_dist = uf.euclidean_distance(graph.nodes[node1][Graph.PIXEL_POS], graph.nodes[node2][Graph.PIXEL_POS])
            total_weight.append(weight)
            pixel_distances.append(pixel_dist)
        total = uf.kahan_sum(total_weight)
        total_pixel_dist = uf.kahan_sum(pixel_distances)
        # print(f"Total path weight: {total}, Total pixel distance: {total_pixel_dist}")

        return total

    


    @staticmethod
    def heuristic(node, goal):
        return uf.euclidean_distance(node, goal)       
    
    def draw_path_weights(image, graph, path):
        overlay_image = image.copy()
        for i in range(len(path) - 1):
            node_a = path[i]
            node_b = path[i + 1]
            
            pos_a = graph.nodes[node_a][Graph.PIXEL_POS]
            edge_weight = graph[node_a][node_b].get(Graph.EDGE_WEIGHT, None)
            cv.putText(overlay_image, str(edge_weight), (int(pos_a[0]), int(pos_a[1])), 
                cv.FONT_HERSHEY_SIMPLEX, 0.8, uf.GREEN, 2)
            
        return overlay_image
    
    def draw_transformed_path(image, graph, path):
        for i in range(len(path) - 1):
            node_a = path[i]
            node_b = path[i + 1]
            
            pos_a = graph.nodes[node_a][Graph.PIXEL_POS]
            pos_b = graph.nodes[node_b][Graph.PIXEL_POS]

            cv.line(image, (int(pos_a[0]), int(pos_a[1])), 
                    (int(pos_b[0]), int(pos_b[1])), uf.GREEN, 2)  

            cv.circle(image, (int(pos_a[0]), int(pos_a[1])), 
                      5, uf.YELLOW, -1)  # Yellow circle

        return image