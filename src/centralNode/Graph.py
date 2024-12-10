import networkx as nx
import numpy as np
from util import UtilityFunctions as uf
from typing import Optional
import cv2 as cv

class Graph(nx.Graph):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def set_node_positions(graph, matrix):
        pos = {node: node for node in graph.nodes()}
        nx.set_node_attributes(graph, pos, "pos")

        real_pos = {node: uf.apply_affine_transform(node, matrix) for node in graph.nodes()}
        nx.set_node_attributes(graph, real_pos, "real_pos")

    def draw_nodes_overlay(graph, overlay_image):
        for node in graph.nodes():
            transformed_node = graph.nodes[node]["real_pos"]
            near_obstacle = Graph.is_node_near_obstacle(graph, node)
            color = uf.RED if near_obstacle else uf.GREEN
            cv.circle(overlay_image, (int(transformed_node[0]), int(transformed_node[1])), radius=5, color=color, thickness=-1)
        return overlay_image
    
    def draw_edges_overlay(graph, overlay_image):
        for edge in graph.edges():
            node1_x, node1_y, _ = graph.nodes[edge[0]]["real_pos"]
            node2_x, node2_y, _ = graph.nodes[edge[1]]["real_pos"]
            graph[edge[0]][edge[1]]['weight'] = uf.euclidean_distance((node1_x, node1_y), (node2_x, node2_y))
            color = uf.RED if Graph.is_node_near_obstacle(graph, edge[0], edge[1]) else uf.BLUE
            cv.line(overlay_image, 
                    (int(node1_x), int(node1_y)), 
                    (int(node2_x), int(node2_y)), 
                    color=color, thickness=1)
        return overlay_image

    def is_node_near_obstacle(graph, node_a, node_b: Optional[any] = None):
        if node_b is None:
            return graph.nodes[node_a].get("is_near_obstacle")
        return graph.nodes[node_a].get("is_near_obstacle") or graph.nodes[node_b].get("is_near_obstacle")

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
            for nx_, ny_ in diagonal_neighbors:
                if 0 <= nx_ < grid_width and 0 <= ny_ < grid_height:
                    graph.add_edge((x, y), (nx_, ny_))
    
    def update_graph_weights_based_on_obstacles(graph):
        for node in graph.nodes:
            if graph.nodes[node].get("is_near_obstacle", False):
                for neighbor in graph.neighbors(node):
                    graph[node][neighbor]['weight'] = float('inf')
                    graph[neighbor][node]['weight'] = float('inf') 

    def adjust_graph_weights(graph):
        for edge in graph.edges():
            node_a, node_b = edge
            if graph.nodes[node_a].get("is_near_obstacle", False) or graph.nodes[node_b].get("is_near_obstacle", False):
                graph.edges[node_a, node_b]['weight'] = float('inf')
                graph.edges[node_b, node_a]['weight'] = float('inf')
            else:
                node_a_x, node_a_y, _ = graph.nodes[node_a]["real_pos"]
                node_b_x, node_b_y, _ = graph.nodes[node_b]["real_pos"]
                distance = uf.euclidean_distance((node_a_x, node_a_y), (node_b_x, node_b_y))
                graph.edges[node_a, node_b]['weight'] = distance
                graph.edges[node_b, node_a]['weight'] = distance

    def update_graph_based_on_obstacle(graph, contour, proximity_threshold):
        for node in graph.nodes:
            node_x, node_y,_ = graph.nodes[node]["real_pos"]
            distance = abs(cv.pointPolygonTest(contour, (node_x,node_y), True)) 
            if distance <= proximity_threshold:
                graph.nodes[node]["is_near_obstacle"] = True
            else:
                graph.nodes[node].setdefault("is_near_obstacle", False)

    def update_graph_based_on_qr_code(graph, overlapping_nodes):
        for node in overlapping_nodes:
            graph.nodes[node]["is_near_obstacle"] = True

        Graph.update_graph_weights_based_on_obstacles(graph)

    def find_nodes_within_bounding_box(graph, min_x, max_x, min_y, max_y, proximity_threshold):
        overlapping_nodes = []
        for node in graph.nodes():
            node_pos = graph.nodes[node]["real_pos"]
            node_x, node_y = node_pos[0], node_pos[1]
            
            # Check if the node is within the bounding box of the QR code
            if min_x - proximity_threshold <= node_x <= max_x + proximity_threshold and \
            min_y - proximity_threshold <= node_y <= max_y + proximity_threshold:
                overlapping_nodes.append(node)
        return overlapping_nodes
    
    @staticmethod
    def safe_astar_path(graph, start_node, goal_node, heuristic):
        if not nx.has_path(graph, start_node, goal_node):
            return None

        path = nx.astar_path(graph, source=start_node, target=goal_node, weight= lambda u, v, d: None if d['weight'] == float("inf") else d['weight'], heuristic=heuristic)
        
        # Check if any edge in the path has infinite weight
        for u, v in zip(path[:-1], path[1:]):
            if graph.edges[u, v].get('weight', None) == float('inf'):
                print(f"No path exists: Infinite weight edge encountered between {u} and {v}.")
                return None
        return path
    

    def a_star_from_pixel_pos(graph, pixel_pos, goal):
        nearest_node = Graph.find_nearest_node(graph, pixel_pos)
        path = Graph.safe_astar_path(graph, nearest_node, goal, Graph.heuristic)
        return path

    # Feel free to improve
    @staticmethod
    def find_nearest_node(graph, query_point):
        # Extract node positions
        positions = nx.get_node_attributes(graph, "real_pos")
        query_point = query_point + (np.float32(1.0),)
        distances = {
                node: np.linalg.norm(np.array(pos) - np.array(query_point))
                for node, pos in positions.items()
            }
        
        # Find the node with the minimum distance
        nearest_node = min(distances, key=distances.get)
        return nearest_node

    def print_path_weights(graph, path):
        total_weight = 0
        for i in range(len(path) - 1):
            node1 = path[i]
            node2 = path[i + 1]
            weight = graph[node1][node2].get('weight', None)  # Access the weight of the edge
            total_weight += weight if weight is not None else 0
            print(f"Edge {node1} -> {node2}: weight = {weight}")
        print(f"Total path weight: {total_weight}")

    @staticmethod
    def heuristic(node, goal):
        return uf.euclidean_distance(node, goal)       
    
    def draw_path_weights(image, graph, path):
        overlay_image = image.copy()
        for i in range(len(path) - 1):
            node_a = path[i]
            node_b = path[i + 1]
            
            pos_a = graph.nodes[node_a]['real_pos']
            pos_b = graph.nodes[node_b]['real_pos']
            recalc = uf.euclidean_distance(pos_a, pos_b)
            if recalc != graph[node_a][node_b].get('weight', None):
                print(f"Weight mismatch between nodes {node_a} and {node_b}. Recalculated: {recalc}, Stored: {graph[node_a][node_b].get('weight', None)}")
            edge_weight = graph[node_a][node_b].get('weight', None)
            cv.putText(overlay_image, str(edge_weight), (int(pos_a[0]), int(pos_a[1])), 
                cv.FONT_HERSHEY_SIMPLEX, 0.8, uf.GREEN, 2)
            
        return overlay_image
    
    def draw_transformed_path(image, graph, path):
        overlay_image = image.copy()
        for i in range(len(path) - 1):
            node_a = path[i]
            node_b = path[i + 1]
            
            pos_a = graph.nodes[node_a]['real_pos']
            pos_b = graph.nodes[node_b]['real_pos']

            cv.line(overlay_image, (int(pos_a[0]), int(pos_a[1])), 
                    (int(pos_b[0]), int(pos_b[1])), uf.BLUE, 2)  

            cv.circle(overlay_image, (int(pos_a[0]), int(pos_a[1])), 
                      5, uf.YELLOW, -1)  # Yellow circle

        return overlay_image