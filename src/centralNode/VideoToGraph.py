import numpy as np
import cv2 as cv
import threading as th 
import networkx as nx
from networkx.algorithms.shortest_paths.astar import astar_path

import matplotlib.pyplot as plt

def main():
    vg = VideoToGraph(75, 150)
    image = cv.imread("img/maze_crop_modified.png")
    vg.convert_image_to_graph(image)

class VideoToGraph:

    #initilaize
    def __init__(self, width, length, metric = True):
        #self.cap = self.initialize_camera()
        self.maze_height = width if metric else width * 2.54
        self.maze_length = length if metric else length * 2.54
        self.pixel_block_height_px = 0
        self.pixel_block_width_px = 0

# Video input

# create graph

# update graph

# Detecting the objects

