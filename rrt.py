import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import random

class Node():
    def __init__(self, x, y, parent = None):
        self.x = x
        self.y = y
        self.parent = parent
        self.children = []
        self.cost = 0

    def distance_to(self, node):
        """Calculate the Euclidean distance to a specified node"""
        return math.sqrt((self.x - node.x) **2 + (self.y - node.y) **2)
    
    def calculate_cost(self, parent):
        if self.parent:
            self.cost = self.parent.cost + self.distance_to(parent)

# Circles will serve as the obstacles
class Circle():
    def __init__(self, x_center, y_center, radius):
        self.x_center = x_center
        self.y_center = y_center
        self.radius = radius

    def is_point_inside(self, x, y):
        distance_to_center = math.sqrt((self.x_center - x) **2 + (self.y_center - y) **2)
        return distance_to_center <= self.radius
        
class RRT_star():
    def __init__(self, start, goal, map_size):
        self.start = start
        self.goal = goal
        self.map_size = map_size
        self.x_bound = [0, map_size[0]]
        self.y_bound = [0, map_size[1]]
        
    def generate_point(self):
        [newX, newY] = [random.uniform(self.x_bound), random.uniform(self.y_bound)]
        # if new coordinate is inside an obstacle radius, retry
        return Node(newX, newY)
