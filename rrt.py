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
        