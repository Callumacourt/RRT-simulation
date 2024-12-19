import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
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
        self.obstacles = []

    def distance_to(self, x , y, targetX, targetY):
        return math.sqrt((x - targetX) ** 2 + (y - targetY)**2)

    def overlap_check(self, obstacles, x, y, radius):
        
        # Check if new point overlaps with start or goal node
        distance_to_start = self.distance_to(x, y, self.start[0], self.start[1])
        distance_to_goal = self.distance_to(x, y, self.goal[0], self.goal[1])
       
        if distance_to_start < radius or distance_to_goal < radius:
            return True
        
        # Check if the new point overlaps with any obstacles
        for obs in obstacles:
            distance_to_obs = self.distance_to(x, obs.x_center, y, obs.y_center)

            if (radius + obs.radius) > distance_to_obs:
                return True
        return False # no overlap

    def create_obstacles(self, obstacle_class, amount, radius_range):

        obstacles = []

        for _ in range(amount):
            x_center = random.uniform(0, self.x_bound[1])
            y_center = random.uniform(0, self.y_bound[1])
            radius = random.uniform(radius_range[0], radius_range[1])

            # if new obstacle overlaps with current, then start again
            if self.overlap_check(obstacles, x_center, y_center, radius): 
                continue
            
            obstacle = obstacle_class(x_center, y_center, radius)
            obstacles.append(obstacle)

        self.obstacles = obstacles

    def generate_point(self):
        newX = random.uniform(self.x_bound[0], self.x_bound[1])
        newY = random.uniform(self.y_bound[0], self.y_bound[1])

        # Recursively try to generate a new point that isn't inside an obstacle
        if self.overlap_check(self.obstacles, newX, newY, 0):
            return self.generate_point()
        
        return Node(newX, newY)
    
    def plot(self):
        plt.figure()
        plt.xlim(self.x_bound)
        plt.ylim(self.y_bound)

        plt.scatter(self.start[0], self.start[1], c='green', s= 100, label= 'Start')
        plt.scatter(self.goal[0], self.goal[1], c='red', s= 100, label = 'Goal' )

        for obs in self.obstacles:
            circle = patches.Circle(
                (obs.x_center, obs.y_center),
                radius = obs.radius,
                color = 'orange'
            )
            plt.gca().add_patch(circle)

        plt.grid()
        plt.legend()
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

start = [1,1]
goal = [9,9]
map_size = [10,10]
rrt = RRT_star(start, goal, map_size)
rrt.create_obstacles(Circle, 4, [0.5, 1])
rrt.plot()
    
    