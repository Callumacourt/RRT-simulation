import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import math
import random

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parents = []
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
    
    def intersect_check(self, x1, y1, x2, y2):
        """Check for obstacles between two coordinates"""
        vector_v = np.array([x2, y2]) - np.array([x1, y1]) # Line segment
        vector_w = np.array([self.x_center, self.y_center]) - np.array([x1, y1]) # Start point to circle center
        # Projection result
        t = max(0, min(1, (np.dot(vector_v, vector_w) / np.dot(vector_v, vector_v))))
        
        # Determine closest point in segment to circle
        if t <= 0:
            closest_point = [x1, y1]
        elif t >= 1:
            closest_point = [x2, y2]
        elif 0 < t < 1:
            closest_point = np.array([x1, y1]) + t * vector_v
        
        # Calculate distance to circle
        distance = math.sqrt((self.x_center - closest_point[0]) **2 + (self.y_center -  closest_point[1]) **2)

        # Check if closest point is inside circle
        if distance > self.radius:
            return False
        return True


class RRT_star():
    def __init__(self, start, goal, map_size):
        self.start = start
        self.goal = goal
        self.step_size = 1
        self.tree = []
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
        """Generate a coordinate point in the graph and return as a node"""
        newX = random.uniform(self.x_bound[0], self.x_bound[1])
        newY = random.uniform(self.y_bound[0], self.y_bound[1])

        # Recursively try to generate a new point that isn't inside an obstacle
        if self.overlap_check(self.obstacles, newX, newY, 0):
            return self.generate_point()
        
        self.tree.append(Node(newX, newY))
    
    def find_nearest_node(self, new_node):
        """Find the nearest node to a specified node"""
        if len(self.tree) == 0:
            return None
        
        nearest_node = self.tree[0]
        smallest_distance = self.distance_to(nearest_node.x, nearest_node.y, new_node.x, new_node.y)

        for node in self.tree:
            dist = self.distance_to(node.x, node.y, new_node.x, new_node.y)
            if dist < smallest_distance:
                nearest_node = node
                smallest_distance = dist
        return nearest_node
    
    def validate_node(self, new_node):
        # Check if node is inside an obstacle
        for obs in self.obstacles:
            if obs.is_point_inside(new_node.x, new_node.y):
                return None

        # Check node isn't outside map boundaries
        if new_node.x > self.map_size[0] or new_node.y > self.map_size[1]:
            return None

        # Find nearest node in tree    
        nearest_node = self.find_nearest_node(new_node)

        for obs in self.obstacles:
            if obs.intersect_check(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
                return None
            
        return nearest_node
        

    def add_node(self, new_node):
        """Appending a node to the tree"""
        nearest_node = self.validate_node(new_node)

        if nearest_node is None: # Node is invalud
            return
        
        nearest_node.children.append(new_node)
        new_node.parents.append(nearest_node)
        new_node.cost = nearest_node.cost + self.distance_to(nearest_node.x, nearest_node.y, new_node.x, new_node.y)

    def steer(self, from_node, to_point):
        """Steering the tree towards a certain node"""
        dx = to_point.x - from_node.x
        dy = to_point.y - from_node.y

        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.step_size:
            new_node = Node(to_point.x, to_point.y)
            if self.validate_node(new_node) is not None:
                return new_node
            return
    
        dx_normalised = dx / distance
        dy_normalised = dy / distance

        new_x = from_node.x + (dx_normalised * self.step_size)
        new_y = from_node.y + (dy_normalised * self.step_size)

        new_node = Node(new_x, new_y)

        if self.validate_node(new_node is not None):
            return new_node
        
        
    
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
    
    