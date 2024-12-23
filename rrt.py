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
        self.parent = None
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
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_size = 1
        self.tree = [self.start]
        self.map_size = map_size
        self.x_bound = [0, map_size[0]]
        self.y_bound = [0, map_size[1]]
        self.obstacles = []

    def distance_to(self, x , y, targetX, targetY):
        return math.sqrt((x - targetX) ** 2 + (y - targetY)**2)

    def overlap_check(self, obstacles, x, y, radius):
        # Check if new point overlaps with start or goal node
        distance_to_start = self.distance_to(x, y, self.start.x, self.start.y)
        distance_to_goal = self.distance_to(x, y, self.goal.x, self.goal.y)
       
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
        
        return Node(newX, newY)
    
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
        new_node.parent = nearest_node  
        new_node.cost = nearest_node.cost + self.distance_to(nearest_node.x, nearest_node.y, new_node.x, new_node.y)
        self.tree.append(new_node)

    def steer(self, from_node, to_point):
        """Steering the tree towards a certain node"""
        dx = to_point.x - from_node.x
        dy = to_point.y - from_node.y

        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.step_size:
            new_node = Node(to_point.x, to_point.y)
            if self.validate_node(new_node) is not None:
                return new_node
            return None
    
        dx_normalised = dx / distance
        dy_normalised = dy / distance

        new_x = from_node.x + (dx_normalised * self.step_size)
        new_y = from_node.y + (dy_normalised * self.step_size)

        new_node = Node(new_x, new_y)

        return new_node if self.validate_node(new_node) else None

        
    def rrt(self):
        current_coordinate = self.start

        while current_coordinate != self.goal:
            random_point = self.generate_point()
            nearest_node = self.find_nearest_node(random_point)
            new_node = self.steer(nearest_node, random_point)

            if new_node and self.validate_node(new_node):
                self.add_node(new_node)
                current_coordinate = new_node

            if new_node and self.distance_to(current_coordinate.x, current_coordinate.y,  self.goal.x, self.goal.y) < self.step_size:
                print('goal reached')
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.distance_to(new_node.x, new_node.y, self.goal.x, self.goal.y)      
                self.tree.append(self.goal)          
                break
    
    def plot(self):
        plt.figure()
        plt.xlim(self.x_bound)
        plt.ylim(self.y_bound)

        # Plot start and goal
        plt.scatter(self.start.x, self.start.y, c='green', s=100, label='Start')
        plt.scatter(self.goal.x, self.goal.y, c='red', s=100, label='Goal')

        # Plot obstacles
        for obs in self.obstacles:
            circle = patches.Circle(
                (obs.x_center, obs.y_center),
                radius=obs.radius,
                color='orange'
            )
            plt.gca().add_patch(circle)

        # Plot tree edges
        for node in self.tree:
            for child in node.children:
                plt.plot([node.x, child.x], [node.y, child.y], c='blue')

        # Highlight the lowest-cost path to the goal
        if self.goal.parent:
            current = self.goal
            while current.parent:
                plt.plot([current.x, current.parent.x],
                        [current.y, current.parent.y],
                        c='green', linewidth=2)
                current = current.parent

        plt.grid()
        plt.legend()
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

# Initialize RRT*
start = [1, 9]
goal = [10,10]
map_size = [10, 10]

rrt = RRT_star(start, goal, map_size)

# Create obstacles
rrt.create_obstacles(Circle, 4, [0.5, 1])

# Run the RRT* algorithm
rrt.rrt()

# Plot the resulting tree and obstacles
rrt.plot()

    