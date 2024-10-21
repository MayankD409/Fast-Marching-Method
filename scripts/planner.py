import pygame
import numpy as np
import time
import math
import os
import cv2
import random
from collections import deque
from collections import namedtuple

class PNode:
    def __init__(self, x, y, parent, theta, rpm_left, rpm_right, c2c, c2g, cst):
        self.x = x
        self.y = y
        self.parent = parent
        self.theta = theta
        self.rpm_left = rpm_left
        self.rpm_right = rpm_right
        self.c2c = c2c
        self.c2g = c2g
        self.cst = cst
        
    def __lt__(self, other):
        return self.cst < other.cst

def steer(nearest, towards, step_size, robot_radius, clearance):
    theta = math.atan2(towards.y - nearest.y, towards.x - nearest.x)
    distance = min(step_size, euclidean_distance((nearest.x, nearest.y), (towards.x, towards.y)))
    new_x = nearest.x + distance * math.cos(theta)
    new_y = nearest.y + distance * math.sin(theta)
    if is_valid(new_x, new_y, robot_radius, clearance):
        new_node = PNode(new_x, new_y, nearest, theta, 0, 0, nearest.c2c + distance, 0, nearest.c2c + distance)
        return new_node
    return None

def connect_trees(node_from_start, node_from_goal, step_size):
    if node_from_goal and node_from_start:
        if euclidean_distance((node_from_start.x, node_from_start.y), (node_from_goal.x, node_from_goal.y)) <= 5:
            original_parent_goal = node_from_goal.parent
            node_from_goal.parent = node_from_start
            return True, original_parent_goal
    return False, None

def find_nearby_nodes(tree, new_node, radius):
    nearby_nodes = []
    for node in tree:
        if euclidean_distance((node.x, node.y), (new_node.x, new_node.y)) < radius:
            nearby_nodes.append(node)
    return nearby_nodes

def choose_optimal_parent(tree, new_node, radius):
    nearby_nodes = find_nearby_nodes(tree, new_node, radius)
    if not nearby_nodes:
        return None, float('inf')
    optimal_parent = min(nearby_nodes, key=lambda node: node.c2c + euclidean_distance((node.x, node.y), (new_node.x, new_node.y)))
    optimal_cost = optimal_parent.c2c + euclidean_distance((optimal_parent.x, optimal_parent.y), (new_node.x, new_node.y))
    return optimal_parent, optimal_cost

def rewire(tree, new_node, radius):
    for node in find_nearby_nodes(tree, new_node, radius):
        potential_cost = new_node.c2c + euclidean_distance((node.x, node.y), (new_node.x, new_node.y))
        if potential_cost < node.c2c:
            node.parent = new_node
            node.c2c = potential_cost
            node.cst = node.c2c + node.c2g

def euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


def informed_bi_rrt_star(start_position, goal_position, rpm1, rpm2, clearance, robot_radius):
    iterations = 5000
    step_size = 15
    radius = 20  # The radius within which to consider rewiring
    start_tree = [start_position]
    goal_tree = [goal_position]
    c_best = float('inf')
    c_min = euclidean_distance((start_position.x, start_position.y), (goal_position.x, goal_position.y))

    for i in range(iterations):
        if c_best < float('inf'):
            random_point = sample_in_ellipse(start_position, goal_position, c_best, c_min)
        else:
            random_point = PNode(random.uniform(0, 600), random.uniform(0, 200), None, 0, rpm1, rpm2, 0, 0, 0)

        tree = start_tree if i % 2 == 0 else goal_tree
        other_tree = goal_tree if i % 2 == 0 else start_tree

        nearest_node = min(tree, key=lambda node: euclidean_distance((node.x, node.y), (random_point.x, random_point.y)))
        new_node = steer(nearest_node, random_point, step_size, robot_radius, clearance)

        if new_node:
            optimal_parent, optimal_cost = choose_optimal_parent(tree, new_node, radius)
            if optimal_parent:
                new_node.parent = optimal_parent
                new_node.c2c = optimal_cost
                new_node.cst = optimal_cost
            tree.append(new_node)
            rewire(tree, new_node, radius)

            nearest_other = min(other_tree, key=lambda node: euclidean_distance((node.x, node.y), (new_node.x, new_node.y)))

            connected, original_parent_goal = connect_trees(new_node, nearest_other, step_size)
            if connected:
                c_current = nearest_other.c2c + euclidean_distance((nearest_other.x, nearest_other.y), (new_node.x, new_node.y))
                c_best = min(c_best, c_current)
                if i % 2 == 0:
                    return 1, start_tree, goal_tree, new_node, nearest_other, original_parent_goal
                else:
                    return 1, start_tree, goal_tree, nearest_other, new_node, original_parent_goal

    return 0, start_tree, goal_tree, None, None, None

def sample_in_ellipse(start, goal, c_best, c_min):
    # Ellipse focal points
    f1 = np.array([start.x, start.y])
    f2 = np.array([goal.x, goal.y])
    center = (f1 + f2) / 2
    d = np.linalg.norm(f1 - f2)
    
    # Semi-major axis
    a = c_best / 2
    # Semi-minor axis
    b = math.sqrt(a**2 - (d / 2)**2)

    while True:
        # Generate a random point in a unit circle
        angle = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1))
        x_unit = r * math.cos(angle)
        y_unit = r * math.sin(angle)

        # Scale and rotate to the ellipse
        x_ellipse = a * x_unit
        y_ellipse = b * y_unit
        theta = math.atan2(goal.y - start.y, goal.x - start.x)
        rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        sample = np.dot(rotation, np.array([x_ellipse, y_ellipse])) + center
        
        if is_valid(sample[0], sample[1], 0, 0):
            return PNode(sample[0], sample[1], None, 0, 0, 0, 0, 0, 0)

def backtrack(connection_node_start, connection_node_goal, original_parent_goal):  
    # Backtrack from connection node to start node
    current_node = connection_node_start
    path_start = []
    while current_node is not None:
        path_start.append((current_node.x, current_node.y))
        current_node = current_node.parent
        
    
    # Backtrack from connection node to goal node
    current_node = original_parent_goal
    path_goal = []
    while current_node is not None:
        path_goal.append((current_node.x, current_node.y))
        current_node = current_node.parent
    
    # Reverse the paths to get the correct order of nodes
    path_start.reverse()
    # path_goal.reverse()

    # Combine the paths
    x_path = []
    y_path = []

    # Add path from start node to connection node
    for node in path_start:
        x_path.append(node[0])
        y_path.append(node[1])

    # Add path from connection node to goal node
    for node in path_goal[1:]:  # Exclude the connection node
        x_path.append(node[0])
        y_path.append(node[1])
    
    return x_path, y_path

Point = namedtuple("Point", "x y")

def rdp_path(points, epsilon):
    """
    Ramer-Douglas-Peucker (RDP) Algorithm implementation to prune points.

    :param points: List of Point tuples [(x1, y1), (x2, y2), ...].
    :param epsilon: Distance threshold for pruning.
    :return: Pruned list of Point tuples.
    """
    def perpendicular_distance(p, line_start, line_end):
        # Compute the perpendicular distance of point p to the line segment between line_start and line_end.
        if line_start == line_end:
            return math.sqrt((p.x - line_start.x) ** 2 + (p.y - line_start.y) ** 2)
        else:
            norm = ((line_end.x - line_start.x) ** 2 + (line_end.y - line_start.y) ** 2) ** 0.5
            return abs((line_end.x - line_start.x) * (line_start.y - p.y) - (line_start.x - p.x) * (line_end.y - line_start.y)) / norm

    def rdp_recursive(pts, start_idx, end_idx, epsilon, pruned_path):
        # Find the point farthest away from the line segment defined by the start and end index.
        max_distance = 0
        index = -1
        for i in range(start_idx + 1, end_idx):
            distance = perpendicular_distance(pts[i], pts[start_idx], pts[end_idx])
            if distance > max_distance:
                max_distance = distance
                index = i

        # If the maximum distance is greater than epsilon, recursively simplify.
        if max_distance > epsilon:
            rdp_recursive(pts, start_idx, index, epsilon, pruned_path)
            rdp_recursive(pts, index, end_idx, epsilon, pruned_path)
        else:
            pruned_path.append(pts[end_idx])

    if len(points) < 2:
        return points

    pruned_path = [points[0]]
    rdp_recursive(points, 0, len(points) - 1, epsilon, pruned_path)
    return pruned_path

# Replace the prune_path function with the new RDP-based implementation:
def prune_path_rdp(x_path, y_path, epsilon):
    """
    Prune a path using the RDP algorithm.

    :param x_path: List of x coordinates.
    :param y_path: List of y coordinates.
    :param epsilon: Distance threshold for pruning.
    :return: Pruned lists of x and y coordinates.
    """
    points = [Point(x, y) for x, y in zip(x_path, y_path)]
    pruned_points = rdp_path(points, epsilon)
    pruned_x_path, pruned_y_path = zip(*[(point.x, point.y) for point in pruned_points])
    return list(pruned_x_path), list(pruned_y_path)

# Function to create configuration space with obstacles
def is_valid(x, y, robot_radius, clearance):

    # Bottom left corner coordinates of square obstacles
    sqr1_bottom_lft = (60, 40)
    sqr2_bottom_lft = (210, 40)
    sqr3_bottom_lft = (210, 210)
    sqr4_bottom_lft = (360, 210)
    sqr5_bottom_lft = (360, 40)
    sqr6_bottom_lft = (510, 40)
    sqr7_bottom_lft = (510, 210)

    # Creating buffer space for barricades    
    barricade1_buffer_vts = [(150 - (robot_radius + clearance), 300), (152 + (robot_radius + clearance), 300), (152 + (robot_radius + clearance), 120 - (robot_radius + clearance)), (150 - (robot_radius + clearance), 120 - (robot_radius + clearance))]
    barricade2_buffer_vts = [(300 - (robot_radius + clearance), 180 + (robot_radius + clearance)), (302 + (robot_radius + clearance), 180 - (robot_radius + clearance)), (302 + (robot_radius + clearance), 0), (300 - (robot_radius + clearance), 0)]
    barricade3_buffer_vts = [(450 - (robot_radius + clearance), 300), (452 + (robot_radius + clearance), 300), (452 + (robot_radius + clearance), 120 - (robot_radius + clearance)), (450 - (robot_radius + clearance), 120 - (robot_radius + clearance))]

    sqr1_buffer = is_point_inside_square(x, y, sqr1_bottom_lft, 30+2*(robot_radius + clearance))
    sqr2_buffer = is_point_inside_square(x, y, sqr2_bottom_lft, 30+2*(robot_radius + clearance))
    sqr3_buffer = is_point_inside_square(x, y, sqr3_bottom_lft, 30+2*(robot_radius + clearance))
    sqr4_buffer = is_point_inside_square(x, y, sqr4_bottom_lft, 30+2*(robot_radius + clearance))
    sqr5_buffer = is_point_inside_square(x, y, sqr5_bottom_lft, 30+2*(robot_radius + clearance))
    sqr6_buffer = is_point_inside_square(x, y, sqr6_bottom_lft, 30+2*(robot_radius + clearance))
    sqr7_buffer = is_point_inside_square(x, y, sqr7_bottom_lft, 30+2*(robot_radius + clearance))
    barricade1_buffer = is_point_inside_rectangle(x,y, barricade1_buffer_vts)
    barricade2_buffer = is_point_inside_rectangle(x,y, barricade2_buffer_vts)
    barricade3_buffer = is_point_inside_rectangle(x,y, barricade3_buffer_vts)
    
    # Setting buffer space constraints to obtain obstacle space
    if sqr1_buffer or sqr2_buffer or sqr3_buffer or sqr4_buffer or sqr5_buffer or sqr6_buffer or sqr7_buffer or barricade1_buffer or barricade2_buffer or barricade3_buffer:
        return False
    
    # Adding check if obstacle is in walls
    if y >= 300 - (robot_radius + clearance) or y <= (robot_radius + clearance):
        return False

    return True
                                               

def is_point_inside_rectangle(x, y, vertices):
    x_min = min(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    x_max = max(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    y_min = min(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    y_max = max(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    return x_min <= x <= x_max and y_min <= y <= y_max

def is_point_inside_circle(x, y, center_x, center_y, diameter):
    radius = diameter / 2.0
    distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
    return distance <= radius

def is_point_inside_square(x, y, bottom_left, side_length):
    x_min, y_min = bottom_left
    x_max = x_min + side_length
    y_max = y_min + side_length
    return x_min <= x <= x_max and y_min <= y <= y_max