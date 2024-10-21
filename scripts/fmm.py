import numpy as np
import math
import heapq

class GridCell:
    def __init__(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost = cost
    
    def __lt__(self, other):
        return self.cost < other.cost

def get_neighbors(x, y, max_x, max_y):
    neighbors = []
    movements = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    for move in movements:
        nx, ny = x + move[0], y + move[1]
        if 0 <= nx < max_x and 0 <= ny < max_y:
            neighbors.append((nx, ny))
    
    return neighbors

def fast_marching_method(start, goal, clearance, robot_radius, grid_width, grid_height):
    """Returns a path using the Fast Marching Method, with bounds-checking included"""
    distance_map = np.full((grid_width, grid_height), np.inf)
    parent_map = np.full((grid_width, grid_height, 2), -1, dtype=int)
    visited = np.zeros((grid_width, grid_height), dtype=bool)

    # Make sure the start and goal positions are within bounds
    if not (0 <= start[0] < grid_width and 0 <= start[1] < grid_height):
        raise ValueError("Start position is out of grid bounds")
    if not (0 <= goal[0] < grid_width and 0 <= goal[1] < grid_height):
        raise ValueError("Goal position is out of grid bounds")

    priority_queue = []
    heapq.heappush(priority_queue, GridCell(start[0], start[1], 0))
    distance_map[start[0], start[1]] = 0

    while priority_queue:
        current = heapq.heappop(priority_queue)
        cx, cy = current.x, current.y

        if visited[cx, cy]:
            continue
        visited[cx, cy] = True

        if (cx, cy) == (goal[0], goal[1]):
            break

        for nx, ny in get_neighbors(cx, cy, grid_width, grid_height):
            if visited[nx, ny]:
                continue

            if not is_valid(nx, ny, robot_radius, clearance):
                continue

            new_cost = current.cost + np.linalg.norm([nx - cx, ny - cy])
            if new_cost < distance_map[nx, ny]:
                distance_map[nx, ny] = new_cost
                parent_map[nx, ny] = [cx, cy]
                heapq.heappush(priority_queue, GridCell(nx, ny, new_cost))

    # Backtrack with bounds checking
    path = []
    x, y = goal
    while 0 <= x < grid_width and 0 <= y < grid_height and parent_map[x, y][0] != -1:
        path.append((x, y))
        x, y = parent_map[x, y]

    path.append((start[0], start[1]))
    path.reverse()
    return np.array([p[0] for p in path]), np.array([p[1] for p in path])

# Ray-casting function to check for a clear path between two points
def ray_cast(x1, y1, x2, y2, robot_radius, clearance):
    """Returns True if the line between (x1, y1) and (x2, y2) is clear of obstacles"""
    points = np.linspace(0, 1, num=max(abs(x2 - x1), abs(y2 - y1)))
    for t in points:
        x = int(x1 + t * (x2 - x1))
        y = int(y1 + t * (y2 - y1))
        if not is_valid(x, y, robot_radius, clearance):
            return False
    return True

# Path pruning function
def prune_path(x_path, y_path, robot_radius, clearance):
    pruned_x = [x_path[0]]
    pruned_y = [y_path[0]]
    
    current_index = 0
    while current_index < len(x_path) - 1:
        next_index = current_index + 1
        # Move to the farthest index with a clear path
        while next_index < len(x_path) and ray_cast(
            x_path[current_index], y_path[current_index], x_path[next_index], y_path[next_index], robot_radius, clearance):
            next_index += 1

        pruned_x.append(x_path[next_index - 1])
        pruned_y.append(y_path[next_index - 1])
        current_index = next_index - 1

    return pruned_x, pruned_y


# Function to create configuration space with obstacles
def is_valid(x, y, robot_radius, clearance):

    # Bottom left corner coordinates of square obstacles
    sqr1_bottom_lft = (6.0 - (robot_radius + clearance), 4.0 - (robot_radius + clearance))
    sqr2_bottom_lft = (21.0 - (robot_radius + clearance), 4.0 - (robot_radius + clearance))
    sqr3_bottom_lft = (21.0 - (robot_radius + clearance), 21.0 - (robot_radius + clearance))
    sqr4_bottom_lft = (36.0 - (robot_radius + clearance), 21.0 - (robot_radius + clearance))
    sqr5_bottom_lft = (36.0 - (robot_radius + clearance), 4.0 - (robot_radius + clearance))
    sqr6_bottom_lft = (51.0 - (robot_radius + clearance), 4.0 - (robot_radius + clearance))
    sqr7_bottom_lft = (51.0 - (robot_radius + clearance), 21.0 - (robot_radius + clearance))

    # Creating buffer space for barricades    
    barricade1_buffer_vts = [(15.0 - (robot_radius + clearance), 30.0), (15.2 + (robot_radius + clearance), 30.0), (15.2 + (robot_radius + clearance), 12.0 - (robot_radius + clearance)), (15.0 - (robot_radius + clearance), 12.0 - (robot_radius + clearance))]
    barricade2_buffer_vts = [(30.0 - (robot_radius + clearance), 18.0 + (robot_radius + clearance)), (30.2 + (robot_radius + clearance), 18.0 - (robot_radius + clearance)), (30.2 + (robot_radius + clearance), 0), (30.0 - (robot_radius + clearance), 0)]
    barricade3_buffer_vts = [(45.0 - (robot_radius + clearance), 30.0), (45.2 + (robot_radius + clearance), 30.0), (45.2 + (robot_radius + clearance), 12.0 - (robot_radius + clearance)), (45.0 - (robot_radius + clearance), 12.0 - (robot_radius + clearance))]

    sqr1_buffer = is_point_inside_square(x, y, sqr1_bottom_lft, 3.0+2*(robot_radius + clearance))
    sqr2_buffer = is_point_inside_square(x, y, sqr2_bottom_lft, 3.0+2*(robot_radius + clearance))
    sqr3_buffer = is_point_inside_square(x, y, sqr3_bottom_lft, 3.0+2*(robot_radius + clearance))
    sqr4_buffer = is_point_inside_square(x, y, sqr4_bottom_lft, 3.0+2*(robot_radius + clearance))
    sqr5_buffer = is_point_inside_square(x, y, sqr5_bottom_lft, 3.0+2*(robot_radius + clearance))
    sqr6_buffer = is_point_inside_square(x, y, sqr6_bottom_lft, 3.0+2*(robot_radius + clearance))
    sqr7_buffer = is_point_inside_square(x, y, sqr7_bottom_lft, 3.0+2*(robot_radius + clearance))
    barricade1_buffer = is_point_inside_rectangle(x,y, barricade1_buffer_vts)
    barricade2_buffer = is_point_inside_rectangle(x,y, barricade2_buffer_vts)
    barricade3_buffer = is_point_inside_rectangle(x,y, barricade3_buffer_vts)
    
    # Setting buffer space constraints to obtain obstacle space
    if sqr1_buffer or sqr2_buffer or sqr3_buffer or sqr4_buffer or sqr5_buffer or sqr6_buffer or sqr7_buffer or barricade1_buffer or barricade2_buffer or barricade3_buffer:
        return False
    
    # Adding check if obstacle is in walls
    if y >= 30.0 - (robot_radius + clearance) or y <= (robot_radius + clearance):
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