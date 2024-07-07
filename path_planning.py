import heapq
import ast
import matplotlib.pyplot as plt

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to this node
        self.h = 0  # Heuristic cost from this node to end
        self.f = 0  # Total cost

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

def manhattan_distance(a, b):
    """Calculate the Manhattan distance between points a and b"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

def create_collision_set(collision_blocks):
    """Convert collision blocks to a set for O(1) lookups"""
    collision_set = set()
    for (block_x, block_y) in collision_blocks:
        for i in range(-3, 3, 1):
            for j in range(-3, 3, 1):
                collision_set.add((block_x + i, block_y + j))
    return collision_set

def is_collision(point, collision_set):
    """Check if a point collides with any of the 2D blocks on the x, y plane"""
    x, y, _ = point
    return (x, y) in collision_set

def simplify_path(path):
    """Simplify the path by collapsing consecutive points with the same direction"""
    if not path:
        return []

    simplified_path = []
    current_direction = None
    segment_start = path[0]
    simplified_path.append(segment_start)

    for i in range(1, len(path)):
        prev_point = path[i - 1]
        current_point = path[i]

        # Calculate direction change
        direction = (
            current_point[0] - prev_point[0],
            current_point[1] - prev_point[1],
            current_point[2] - prev_point[2]
        )

        if direction != current_direction:
            if current_direction is not None:
                # If the direction changed, record the end of the segment
                simplified_path.append(segment_start)
            # Start a new segment
            segment_start = prev_point
            current_direction = direction

    # Add the final segment
    if segment_start != path[-1]:
        simplified_path.append(path[-1])

    return simplified_path

def astar(start, end, collision_set):
    open_list = []
    closed_list = set()

    start_node = Node(start)
    end_node = Node(end)
    
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path.reverse()  # Reverse path to get it from start to end
            return simplify_path(path)  # Simplify the path

        # Explore neighbors (6 directions for 3D grid)
        neighbors = [(current_node.position[0] + dx, current_node.position[1] + dy, current_node.position[2] + dz)
                     for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]

        for neighbor_pos in neighbors:
            if neighbor_pos in closed_list or is_collision(neighbor_pos, collision_set):
                continue

            neighbor_node = Node(neighbor_pos, current_node)
            neighbor_node.g = current_node.g + 1  # Cost to move to a neighbor
            neighbor_node.h = manhattan_distance(neighbor_pos, end_node.position)
            neighbor_node.f = neighbor_node.g + neighbor_node.h
            
            # Check if node is already in the open list
            if any(neighbor_node.position == item.position and neighbor_node.f >= item.f for item in open_list):
                continue

            heapq.heappush(open_list, neighbor_node)

    return []  # Return empty if no path is found

def load_coordinates(filename):
    coordinates = []
    with open(filename, 'r') as file:
        for line in file:
            if line.strip():  # Check if the line is not empty
                coords = ast.literal_eval(line.strip())
                coordinates.append(coords)  # Combine all coordinates into a single list
             
    return coordinates

def visualize(start, end, obstacles, path):
    fig, ax = plt.subplots()

    # Plot obstacles
    for (x, y) in obstacles:
        rect = plt.Rectangle((x, y), 1, 1, color='red', alpha=0.5)
        ax.add_patch(rect)

    # Plot start and end points
    ax.plot(*start, 'go', label='Start')
    ax.plot(*end, 'bo', label='End')

    # Plot the path
    if path:
        path_x, path_y, _ = zip(*path)
        ax.plot(path_x, path_y, 'k--', label='Path')

    # Set limits and labels
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

# Path to your file
filename = 'pointcloud.txt'

# Read coordinates from file
coordinates = load_coordinates(filename)
round_coords = []
for coord in coordinates[0]:
    round_coords.append((round(coord[0] * 10), round(coord[1] * 10)))
print(round_coords[0])

collision_set = create_collision_set(round_coords)

start = (0, 0, 0)
end = (1 * 10, 4 * 10, 0)

path = astar(start, end, collision_set)
print("Path found:", path)
visualize(start, end, round_coords, path)
