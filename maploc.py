import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Function to read coordinates from a file
def read_coordinates(file_path):
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # Skip empty lines
                coords = line.strip('()\n').split(',')
                x, y, z = map(float, coords)
                coordinates.append((x, y, z))
    return coordinates

# Path to the file containing coordinates
file_path = 'location.txt'

# Read coordinates from the file
coordinates = read_coordinates(file_path)
# Extract x and y values
x_values = [coord[1] for coord in coordinates]
y_values = [coord[0] for coord in coordinates]

# Setup the plot
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(min(x_values) - 4, max(x_values) + 4)
ax.set_ylim(min(y_values) - 4, max(y_values) + 4)
ax.set_xlabel('X Coordinates')
ax.set_ylabel('Y Coordinates')
ax.set_title('Map of Coordinates')
ax.grid(True)

# Scatter plot to hold the points
scat = ax.scatter([], [], color='blue', marker='o')

# Update function for animation
def update(frame):
    current_coords = [(y_values[i], x_values[i]) for i in range(frame+1)]
    print(current_coords[-1])
    scat.set_offsets(current_coords)
    return scat,

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=len(coordinates), interval=100, blit=True, repeat=False)

# Display the plot
plt.show()
