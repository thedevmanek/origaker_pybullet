import matplotlib.pyplot as plt
import ast

# Load coordinates from the text file
def load_coordinates(filename):
    coordinates = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.strip():  # Check if the line is not empty
                coords = ast.literal_eval(line.strip())
                coordinates.append(coords)  # Combine all coordinates into a single list
             
    return coordinates

# Path to your file
filename = 'pointcloud.txt'

# Read coordinates from file
coordinates = load_coordinates(filename)
print(len(coordinates))
for i in range(len(coordinates)):
    # Unpack coordinates
    x, y, _ = zip(*coordinates[i])  # Ignore z coordinate

    # Create a new figure for plotting
    plt.figure()

    # Plot the coordinates
    plt.scatter(x, y, c='r', marker='o')

    # Label axes
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')

    # Set plot title
    plt.title('2D Scatter Plot of Coordinates')

    # Show plot
    plt.show()
