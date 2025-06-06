import pybullet as p
import pybullet_data
import random
import time


def create_complex_scene():
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())


    # Load the stone mesh
    stoneId = p.createCollisionShape(p.GEOM_MESH, fileName="stone.obj", meshScale=[0.09] * 3)

    # Define the number of rows and stones per row
    num_rows = 5
    num_stones_per_row = 10

    # Define the base position for the first stone
    base_position = [0, 0, 0]  # Adjust the Z coordinate to prevent it from being underground

    # Create multiple rows of stones with random distances between them
    for row in range(num_rows):
        for i in range(num_stones_per_row):
            # Generate random positions within a range for more randomness
            random_x = random.uniform(0.05, 0.2) * i  # Denser random distance along the X-axis
            random_y = random.uniform(-0.05, 0.05)  # Small random offset along the Y-axis
            
            # Set the position for the stone
            stone_position = [base_position[0] + random_x, base_position[1] + random_y, base_position[2]]
            
            # Create the stone
            p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=stoneId,
                            basePosition=stone_position)
        
        # Update the base position for the next row (move along the Y-axis)
        base_position[1] += random.uniform(0.05, 0.2)  # Smaller random distance along the Y-axis for density



