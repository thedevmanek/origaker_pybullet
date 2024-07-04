import pybullet as p
import numpy as np
import time
import pybullet_data
from wall_creator import gen_room
# LiDAR parameters
lidar_range = 20.0  # Maximum range of the LiDAR sensor
num_rays = 360      # Number of rays (resolution)

WALK_FORWARD=True
WALK_BACKWARD=False
TURN_RIGHT=False
TURN_LEFT=False
import pybullet as p
import numpy as np

def perform_lidar_scan(robotId,sensor_pos, sensor_orient, num_rays, lidar_range):
    lidar_data = []
    
    for i in range(num_rays):
        # Calculate the angle for this ray
        angle = 2 * np.pi * i / num_rays
        
        # Define the direction of the ray
        ray_dir = [np.cos(angle), np.sin(angle), 0]
        
        # Cast a ray from the sensor position
        ray_from = sensor_pos
        ray_to = [sensor_pos[j] + lidar_range * ray_dir[j] for j in range(3)]
        
        # Perform the ray test with a callback to filter out the robot
        # rayTest returns a tuple: (objectUniqueId, linkIndex, hitPos, hitNormal)
        ray_result = p.rayTest(ray_from, ray_to)
        
        # Filter out the robot
        filtered_result = [result for result in ray_result if result[0] != robotId]
        
        if filtered_result:
            # Append the hit position if there is any valid result
            lidar_data.append(filtered_result[0][3])

    return lidar_data



def run_single_joint_simulation(joint_name, target_angle, robotId,duration=0.1, force=5):
    # Get joint index for the specified joint
    joint_index = joint_indices[joint_name]

    # Start the simulation
    start_time = time.time()
    while time.time() - start_time < duration:
        # Apply control signal
        p.setJointMotorControl2(
            bodyUniqueId=robotId,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=force
        )
        
        p.stepSimulation()
        time.sleep(1. / 240.)

def run_double_joint_simulation(joint_names, target_angle1,target_angle2, robotId,duration=0.5, force=5):
    # Get joint index for the specified joint
    joint_index_1 = joint_indices[joint_names[0]]
    joint_index_2 = joint_indices[joint_names[1]]

    # Start the simulation
    start_time = time.time()
    while time.time() - start_time < duration:
        # Apply control signal
        p.setJointMotorControl2(
            bodyUniqueId=robotId,
            jointIndex=joint_index_1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle1,
            force=force
        )
        p.setJointMotorControl2(
            bodyUniqueId=robotId,
            jointIndex=joint_index_2,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle2,
            force=force
        )
        
        p.stepSimulation()
        time.sleep(1. / 240.)



# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)

# Load the plane and robot URDF
planeId = p.loadURDF("plane.urdf")


startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 95])
robotId = p.loadURDF("urdf/origaker_fix.urdf", startPos, startOrientation)

gen_room()
# Get joint indices
joint_indices = {p.getJointInfo(robotId, i)[1].decode('UTF-8'): i for i in range(p.getNumJoints(robotId))}
convert_rad_to_deg = 3.14 / 180
# Define target angle (in radians)
target_angle = convert_rad_to_deg * 10  # Example target angle for the joint
angles = [
    ["base_tr1", convert_rad_to_deg * 8],
    ["tr1_tr2", -convert_rad_to_deg * 70],
    ["tr2_tr3", convert_rad_to_deg * 140],
    ["base_tl1", convert_rad_to_deg * 10],
    ["tl1_tl2", -convert_rad_to_deg* 70],
    ["tl2_tl3", convert_rad_to_deg * 140],
    ["base_bl1", convert_rad_to_deg * 8],
    ["bl1_bl2", -convert_rad_to_deg * 70],
    ["bl2_bl3", convert_rad_to_deg * 140],
    ["base_br1", convert_rad_to_deg * 15],
    ["br1_br2", -convert_rad_to_deg * 70],
    ["br2_br3", convert_rad_to_deg * 140]
]
settle_time = 2  # Time to let the robot fall and settle
start_time = time.time()
while time.time() - start_time < settle_time:
    p.stepSimulation()
    time.sleep(1)

for angle in angles:
    run_single_joint_simulation(angle[0],angle[1], robotId,duration=0.8)

open('location.txt', 'w').close()
open("pointcloud.txt", "w").close()

while True:
    basePos, baseOrn = p.getBasePositionAndOrientation(robotId) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=0.7, cameraYaw=0, cameraPitch=-85,cameraTargetPosition=basePos) # fix camera onto model
    time.sleep(0.01)
    if WALK_FORWARD:
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tl1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_bl1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 70, robotId)
        time.sleep(0.5)
        run_double_joint_simulation(["base_bl1","base_tl1"],convert_rad_to_deg * 13,convert_rad_to_deg * 13, robotId)
        current_position, previous_orientation = p.getBasePositionAndOrientation(robotId)
        with open("location.txt", "a") as file:
            file.write(f"{current_position}\n")
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tr1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_br1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 70, robotId)
        time.sleep(0.5)
        run_double_joint_simulation(["base_br1","base_tr1"],convert_rad_to_deg * 15,convert_rad_to_deg * 8, robotId)
        current_position, current_orientation = p.getBasePositionAndOrientation(robotId)
        print()
        with open("location.txt", "a") as file:
            file.write(f"{current_position}\n")
        lidar_scan = perform_lidar_scan(robotId,current_position, current_orientation, num_rays, lidar_range)
        with open("pointcloud.txt", "a") as file:
            file.write(f"{lidar_scan}\n")

    elif WALK_BACKWARD:
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tr1",convert_rad_to_deg * 40, robotId)
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_br1",convert_rad_to_deg * 40, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 70, robotId)
        run_double_joint_simulation(["base_br1","base_tr1"],convert_rad_to_deg * 20,convert_rad_to_deg * 10, robotId)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tl1",convert_rad_to_deg * 40, robotId)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_bl1",convert_rad_to_deg * 40, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 70, robotId)
        run_double_joint_simulation(["base_bl1","base_tl1"],convert_rad_to_deg * 0,convert_rad_to_deg * 0, robotId)


 # Set up the plot
fig, ax = plt.subplots()
ax.set_title('Live Robot Path')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.grid()
line, = ax.plot([], [], marker='o', markersize=2, linestyle='-', color='blue')
# Initialize plot limits
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)