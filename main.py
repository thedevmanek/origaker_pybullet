import pybullet as p
import numpy as np
import time
import pybullet_data
from wall_creator import gen_room
import open3d as o3d
import numpy as np
import math
# LiDAR parameters
lidar_range = 20.0  # Maximum range of the LiDAR sensor
num_rays = 360      # Number of rays (resolution)

WALK_FORWARD=False
WALK_BACKWARD=False
TURN_RIGHT=False
TURN_LEFT=False
import pybullet as p
import numpy as np
import pybullet_planning as pp



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

        if ray_result[0][0]==robotId:
            new_coord=(ray_result[0][3][0]+0.1,ray_result[0][3][1]+0.1,ray_result[0][3][2]+0.1)
            ray_test1=p.rayTest(new_coord,ray_to)
            lidar_data.append(ray_test1[0][3])
        else:
            lidar_data.append(ray_result[0][3])
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
def vector_to_euler(pos):
    x,y,z=pos
    yaw = math.atan2(y, x)
    hyp_xy = math.sqrt(x**2 + y**2)
    pitch = math.atan2(z, hyp_xy)
    roll = 0
    yaw_deg = math.degrees(yaw)
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)
    
    return roll_deg, pitch_deg, yaw_deg

def get_distance(start,stop):
    x1,y1,_=start
    x2,y2,_=stop
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_rotation_direction(current_orientation, dest_orientation):
    yaw_diff = normalize_angle(dest_orientation - current_orientation)
    if yaw_diff<0:
        return 'clockwise',yaw_diff
    else:
        return 'anticlockwise',yaw_diff

def go_to_loc(current_orientation,dest_pos):
    global WALK_BACKWARD,WALK_FORWARD,TURN_LEFT,TURN_RIGHT
    rotation_quat = p.getQuaternionFromEuler([0, 0, -135*convert_rad_to_deg])
    resulting_quat = p.multiplyTransforms([0, 0, 0], current_orientation, [0, 0, 0], rotation_quat)[1]
    dest_orientation=vector_to_euler(dest_pos)[2]*convert_rad_to_deg
    current_orientation=p.getEulerFromQuaternion(resulting_quat)
    current_position, _ = p.getBasePositionAndOrientation(robotId)
    dst_to_loc=get_distance(current_position,dest_pos)
    if not WALK_FORWARD and not WALK_BACKWARD:
        orient,yaw_diff=calculate_rotation_direction(current_orientation[2],dest_orientation)
        print(orient,yaw_diff)
        if orient=='clockwise' and yaw_diff<-0.1:
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=True
            TURN_LEFT=False
        elif orient=='anticlockwise'and yaw_diff>0.1:
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=True
        else:
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=False
    if not TURN_LEFT and not TURN_RIGHT:
        print(dst_to_loc)
        if dst_to_loc>0.15:
            WALK_FORWARD=True
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=False
        else:
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=False


# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)

# Load the plane and robot URDF
planeId = p.loadURDF("plane.urdf")

convert_rad_to_deg = 3.14 / 180
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0,0])
robotId = p.loadURDF("urdf/origaker_fix.urdf", startPos, startOrientation)

gen_room()
# Get joint indices
joint_indices = {p.getJointInfo(robotId, i)[1].decode('UTF-8'): i for i in range(p.getNumJoints(robotId))}

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
    run_single_joint_simulation(angle[0],angle[1], robotId,duration=0.5)

open('location.txt', 'w').close()
open("pointcloud.txt", "w").close()

while True:
    basePos, baseOrn = p.getBasePositionAndOrientation(robotId) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=4.7, cameraYaw=0, cameraPitch=-85,cameraTargetPosition=basePos) # fix camera onto model
    #Mapping
    current_position, current_orientation = p.getBasePositionAndOrientation(robotId)
    lidar_scan = perform_lidar_scan(robotId,current_position, current_orientation, num_rays, lidar_range)
    with open("pointcloud.txt", "a") as file:
        file.write(f"{lidar_scan}\n")
    req_position=[1,-1,0]
    go_to_loc(current_orientation,req_position)
    if WALK_FORWARD:
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tl1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_bl1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 70, robotId)
        time.sleep(0.5)
        run_double_joint_simulation(["base_bl1","base_tl1"],convert_rad_to_deg * 13,convert_rad_to_deg * 13, robotId)
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tr1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_br1",-convert_rad_to_deg * 40, robotId,duration=0.2)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 70, robotId)
        time.sleep(0.5)
        run_double_joint_simulation(["base_br1","base_tr1"],convert_rad_to_deg * 15,convert_rad_to_deg * 8, robotId)
        lidar_scan = perform_lidar_scan(robotId,current_position, current_orientation, num_rays, lidar_range)


    elif WALK_BACKWARD:
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tr1",convert_rad_to_deg * 40, robotId)
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_br1",convert_rad_to_deg * 40, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 70, robotId)
        run_double_joint_simulation(["base_br1","base_tr1"],convert_rad_to_deg * 20,convert_rad_to_deg * 10, robotId)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tl1",convert_rad_to_deg * 60, robotId)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_bl1",convert_rad_to_deg * 60, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 70, robotId)
        run_double_joint_simulation(["base_bl1","base_tl1"],convert_rad_to_deg * 0,convert_rad_to_deg * 0, robotId)
    elif TURN_LEFT:
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tl1",-convert_rad_to_deg * 60, robotId)
        run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 70, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_bl1",-convert_rad_to_deg * 60, robotId)
        run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 70, robotId)
        run_double_joint_simulation(["base_bl1","base_tl1"],convert_rad_to_deg * 0,convert_rad_to_deg * 0, robotId)
    elif TURN_RIGHT:
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_tr1",-convert_rad_to_deg * 60, robotId)
        run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 80, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, robotId)
        run_single_joint_simulation("base_br1",-convert_rad_to_deg * 60, robotId)
        run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, robotId)
        run_double_joint_simulation(["base_br1","base_tr1"],convert_rad_to_deg * 20,convert_rad_to_deg * 10, robotId)

