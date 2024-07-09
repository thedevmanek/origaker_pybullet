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

POS_REACHED=True
WALK_FORWARD=False
WALK_BACKWARD=False
TURN_RIGHT=False
TURN_LEFT=False
import pybullet as p
import numpy as np
from path_planning import plan_path



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
import math

def vector_to_euler_with_respect_to_point(pos, ref_point):
    x, y, z = pos
    px, py, pz = ref_point
    
    # Translate the position to the new origin
    translated_x = x - px
    translated_y = y - py
    translated_z = z - pz
    
    # Compute the yaw (rotation around z-axis)
    yaw = math.atan2(translated_y, translated_x)
    
    # Compute the hypotenuse in the xy-plane
    hyp_xy = math.sqrt(translated_x**2 + translated_y**2)
    
    # Compute the pitch (rotation around y-axis)
    pitch = math.atan2(translated_z, hyp_xy)
    
    # Roll is assumed to be 0 as no rotation around x-axis is considered
    roll = 0
    
    # Convert to degrees
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
ORIENTATION_SET=False
def go_to_loc(dest_pos):
    global WALK_BACKWARD,WALK_FORWARD,TURN_LEFT,TURN_RIGHT,POS_REACHED,ORIENTATION_SET
    POS_REACHED=False
    # Fetch current pos and orientationn
    current_position, current_orientation = p.getBasePositionAndOrientation(robotId)
    rotation_quat = p.getQuaternionFromEuler([0, 0, -135*convert_rad_to_deg])
    resulting_quat = p.multiplyTransforms([0, 0, 0], current_orientation, [0, 0, 0], rotation_quat)[1]
    dest_orientation=vector_to_euler_with_respect_to_point(dest_pos,current_position)[2]*convert_rad_to_deg
    current_orientation=p.getEulerFromQuaternion(resulting_quat)
    current_position, _ = p.getBasePositionAndOrientation(robotId)
    dst_to_loc=get_distance(current_position,dest_pos)
    orient,yaw_diff=calculate_rotation_direction(current_orientation[2],dest_orientation)

    if dst_to_loc<0.25:
        print("DST TRIGGER")
        POS_REACHED=True
        ORIENTATION_SET=False
        return
    if True:
        if orient=='clockwise' and yaw_diff<-0.1:
            print("CLOCK")
            ORIENTATION_SET=False
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=True
            TURN_LEFT=False
        elif orient=='anticlockwise'and yaw_diff>0.1:
            print("ANTICLOCK")
            ORIENTATION_SET=False
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=True
        else:
            ORIENTATION_SET=True
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=False
    if ORIENTATION_SET :
        print("WALKING")
        if dst_to_loc>0.2:
            WALK_FORWARD=True
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=False
            
        else:
            WALK_FORWARD=False
            WALK_BACKWARD=False
            TURN_RIGHT=False
            TURN_LEFT=False
            POS_REACHED=True
            ORIENTATION_SET=False
            


# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setPhysicsEngineParameter(numSubSteps=1)

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



i=0
all_landmarks=[]
def check_new_landmarks(coordinates):
    global all_landmarks
    unique_coords=0
    for coord in coordinates:
        round_coords=(round(coord[0]*10), round(coord[1]*10),round(coord[2]*100))
        if round_coords not in all_landmarks:
            all_landmarks.append(round_coords)
            unique_coords=unique_coords+1
    print(unique_coords)
    if unique_coords>15:
        return True
    else:
        return False
while True:
    current_position, current_orientation = p.getBasePositionAndOrientation(robotId)
    lidar_scan = perform_lidar_scan(robotId,current_position, current_orientation, num_rays, lidar_range)
    new_landmarks=check_new_landmarks(lidar_scan)
    if new_landmarks:
        p.removeAllUserDebugItems()
        i=0
        pos_list=plan_path(all_landmarks,current_position,[1,2,0])
        # pointColorsRGB = [[1, 0, 0] for _ in range(len(pos_list))]
        # p.addUserDebugPoints(pos_list,pointColorsRGB, pointSize=10)

    basePos, baseOrn = p.getBasePositionAndOrientation(robotId) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=4.7, cameraYaw=0, cameraPitch=-85,cameraTargetPosition=basePos) # fix camera onto model
    if POS_REACHED:
        i=i+1
    go_to_loc(pos_list[i])
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

