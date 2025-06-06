import pybullet as p
import numpy as np
from scipy.spatial import distance
import math
from origaker_locomotion import Origaker
from path_planning import plan_path
# LiDAR parameters
lidar_range = 200.0  # Maximum range of the LiDAR sensor
num_rays = 360      # Number of rays (resolution)
POS_REACHED=True
WALK_FORWARD=False
WALK_BACKWARD=False
TURN_RIGHT=False
TURN_LEFT=False
DEPLOY_GAP=False

def perform_lidar_scan(robotId,sensor_pos, sensor_orient, num_rays, lidar_range):
    lidar_data = []
    sensor_pos=list(sensor_pos)
    sensor_pos[2]=0.3
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
def go_to_loc(dest_pos,current_position):
    global WALK_BACKWARD,WALK_FORWARD,TURN_LEFT,TURN_RIGHT,POS_REACHED,ORIENTATION_SET
    POS_REACHED=False

    # Fetch current pos and orientationn
    _, current_orientation = p.getBasePositionAndOrientation(O.robot_id)
    rotation_quat = p.getQuaternionFromEuler([0, 0, math.radians(20)])
    if O.current_model==O.POSE_MODEL_3:
        rotation_quat = p.getQuaternionFromEuler([0, 0, math.radians(-38)])
    resulting_quat = p.multiplyTransforms([0, 0, 0], current_orientation, [0, 0, 0], rotation_quat)[1]
    print(current_orientation,rotation_quat)
    current_orientation=p.getEulerFromQuaternion(resulting_quat)
    dest_orientation=math.radians(vector_to_euler_with_respect_to_point(dest_pos,current_position)[2])
    dst_to_loc=get_distance(current_position,dest_pos)
    orient,yaw_diff=calculate_rotation_direction(current_orientation[2],dest_orientation)
    if dst_to_loc<0.25:
        print("DST TRIGGER")
        POS_REACHED=True
        ORIENTATION_SET=False
        return
    if orient=='clockwise' and yaw_diff<-0.3:
        print("CLOCK")
        ORIENTATION_SET=False
        WALK_FORWARD=False
        WALK_BACKWARD=False
        TURN_RIGHT=True
        TURN_LEFT=False
    elif orient=='anticlockwise'and yaw_diff>0.3:
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
            



O=Origaker()
O.init_robot()

O.init_pose(O.POSE_MODEL_1)



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

    if unique_coords>3:
        return True
    else:
        return False
pos_list=[]



def check_gap_distance(all_landmarks,current_position,pos_list):
    fin_set=[]
    current_position = [x * 10 for x in current_position]
    for coord in all_landmarks:
        fin_set.append(distance.euclidean(coord,current_position))
    return min(fin_set)
    # for i,pos in enumerate(pos_list):
    #     coords=[]
    #     pos = [x * 10 for x in pos]
    #     for coord in all_landmarks:
    #         # print(coord,pos)
    #         if coord[0]==pos[0]:
    #             coords.append(coord)
    #     if len(coords)>1:
    #         if (abs(coords[0][1])-abs(pos[1])+ abs(coords[1][1])-abs(pos[0]))<10:
    #             print(coords,pos)
    #             print((abs(coords[0][1])-abs(pos[1])+ abs(coords[1][1])-abs(pos[0])))
k=0
while True:

    current_position, current_orientation = p.getBasePositionAndOrientation(O.robot_id)
    print(current_position)
    if O.current_model==O.POSE_MODEL_3:
        current_position = list(current_position)
        current_position[0]=current_position[0]+0.09
        current_position[1]=current_position[1]-0.13
        current_position[2]=current_position[2]+0.3
    p.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=0, cameraPitch=-85,cameraTargetPosition=current_position) # fix camera onto model
  
    lidar_scan = perform_lidar_scan(O.robot_id,current_position, current_orientation, num_rays, lidar_range)
    new_landmarks=check_new_landmarks(lidar_scan)
    if new_landmarks and O.current_model!=O.POSE_MODEL_3_GAP:
        i=0
        pos_list=plan_path(all_landmarks,current_position,[0.1,1.4,0])
        if pos_list==None:
            pos_list=plan_path(all_landmarks,current_position,[0.1,1.4,0],robot_radius=0.1,grid_size=1)
            O.init_pose(O.POSE_MODEL_3)
    print(len(pos_list),i)
    if len(pos_list)-i<7 and O.current_model!=O.POSE_MODEL_3_GAP:
        O.move_robot(O.MOVE_RIGHT)
        O.move_robot(O.MOVE_RIGHT)
        O.move_robot(O.MOVE_RIGHT)
        O.current_model=O.POSE_MODEL_3_GAP
        
    
  
    
    if POS_REACHED:
        i=i+1
    go_to_loc(pos_list[i],current_position)
    if O.current_model== O.POSE_MODEL_3_GAP:
        O.move_robot(O.MOVE_FORWARD)
        if k==0:
            O.move_robot(O.MOVE_LEFT)
            k=k+1
    elif WALK_FORWARD:
        O.move_robot(O.MOVE_FORWARD)
    elif TURN_LEFT:
        O.move_robot(O.MOVE_LEFT)
    elif TURN_RIGHT:
        O.move_robot(O.MOVE_RIGHT)
