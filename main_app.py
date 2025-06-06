import pybullet as p
import time
import math
import pybullet_data
from origaker_locomotion import Origaker
import numpy as np
import cv2


O = Origaker()
O.init_robot()
O.init_pose(O.POSE_MODEL_1)
fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

def get_obstacles(image):

    image = cv2.transpose(image)
    image = cv2.flip(image, flipCode=1)
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    # Apply a binary threshold to the grayscale image
    _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    thresh = cv2.bitwise_not(thresh)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    # Count closed contours
    closed_contours_count = 0
    for contour in contours:
                closed_contours_count += 1
    return closed_contours_count
def camera():
    # Center of mass position and orientation (of link-7)
    com_p, com_o, _, _, _, _ = p.getLinkState(O.robot_id, 6)
    # Convert quaternion to Euler angles
    yaw, pitch, roll = p.getEulerFromQuaternion(com_o)

    com_p=list(com_p)
    com_p[0]+=0.15
    com_p[1]+=0.1
    com_p[1]+=0.2
    # Modify the Euler angles
    roll += np.deg2rad(85)  # Rotate by 10 degrees in yaw direction
    
    # Convert back to quaternion
    com_o = p.getQuaternionFromEuler([yaw, pitch, roll])

    rot_matrix = p.getMatrixFromQuaternion(com_o)

    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    # Initial vectors
    init_camera_vector = (0, 0, 1) # z-axis
    init_up_vector = (0, 1, 0) # y-axis
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)

    p.getCameraImage(1000, 1000, view_matrix, projection_matrix)
    img = p.getCameraImage(1000, 1000, view_matrix, projection_matrix)
    return img
for i in range(7):
    O.move_robot(O.MOVE_RIGHT)

while True:
    img_arr=camera()
    rgb_image = np.reshape(img_arr[2], (1000, 1000, 4))[:, :, :3]
    if get_obstacles(rgb_image)>10:
          O.init_pose(O.POSE_MODEL_2)
    O.move_robot(O.MOVE_FORWARD)

