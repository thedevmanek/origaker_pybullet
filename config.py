import pybullet as p
import time
import math
import pybullet_data
from origaker_locomotion import Origaker
import numpy as np
import threading
import cv2
O = Origaker()
O.init_robot()

def joint_get():
    current_model=O.current_model
    num_joints = p.getNumJoints(O.robot_id)
    with open("joints.txt", "a") as file:
        while True:
            joint_data = []
            for joint_index in range(num_joints):
                joint_info = p.getJointInfo(O.robot_id, joint_index)
                joint_name = joint_info[1].decode("utf-8")  # Joint name as string
                joint_state = p.getJointState(O.robot_id, joint_index)
                joint_angle = joint_state[0]  # The position of the joint
                joint_angle=math.degrees(joint_angle)
                file.write(f"{joint_name},{joint_angle}")
            file.write("\n")  # Add a new line between frames if needed
            print(f"Joint angles and names saved to joints.txt")
            if current_model!=O.current_model:
                current_model=O.current_model
                file.write(f"{O.current_model}")
                file.write("\n")  # Add a new line between frames if needed

            time.sleep(0.1)
current_position, current_orientation = p.getBasePositionAndOrientation(O.robot_id)
joint_thread = threading.Thread(target=joint_get)
joint_thread.start()
O.init_pose(O.POSE_MODEL_4)
O.init_pose(O.POSE_MODEL_1)
O.init_pose(O.POSE_MODEL_3)
O.init_pose(O.POSE_MODEL_1)
O.init_pose(O.POSE_MODEL_2)
