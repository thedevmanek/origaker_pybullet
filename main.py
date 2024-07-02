import pybullet as p
import time
import pybullet_data

# Define the angle conversion factor
convert_rad_to_deg = 3.14 / 180

# Initialize angles in radians
angles = [
    ["base_tr1", convert_rad_to_deg * 0],
    ["tr1_tr2", -convert_rad_to_deg * 70],
    ["tr2_tr3", convert_rad_to_deg * 140],
    ["base_tl1", convert_rad_to_deg * 0],
    ["tl1_tl2", -convert_rad_to_deg * 70],
    ["tl2_tl3", convert_rad_to_deg * 140],
    ["base_bl1", convert_rad_to_deg * 0],
    ["bl1_bl2", -convert_rad_to_deg * 70],
    ["bl2_bl3", convert_rad_to_deg * 140],
    ["base_br1", convert_rad_to_deg * 0],
    ["br1_br2", -convert_rad_to_deg * 70],
    ["br2_br3", convert_rad_to_deg * 140]
]

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)

# Load the plane URDF
planeId = p.loadURDF("plane.urdf")

# Load the robot URDF
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("urdf/origaker_fix.urdf", startPos, startOrientation)

# Optionally set the center of mass frame (loadURDF sets base link frame) startPos/Orn
p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# Get joint indices (assuming the joints are named base_tr1, tr1_tr2, and tr2_tr3)
joint_names = ["base_tr1", "tr1_tr2", "tr2_tr3",
               "base_tl1", "tl1_tl2", "tl2_tl3",
               "base_br1", "br1_br2", "br2_br3",
               "base_bl1", "bl1_bl2", "bl2_bl3"]

joint_indices = {p.getJointInfo(boxId, i)[1].decode('UTF-8'): i for i in range(p.getNumJoints(boxId))}
settle_time = 2  # Time to let the robot fall and settle
start_time = time.time()
while time.time() - start_time < settle_time:
    p.stepSimulation()
    time.sleep(1. / 240.)

# Set initial joint angles
for joint_name, angle in angles:
    joint_index = joint_indices[joint_name]
    p.resetJointState(boxId, joint_index, targetValue=angle)

# PID parameters
kp = 1  # Proportional gain
kd = 0.1 # Derivative gain
ki = 0.1 # Integral gain
prev_error = {joint_name: 0 for joint_name in joint_names}
integral = {joint_name: 0 for joint_name in joint_names}

# Run the simulation
start_time = time.time()
while time.time()-start_time<5:
    for joint_name in joint_names:
        joint_index = joint_indices[joint_name]
        target_position = angles[joint_index][1]
        
        # Get the current joint position
        current_position = p.getJointState(boxId, joint_index)[0]
        
        # PID control
        error = target_position - current_position
        integral[joint_name] += error
        derivative = error - prev_error[joint_name]
        control_signal = kp * error + ki * integral[joint_name] + kd * derivative
        prev_error[joint_name] = error
        
        # Apply control signal
        p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=current_position + control_signal,
            force=10  # Adjust the force as needed
        )
    
    p.stepSimulation()
    time.sleep(1. / 240.)
time.sleep(2)
while True:
    
# Disconnect from PyBullet (this won't be reached in this infinite loop, but it's good practice to include it)
p.disconnect()
