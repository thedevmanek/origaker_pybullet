import pybullet as p
import time
import pybullet_data

def run_single_joint_simulation(joint_name, target_angle, boxId,duration=0.1, force=5):
    # Get joint index for the specified joint
    joint_index = joint_indices[joint_name]

    # Start the simulation
    start_time = time.time()
    while time.time() - start_time < duration:
        # Apply control signal
        p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=force
        )
        
        p.stepSimulation()
        time.sleep(1. / 240.)

def run_double_joint_simulation(joint_names, target_angle, boxId,duration=0.1, force=5):
    # Get joint index for the specified joint
    joint_index_1 = joint_indices[joint_names[0]]
    joint_index_2 = joint_indices[joint_names[1]]

    # Start the simulation
    start_time = time.time()
    while time.time() - start_time < duration:
        # Apply control signal
        p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=joint_index_1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=force
        )
        p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=joint_index_2,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
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
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("urdf/origaker_fix.urdf", startPos, startOrientation)

# Get joint indices
joint_indices = {p.getJointInfo(boxId, i)[1].decode('UTF-8'): i for i in range(p.getNumJoints(boxId))}
convert_rad_to_deg = 3.14 / 180
# Define target angle (in radians)
target_angle = convert_rad_to_deg * 10  # Example target angle for the joint
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
settle_time = 2  # Time to let the robot fall and settle
start_time = time.time()
while time.time() - start_time < settle_time:
    p.stepSimulation()
    time.sleep(1. / 240.)

for angle in angles:
    run_single_joint_simulation(angle[0],angle[1], boxId)

while True:
    run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 90, boxId)
    run_single_joint_simulation("base_tr1",convert_rad_to_deg * 40, boxId)
    run_single_joint_simulation("tr1_tr2",-convert_rad_to_deg * 70, boxId)
    run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 90, boxId)
    run_single_joint_simulation("base_br1",convert_rad_to_deg * 40, boxId)
    run_single_joint_simulation("br1_br2",-convert_rad_to_deg * 70, boxId)
    run_double_joint_simulation(["base_br1","base_tr1"],convert_rad_to_deg * 0, boxId)
    run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 90, boxId)
    run_single_joint_simulation("base_tl1",convert_rad_to_deg * 40, boxId)
    run_single_joint_simulation("tl1_tl2",-convert_rad_to_deg * 70, boxId)
    run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 90, boxId)
    run_single_joint_simulation("base_bl1",convert_rad_to_deg * 40, boxId)
    run_single_joint_simulation("bl1_bl2",-convert_rad_to_deg * 70, boxId)
    run_double_joint_simulation(["base_bl1","base_tl1"],convert_rad_to_deg * 0, boxId)
# Disconnect from PyBullet
p.disconnect()
