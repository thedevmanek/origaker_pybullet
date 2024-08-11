import matplotlib.pyplot as plt
import re
current_time_step = 0

def is_number(s):
    """Check if the input string s represents a number (integer or float)."""
    try:
        float(s)  # Try converting to float
        return True
    except ValueError:
        return False
joints=[]
pose_data = []
pose_time = []
pose_labels = []

# Read the file and check if each line is a number
with open('joints.txt', 'r') as file:
    for line in file:
        stripped_line = line.strip()  # Remove leading/trailing whitespace
        if is_number(stripped_line):
            print(stripped_line,len(joints))
            pose_data.append(stripped_line)
            pose_time.append(len(joints)*0.1)
            pose_labels.append(f'Model {stripped_line}')  # Assign a label for each pose

        else:
            joints.append(stripped_line)
joint_data = {}

for joint1 in joints:
    pattern = r'(JOINT_\w+),(-?\d+\.?\d*e?-?\d*)|(-?\d+\.?\d*)'
    matches = re.findall(pattern, joint1)
    for match in matches:
        joint, angle, pose = match
        if joint:  # If joint is present
            if joint not in joint_data:
                joint_data[joint] = []
            joint_data[joint].append(float(angle))
            current_time_step += 0.1

# Plotting
plt.figure(figsize=(14, 8))

# Plot joint angles
for joint, angles in joint_data.items():
    plt.plot([i * 0.1 for i in range(len(angles))], angles, label=f'{joint}')


# Add vertical lines and labels for pose configurations
for pt, label in zip(pose_time, pose_labels):
    plt.axvline(x=pt, color='r', linestyle='--')
    plt.text(pt+2, plt.ylim()[1] * 0.95, label, rotation=0, color='r', ha='left')

plt.title('Joint Angles')
plt.xlabel('Time Step')
plt.ylabel('Angle (degrees)')
plt.legend(loc='top right', bbox_to_anchor=(1, 1), fancybox=True, shadow=True)
plt.grid(True)
plt.show()

