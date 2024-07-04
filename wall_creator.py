import pybullet as p
import pybullet_data

# Room dimensions
room_length = 5
room_width = 5
room_height = 2
wall_thickness = 0.1

# Function to create a wall
def create_wall(start_pos, orientation, length, height, thickness):
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          halfExtents=[length / 2, thickness / 2, height / 2])
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[length / 2, thickness / 2, height / 2])
    wall_id = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=collision_shape_id,
                                baseVisualShapeIndex=visual_shape_id,
                                basePosition=start_pos,
                                baseOrientation=orientation)
    return wall_id

def gen_room():
    # Front wall
    create_wall(start_pos=[0, room_width / 2, room_height / 2],
                orientation=p.getQuaternionFromEuler([0, 0, 0]),
                length=room_length, height=room_height, thickness=wall_thickness)

    # Back wall
    create_wall(start_pos=[0, -room_width / 2, room_height / 2],
                orientation=p.getQuaternionFromEuler([0, 0, 0]),
                length=room_length, height=room_height, thickness=wall_thickness)

    # Left wall
    create_wall(start_pos=[room_length / 2, 0, room_height / 2],
                orientation=p.getQuaternionFromEuler([0, 0, 1.5708]),  # 90 degrees in radians
                length=room_width, height=room_height, thickness=wall_thickness)

    # Right wall
    create_wall(start_pos=[-room_length / 2, 0, room_height / 2],
                orientation=p.getQuaternionFromEuler([0, 0, 1.5708]),  # 90 degrees in radians
                length=room_width, height=room_height, thickness=wall_thickness)


