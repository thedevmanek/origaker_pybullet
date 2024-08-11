import time
import math
import pybullet as p
import pybullet_data
from wall_creator import gen_room

class Origaker:
    POSE_MODEL_1=1
    POSE_MODEL_2=2
    POSE_MODEL_3=3
    POSE_MODEL_4=4
    POSE_MODEL_3_GAP=8
    MOVE_FORWARD=5
    MOVE_RIGHT=6
    MOVE_LEFT=7
    def __init__(self):
        self.joint_name_to_index={}
        self.robot_id=None
        self.current_model=self.POSE_MODEL_1


    def init_robot(self):

        # Connect to PyBullet
        physicsClient = p.connect(p.GUI,options='--background_color_red=0.0 --background_color_green=1.0 --background_color_blue=0.0')
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.87)
        # Load the plane
        planeId = p.loadURDF("plane.urdf")
        # Load the robot -1.5,1.2
        
        self.robot_id = p.loadURDF("urdf/origaker.urdf",basePosition=[0,0,0])
        # gen_room()
        
        # create_complex_scene()
        settle_time = 1  # Time to let the robot fall and settle
        start_time = time.time()
        while time.time() - start_time < settle_time:
            p.stepSimulation()
            time.sleep(1. / 240.)
        for _id in range(p.getNumJoints(self.robot_id)):
            _name = p.getJointInfo(self.robot_id, _id)[1].decode('UTF-8')
            self.joint_name_to_index[_name] = _id


    def __run_double_joint_simulation(self,joint_names, target_angle1,target_angle2,duration=0.5, force=5):
        # Get joint index for the specified joint
        joint_index_1 = self.joint_name_to_index[joint_names[0]]
        joint_index_2 = self.joint_name_to_index[joint_names[1]]

        # Start the simulation
        start_time = time.time()
        while time.time() - start_time < duration:
            # Apply control signal
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_index_1,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle1,
                force=force
            )
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_index_2,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle2,
                force=force
            )
            
            p.stepSimulation()
            time.sleep(1. / 240.)

    def __run_single_joint_simulation(self,joint_name, target_angle,duration=0.25, force=5):
        # Get joint index for the specified joint
        joint_index = self.joint_name_to_index[joint_name]

        # Start the simulation
        start_time = time.time()
        while time.time() - start_time < duration:
            # Apply control signal
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=force
            )
            
            p.stepSimulation()
            time.sleep(1. / 240.)

    def __model_1_activate(self):
        self.current_model=self.POSE_MODEL_1
        self.__run_single_joint_simulation('JOINT_BL_BR', 0,force=1)
        self.__run_single_joint_simulation('JOINT_TLS_BLS', 0,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BRS_TRS', 0,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BLS_BL', 0,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BR_BRS', 0,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TL_TLS', 0,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TR_TRS', 0,force=1,duration=0.5)
        self.__run_double_joint_simulation(['JOINT_BL1_BL2','JOINT_BR1_BR2'],math.radians(-70), math.radians(-70),force=0.2,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TL1_TL2','JOINT_TR1_TR2'],math.radians(-70), math.radians(-70),force=0.2,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TL2_TL3','JOINT_BR2_BR3'],math.radians(140), math.radians(140),force=1,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_BL2_BL3','JOINT_TR2_TR3'],math.radians(140), math.radians(140),force=1,duration=0.25)
        time.sleep(1. / 240.)
    def __model_2_activate(self):
        self.current_model=self.POSE_MODEL_2
        self.__run_single_joint_simulation('JOINT_BL_BR', 0,force=1)
        self.__run_double_joint_simulation(['JOINT_BLS_BL1','JOINT_BRS_BR1'],math.radians(-20), math.radians(-20),force=1,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TLS_TL1','JOINT_TRS_TR1'],math.radians(20), math.radians(20),force=1,duration=0.25)
        self.__run_single_joint_simulation('JOINT_TL_TLS', -0.285,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TR_TRS', -0.285,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BLS_BL', -0.26,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BR_BRS', -0.26,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TLS_BLS', 0.521,force=1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BRS_TRS', 0.521,force=1,duration=0.5)
        self.__run_double_joint_simulation(['JOINT_BL1_BL2','JOINT_BR1_BR2'],math.radians(-60), math.radians(-60),force=1,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TL1_TL2','JOINT_TR1_TR2'],math.radians(-60), math.radians(-60),force=1,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TL2_TL3','JOINT_BR2_BR3'],math.radians(140), math.radians(140),force=1,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_BL2_BL3','JOINT_TR2_TR3'],math.radians(140), math.radians(140),force=1,duration=0.25)
        self.__run_single_joint_simulation('JOINT_BLS_BL', -0.26,force=1.2,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BR_BRS', -0.26,force=1.2,duration=0.5)

        time.sleep(1. / 240.)
    def __model_3_activate(self):
        self.current_model=self.POSE_MODEL_3

        self.__run_single_joint_simulation('JOINT_TL_TLS', -1.4,force=0.1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TR_TRS', -1.4,force=0.1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BLS_BL', -1.42,force=0.1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BR_BRS', -1.42,force=0.1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TLS_BLS', 2.8,force=0.1,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BRS_TRS', 2.8,force=0.1,duration=0.5)
        
        self.__run_double_joint_simulation(["JOINT_TRS_TR1","JOINT_BRS_BR1"],math.radians(0),math.radians(0),force=0.1)
        self.__run_double_joint_simulation(["JOINT_TLS_TL1","JOINT_BLS_BL1"],math.radians(0),math.radians(0),force=0.1)

        self.__run_double_joint_simulation(['JOINT_BL1_BL2','JOINT_TL1_TL2'],math.radians(-20), math.radians(-20),force=0.1,duration=0.5)
        self.__run_double_joint_simulation(['JOINT_BR1_BR2','JOINT_TR1_TR2'],math.radians(-20), math.radians(-20),force=0.1,duration=0.5)
        self.__run_double_joint_simulation(['JOINT_BL2_BL3','JOINT_TL2_TL3'],math.radians(120), math.radians(120),force=1,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_BR2_BR3','JOINT_TR2_TR3'],math.radians(120), math.radians(120),force=1,duration=0.25)
        time.sleep(1. / 240.)

    def __model_4_activate(self):
        self.current_model=self.POSE_MODEL_4
        self.__run_single_joint_simulation('JOINT_BLS_BL1', math.radians(70),force=0.5,duration=0.25)
        self.__run_single_joint_simulation('JOINT_BRS_BR1', math.radians(70),force=0.5,duration=0.25)
        self.__run_single_joint_simulation('JOINT_TLS_TL1', math.radians(-70),force=0.5,duration=0.25)
        self.__run_single_joint_simulation('JOINT_TRS_TR1', math.radians(-70),force=0.5,duration=0.25)
        self.__run_single_joint_simulation('JOINT_TL_TLS', -0.285,force=3,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TR_TRS', -0.285,force=3,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BLS_BL', -0.26,force=3,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BR_BRS', -0.26,force=3,duration=0.5)
        self.__run_single_joint_simulation('JOINT_TLS_BLS', 0.529,force=6,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BRS_TRS', 0.529,force=6,duration=0.5)
        self.__run_single_joint_simulation('JOINT_BL_BR', -2.6,force=0.1,duration=2)
        self.__run_double_joint_simulation(['JOINT_BL1_BL2','JOINT_BR1_BR2'],math.radians(90), math.radians(90),force=0.09,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TL1_TL2','JOINT_TR1_TR2'],math.radians(90), math.radians(90),force=0.09,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_BLS_BL1','JOINT_BRS_BR1'],math.radians(0), math.radians(0),force=0.09,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TLS_TL1','JOINT_TRS_TR1'],math.radians(0), math.radians(0),force=0.09,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_BL2_BL3','JOINT_BR2_BR3'],math.radians(-90), math.radians(-90),force=0.09,duration=0.25)
        self.__run_double_joint_simulation(['JOINT_TL2_TL3','JOINT_TR2_TR3'],math.radians(-90), math.radians(-90),force=0.09,duration=0.25)
        
        time.sleep(1. / 240.)

    def init_pose(self,pose):
        current_position, current_orientation = p.getBasePositionAndOrientation(self.robot_id)
        p.resetDebugVisualizerCamera( cameraDistance=0.8, cameraYaw=10, cameraPitch=-45,cameraTargetPosition=current_position) # fix camera onto model
        if pose == self.current_model and self.current_model!=self.POSE_MODEL_1:
            return
        elif pose==self.POSE_MODEL_1:
            self.__model_1_activate()
        elif pose==self.POSE_MODEL_2:
            self.__model_2_activate()
        elif pose==self.POSE_MODEL_3:
            self.__model_3_activate()
        elif pose==self.POSE_MODEL_4:
            self.__model_4_activate()
    
    def forward_movement(self):
        if self.current_model==self.POSE_MODEL_1:
            # Move left Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TLS_TL1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-70))
            # Move left Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BLS_BL1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-70))
            time.sleep(0.5)
            # Move both Legs Forward
            self.__run_double_joint_simulation(["JOINT_BLS_BL1","JOINT_TLS_TL1"],math.radians(0),math.radians(0))
            # Move right Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TRS_TR1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-70))
            # Move right Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BRS_BR1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-70))
            time.sleep(0.5)
            # Move both Legs Backward
            self.__run_double_joint_simulation(["JOINT_BRS_BR1","JOINT_TRS_TR1"],math.radians(0),math.radians(0))
        elif self.current_model==self.POSE_MODEL_2:
            # Move left Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-110))
            self.__run_single_joint_simulation("JOINT_TLS_TL1",math.radians(-80),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-70))
            # Move left Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-110))
            self.__run_single_joint_simulation("JOINT_BLS_BL1",math.radians(-80),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-70))
            time.sleep(0.5)
            # Move both Legs Forward
            self.__run_double_joint_simulation(["JOINT_BLS_BL1","JOINT_TLS_TL1"],math.radians(0),math.radians(-0))
            # Move right Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-110))
            self.__run_single_joint_simulation("JOINT_TRS_TR1",math.radians(-80),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-70))
            # Move right Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-110))
            self.__run_single_joint_simulation("JOINT_BRS_BR1",math.radians(-80),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-70))
            time.sleep(0.5)
            # Move both Legs Backward
            self.__run_double_joint_simulation(["JOINT_BRS_BR1","JOINT_TRS_TR1"],math.radians(0),math.radians(-0))



        elif self.current_model==self.POSE_MODEL_3:
             # Move right Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BRS_BR1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-70))
            time.sleep(0.5)

            # Move left Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BLS_BL1",math.radians(40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-70))
            time.sleep(0.5)
            # Move both Legs Backward
            self.__run_double_joint_simulation(["JOINT_BRS_BR1","JOINT_BLS_BL1"],math.radians(0),math.radians(0))

             # Move left Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TLS_TL1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-70))
            # Move right Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TRS_TR1",math.radians(40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-70))
            # Move both Legs Forward
            self.__run_double_joint_simulation(["JOINT_TRS_TR1","JOINT_TLS_TL1"],math.radians(0),math.radians(0))


            
        elif self.current_model==self.POSE_MODEL_3_GAP:
            self.__run_double_joint_simulation(["JOINT_TRS_TR1","JOINT_BLS_BL1"],math.radians(70),math.radians(-70))
            self.__run_double_joint_simulation(["JOINT_BRS_BR1","JOINT_TLS_TL1"],math.radians(-70),math.radians(70))
            self.__run_double_joint_simulation(["JOINT_TR1_TR2","JOINT_BL1_BL2"],math.radians(-40),math.radians(-40))
            self.__run_double_joint_simulation(["JOINT_BR1_BR2","JOINT_TL1_TL2"],math.radians(-40),math.radians(-40))
            self.__run_double_joint_simulation(["JOINT_TR2_TR3","JOINT_BL2_BL3"],math.radians(95),math.radians(165))
            self.__run_double_joint_simulation(["JOINT_BR2_BR3","JOINT_TL2_TL3"],math.radians(95),math.radians(165))
            self.__run_double_joint_simulation(["JOINT_TR1_TR2","JOINT_BL1_BL2"],math.radians(-20),math.radians(-20))
            self.__run_double_joint_simulation(["JOINT_BR1_BR2","JOINT_TL1_TL2"],math.radians(-20),math.radians(-20))
            self.__run_double_joint_simulation(["JOINT_BL2_BL3","JOINT_TL2_TL3"],math.radians(120),math.radians(120))
            self.__run_double_joint_simulation(["JOINT_TR2_TR3","JOINT_BR2_BR3"],math.radians(120),math.radians(120))
            
           
        elif self.current_model==self.POSE_MODEL_4:
            self.__run_double_joint_simulation(["JOINT_TR2_TR3","JOINT_BR2_BR3"],math.radians(-60),math.radians(-60))
            self.__run_double_joint_simulation(["JOINT_TRS_TR1","JOINT_BRS_BR1"],math.radians(-30),math.radians(-30))
            self.__run_double_joint_simulation(["JOINT_TR2_TR3","JOINT_BR2_BR3"],math.radians(-90),math.radians(-90))
            self.__run_double_joint_simulation(["JOINT_TRS_TR1","JOINT_BRS_BR1"],math.radians(0),math.radians(0))
            self.__run_double_joint_simulation(["JOINT_TL2_TL3","JOINT_BL2_BL3"],math.radians(-60),math.radians(-60))
            self.__run_double_joint_simulation(["JOINT_TLS_TL1","JOINT_BLS_BL1"],math.radians(-30),math.radians(-30))
            self.__run_double_joint_simulation(["JOINT_TL2_TL3","JOINT_BL2_BL3"],math.radians(-90),math.radians(-90))
            self.__run_double_joint_simulation(["JOINT_TLS_TL1","JOINT_BLS_BL1"],math.radians(0),math.radians(0))
            

    def right_movement(self):
        if self.current_model==self.POSE_MODEL_1:
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TLS_TL1",math.radians(-60),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-70))
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BLS_BL1",math.radians(-60),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-70))
            time.sleep(0.5)
            self.__run_double_joint_simulation(["JOINT_BLS_BL1","JOINT_TLS_TL1"],math.radians(0),math.radians(0))
       
        elif self.current_model==self.POSE_MODEL_3 or self.current_model==self.POSE_MODEL_3_GAP:
            # Move right Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BRS_BR1",math.radians(40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-70))
            time.sleep(0.5)

            # Move left Bottom Leg Forward
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BLS_BL1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BL1_BL2",math.radians(-70))
            time.sleep(0.5)
            # Move both Legs Backward
            self.__run_double_joint_simulation(["JOINT_BRS_BR1","JOINT_BLS_BL1"],math.radians(0),math.radians(0))
    def left_movement(self):
        if self.current_model==self.POSE_MODEL_1:
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TRS_TR1",math.radians(-60),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-70))
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_BRS_BR1",math.radians(-60),duration=0.2)
            self.__run_single_joint_simulation("JOINT_BR1_BR2",math.radians(-70))
            time.sleep(0.5)
            self.__run_double_joint_simulation(["JOINT_BRS_BR1","JOINT_TRS_TR1"],math.radians(0),math.radians(0))
        elif self.current_model==self.POSE_MODEL_3 or self.current_model==self.POSE_MODEL_3_GAP:
             # Move left Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TLS_TL1",math.radians(40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TL1_TL2",math.radians(-70))
            # Move right Top Leg Forward
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-90))
            self.__run_single_joint_simulation("JOINT_TRS_TR1",math.radians(-40),duration=0.2)
            self.__run_single_joint_simulation("JOINT_TR1_TR2",math.radians(-70))
            # Move both Legs Forward
            self.__run_double_joint_simulation(["JOINT_TRS_TR1","JOINT_TLS_TL1"],math.radians(0),math.radians(0))
        
        
    def move_robot(self,movement):
        if movement==self.MOVE_FORWARD:
            self.forward_movement()
        elif movement==self.MOVE_RIGHT:
            self.right_movement()
        elif movement==self.MOVE_LEFT:
            self.left_movement()