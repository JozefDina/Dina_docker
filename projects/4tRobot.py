import pybullet as pb
import pybullet_data
import time
import numpy as np

class Env():
    def __init__(self):
        super(Env, self).__init__()

        #cennect GUI
        pb.connect(pb.GUI)
        pb.setGravity(0, 0, -9.8)
        
    def robot_model(self):
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = pb.loadURDF("plane.urdf")
        self.robot_id = pb.loadURDF("franka_panda/panda.urdf", basePosition = [0,0,0],useFixedBase = True)

        for joint_number in range (pb.getNumJoints(self.robot_id)):
            pb.resetJointState(self.robot_id, joint_number, 0)

    def move_joint(self, robot_id, joint_index, target_angle):
        pb.setJointMotorControl2(bodyUniqueId=robot_id,
                                 jointIndex=joint_index,
                                 controlMode=pb.POSITION_CONTROL,
                                 targetPosition=target_angle)
        
env=Env()

if __name__=="__main__":
    env.robot_model()
    print(env.robot_id)

    joint_angles = [0,0,0,np.deg2rad(-45),0,np.deg2rad(90),0]
    joint_index = [0,1,2,3,4,5,6]

    num_timestep = 10000
    num_dof = 7
    for t in range(num_timestep):
        for joint_idx, angle in enumerate(joint_angles):
            env.move_joint(env.robot_id, joint_idx, angle)
        pb.stepSimulation()
        time.sleep(1/240)
    pb.disconnect()