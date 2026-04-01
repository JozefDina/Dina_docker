import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
 
if __name__ == '__main__':
    
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.setTimeStep(0.01)
    
    plane_id = p.loadURDF("plane.urdf")
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase = True)
    
    joint_indices = [i for i in range(7)]
    ee_link_index = 6
    
    radius = 0.15
    steps = 2000
    speed = 0.5
    Kp = 20.0
    Ki = 5.0
    damping = 1
    
    
    initial_target = [0.5 + radius,0.0,0.5]
    ik_solution = p.calculateInverseKinematics(kuka_id, ee_link_index, initial_target)
    
    for i, joint_index in enumerate(joint_indices):
        p.resetJointState(kuka_id, joint_index, ik_solution[i])
        p.setJointMotorControl2(kuka_id, joint_index, p.POSITION_CONTROL, targetPosition = ik_solution[i])
        
    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)
    center = np.array(initial_target)
    z_const = center[2]
    
    q = np.array([p.getJointState(kuka_id, i)[0] for i in joint_indices])
    
    ee_positions = []
    desired_positions = []
    integral_error = 0.0
    dt = 0.01
    
    for step in range(steps):
        t = step * dt
        angle = speed * t
        
        xd = center + radius * np.array([np.cos(angle), np.sin(angle), 0])
        xd[2] = z_const
        v_des = radius * speed * np.array([-np.sin(angle), np.cos(angle), 0])
        
        ee_state = p.getLinkState(kuka_id, ee_link_index, computeForwardKinematics = True)
        x_act = np.array(ee_state[4])
        pos_error = xd - x_act
        integral_error += pos_error *dt
        v_cmd = v_des + Kp*pos_error + Ki * integral_error
        
        dq_zero = [0.0] * len(joint_indices)
        jac_t, _ = p.calculateJacobian(kuka_id, ee_link_index, [0,0,0], q.tolist(), dq_zero, dq_zero)
        J = np.array(jac_t)
        
        JJT = J @ J.T
        J_pinv = J.T @ np.linalg.inv(JJT + damping**2*np.eye(JJT.shape[0]))
        
        dq = J_pinv @ v_cmd
        dq = np.clip(dq, -5, 5)
        q = q + dq * dt
        
        for i, qi in zip (joint_indices, dq):
            p.setJointMotorControl2(kuka_id, i, controlMode = p.VELOCITY_CONTROL, targetVelocity = qi)
            
        ee_positions.append(x_act.copy())
        desired_positions.append(xd.copy())
        
        p.stepSimulation()
        time.sleep(dt)
        
    p.disconnect()
    ee_positions = np.array(ee_positions)
    desired_positions = np.array(desired_positions)
    
    plt.figure(figsize=(6,6))
    plt.plot(ee_positions, ee_positions[:,1], '--', label = 'Actual Path')
    plt.plot(desired_positions, desired_positions[:,1], '--', label = 'Desired Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.title('End efector trajectory')
    plt.grid(True)
    plt.tight_layout()
    plt.show()
 
 