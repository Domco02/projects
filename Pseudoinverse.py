import pybullet as p
import pybullet_data
import numpy as np
import os
import time
 
 
def forward_kinematics(theta):
    theta1, theta2 = theta
    x=link_length[0] *np.cos(theta1) + link_length[1] * np.cos(theta1 + theta2)
    y=link_length[0] *np.sin(theta1) + link_length[1] * np.sin(theta1 + theta2)
    return np.array([x, y])
 
def jacobian(theta):
    theta1, theta2 = theta
    l1, l2 = link_length
    j11 = -l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2)
    j12 = -l2 * np.sin(theta1 + theta2)
    j21 = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    j22 = l2 * np.cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])
 
def inverse_kinematics(target, theta_init, step=100):
    theta=np.array(theta_init)
    
    for _ in range(step):
        pos=forward_kinematics(theta)
        err= target - pos
        if np.linalg.norm(err) < 1e-3:
            break
        J = jacobian(theta)
        dtheta = np.linalg.pinv(J) @ err
        theta += dtheta
    return theta
 
if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
 
    p.resetSimulation()
    p.setGravity(0,0,-9.81)
    
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("urdf/2dof_planar2.urdf", [0,0,0], useFixedBase=True)
    
    joint_indices= [0,1]
    link_length= [1,1]
    
    target_pos = [1.2,1.2]
    targe_id = p.loadURDF("sphere_small.urdf", [target_pos[0], target_pos[1], 0], globalScaling = 0.1)
    
    theta_guess = [0.1,0.1]
    
    theta_solution = inverse_kinematics(target_pos, theta_guess)
    theta_solution = (theta_solution + np.pi)%(2*np.pi) - np.pi
    
    for i, angle in enumerate(theta_solution):
        p.resetJointState(robot_id, joint_indices[i], angle)
        
    target_xyz = [1.2,1.2,0]
    ik_solution = p.calculateInverseKinematics(robot_id, 2, target_xyz)
    print("calculate solution:", np.rad2deg(theta_solution[0]), np.rad2deg(theta_solution[1]))
    print("simulation solution:", np.rad2deg(ik_solution))
    
    final_pos = p.getLinkState(robot_id,2)[4]
    print("acheived pos", final_pos)
    print("target positon", target_pos)
    
    while True:
        p.stepSimulation()
        time.sleep(1/240)
         