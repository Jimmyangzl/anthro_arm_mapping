from scipy import optimize as optimize
import numpy as np


def ik_cost(q, pose_d, swivel, robot):
    '''
    Cost function for numerical ik
    q: current configuration
    pose_d: desired pose
    swivel: desired nominal vector of arm plane
    robot: robot DH model
    '''
    robot.set_configuration(q)
    pose_actual = robot.get_ith_pose(7)
    error_pose = pose_d - pose_actual
    k_error_swivel = 0.01
    error_swivel = k_error_swivel*(swivel - robot.get_swivel())
    cost = error_pose[:3, :].flatten()
    cost = np.append(cost, error_swivel)
    return cost

def ik_numerical(pose_d, swivel, robot, q0):
    '''
    Numerical IK 
    pose_d: desired pose
    swivel: desired nominal vector of arm plane
    robot: robot DH model
    q0: initial guess for numerical solution of IK
    '''
    # Optimization using scipy lib to find the IK solution
    optimization_result = optimize.least_squares(
        fun=ik_cost, x0=q0, bounds=robot.joint_limits, args=(pose_d, swivel, robot), method='trf'
    )
    q = q0
    # Success check for optimization
    if optimization_result.success:
        q = optimization_result.x
        robot.set_configuration(q)
        pose_actual = robot.get_ith_pose(7)
        # Pose check for the solution, if pose is not accurate enough, set configuration to initial guess
        if not np.allclose(pose_actual, pose_d, atol=2e-2):
            print("Pose not accurate.")
            q = q0
    return q