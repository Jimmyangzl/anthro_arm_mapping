import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_robot_model(robot_name):
    '''
    Return the robot model of class RobotFK with the parameters in yaml file
    '''
    robot_model_path = os.path.join(
        get_package_share_directory('anthro_arm_mapping'),
        'config',
        'robot_models.yaml'
    )
    with open(robot_model_path, 'r') as yaml_file:
        robot_data = yaml.safe_load(yaml_file)
    return RobotFK(np.array(robot_data[robot_name]['robot_model']), np.array(
        robot_data[robot_name]['robot_joint_limit']), robot_data[robot_name]['length_robot'])

def dh_transform(dh_parameters):
    '''
    Return the dh transform matrix T regarding dh parameters
    dh_parameters: alpha - a - d - theta, np.array(4)
    '''
    # extract the parameters
    alpha = dh_parameters[0]
    a = dh_parameters[1]
    d = dh_parameters[2]
    theta = dh_parameters[3]

    T = np.array([
        [np.cos(theta), - np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), - np.sin(alpha), - np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return T


class RobotFK:
    '''
    Class for computing robot forward kinematic and required parameters
    '''
    def __init__(self, robot_model, joints_limit, robot_length):
        # Robot DH model
        self.model = robot_model
        # Set the joint limit of robot
        self.joint_limits = joints_limit
        # Robot arm length for length scaling
        self.robot_length = robot_length
    def __repr__(self):
        return "Robot DH Table:\n%s" % self.model + "\nRobot Joint Limits:\n%s" % self.joint_limits + "\nRobot Length:\n%s" % self.robot_length
    
    def set_configuration(self, q):
        '''
        Set the configuration theta
        q: new configuration, np.array()
        '''
        self.model[:, -1] = q
        # Check if configuration exceeds joint limits
        for i in range(7):
            self.model[i, -1] = np.maximum(self.joint_limits[0, i], self.model[i, -1])
            self.model[i, -1] = np.minimum(self.joint_limits[1, i], self.model[i, -1])

    def get_ith_pose(self, i):
        '''
        Return the pose of the i th joint
        '''
        pose = np.eye(4)
        for j in range(i):
            pose = pose @ dh_transform(self.model[j, :])
        return pose

    def get_swivel(self):
        '''
        Compute the nominal vector of arm plane
        Nominal vector of arm plane: the vector from shoulder to elbow cross the vector from elbow to wrist
        '''
        # Position of shoulder
        pose_s = self.get_ith_pose(1)
        p_s = pose_s[:3, 3]
        # Position of elbow, it is more roboust to use the pose of the 3rd frame to compute n_arm
        pose_e = self.get_ith_pose(3)
        p_e = pose_e[:3, 3]
        # Position of wrist
        pose_w = self.get_ith_pose(7)
        p_w = pose_w[:3, 3]
        # Vector from shoulder to wrist
        p_sw = p_w - p_s
        # Vector from shoulder to elbow
        p_se = p_e - p_s
        # Normalized normal vector of arm plane
        n_arm = np.cross(p_se, p_sw)
        n_arm = n_arm / np.linalg.norm(n_arm) 
        return n_arm
        
    def get_swivel_angle(self):
        # Position of shoulder
        pose_s = self.get_ith_pose(1)
        p_s = pose_s[:3, 3]
        # Position of elbow, it is more roboust to use the pose of the 3rd frame to compute n_arm
        pose_e = self.get_ith_pose(4)
        p_e = pose_e[:3, 3]
        # Position of wrist
        pose_w = self.get_ith_pose(7)
        p_w = pose_w[:3, 3]
        # Vector from shoulder to wrist
        p_sw = p_w - p_s
        # Vector from shoulder to elbow
        p_se = p_e - p_s
        # Normalized normal vector of arm plane
        n_arm = np.cross(p_se, p_sw)
        n_arm = n_arm / np.linalg.norm(n_arm)
        '''
        Swivel Angle
        '''
        # Normalized normal vector of vertical plane
        n_v = np.cross(np.array([0.0, -1.0, 0.0]), p_sw)
        n_v = n_v / np.linalg.norm(n_v)
        # swivel angle
        # swivel_angle = np.arctan2(-n_v @ n_arm, np.dot(n_v, n_arm))
        sign_swivel = -1
        if n_arm[1] < 0:
           sign_swivel = 1
        swivel_angle = sign_swivel * np.arccos(np.minimum(np.dot(n_v, n_arm), 1.0))
        return swivel_angle






