import numpy as np
import anthro_func.vicon2config_rt as v2c_rt
import anthro_func.fk as fk
import anthro_func.ik_num_anthro as ik_num
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from anthro_func.funcs import get_parser
from vicon_msgs.msg import ViconFrame # type: ignore


class JointsPublisher(Node):
    def __init__(self, args_parser):
        super().__init__('joints_rt_pub_node')
        self.parser_args = args_parser.parse_args()
        
    def set_args(self):
        parser_args_dict = vars(self.parser_args)
        config_path = os.path.join(
            get_package_share_directory('anthro_arm_mapping'),
            'config',
            'config.yaml',
        )
        with open(config_path, 'r') as file:
            configs = yaml.safe_load(file)
        for key, value in configs.items():
            if parser_args_dict[key]:
                value = parser_args_dict[key]
            setattr(self, key, value)
            self.get_logger().info(key + ': ' + str(value))
        return True
    
    def node_init(self):
        self.joints_pub = self.create_publisher(Float64MultiArray,
            getattr(self, "joints_pub_topic"), 10)
        self.vicon_sub = self.create_subscription(ViconFrame,                     
            getattr(self, "vicon_sub_topic"), self.viconCallback, 1)
        self.interact_sub = self.create_subscription(Float64MultiArray,                     
            getattr(self, "interaction_module_sub_topic"), self.interaction_callback, 1)
        self.pose_next_pub = self.create_publisher(Float64MultiArray, 
            getattr(self, "pose_next_pub_topic"), 10)
        self.robot = fk.load_robot_model(self.robot_model)
        self.get_logger().info(f"Robot info: {self.robot}")
        self.joint = np.array(self.q_init)
        self.vicon_data = np.zeros(24)
    
    def calibrate_frame(self):
        self.get_logger().info(f"Waiting for frame calibration...")
        flag_calibrated = False
        while not flag_calibrated:
            if any(self.vicon_data):
                self.R_ssr_const, self.R_wwr_const, self.length_scaler = v2c_rt.frame2calibrate_rt(
                    self.vicon_data, self.robot)
                flag_calibrated = True
                self.get_logger().info("Frame calibrated.")
        return True
    
    def start_timer(self):
        timer_period = 1.0 / float(self.freq)
        self.timer = self.create_timer(timer_period, self.joint_pub)
        return True

    def joint_pub(self):
        q2pub = Float64MultiArray()
        pose_next_msg = Float64MultiArray()
        pose_d, swivel_d = v2c_rt.vicon2constraint_rt(self.vicon_data, self.R_ssr_const, self.R_wwr_const, self.length_scaler)
        if self.interaction_flag == 1:
            pose_next_msg.data = np.reshape(pose_d[:3, :], 12)
            self.pose_next_pub.publish(pose_next_msg)
            self.robot.set_configuration(self.q_itt)
            pose_d = self.robot.get_ith_pose(7)
        self.joint = ik_num.ik_numerical(pose_d=pose_d, swivel=swivel_d, robot=self.robot, q0=self.joint)
        q2pub.data = self.joint.astype(np.float64).tolist()
        self.joints_pub.publish(q2pub)
            
    def interaction_callback(self, msg):
        self.q_itt = msg.data[:7]
        self.interaction_flag = msg.data[7]
    
    def viconCallback(self, msg):
        '''
        Callback to set vicon_data_ directly from vicon stream data
        '''
        vicon_data_temp = np.zeros(24)
        # Shoulder back
        vicon_data_temp[0] = msg.vicon_subjects[0].subject_segments[0].segment_markers[0].marker_translation.x
        vicon_data_temp[1] = msg.vicon_subjects[0].subject_segments[0].segment_markers[0].marker_translation.y
        vicon_data_temp[2] = msg.vicon_subjects[0].subject_segments[0].segment_markers[0].marker_translation.z
        # Shoulder front
        vicon_data_temp[3] = msg.vicon_subjects[0].subject_segments[0].segment_markers[1].marker_translation.x
        vicon_data_temp[4] = msg.vicon_subjects[0].subject_segments[0].segment_markers[1].marker_translation.y
        vicon_data_temp[5] = msg.vicon_subjects[0].subject_segments[0].segment_markers[1].marker_translation.z
        # Shoulder up
        vicon_data_temp[6] = msg.vicon_subjects[0].subject_segments[0].segment_markers[2].marker_translation.x
        vicon_data_temp[7] = msg.vicon_subjects[0].subject_segments[0].segment_markers[2].marker_translation.y
        vicon_data_temp[8] = msg.vicon_subjects[0].subject_segments[0].segment_markers[2].marker_translation.z
        # Elbow lateral
        vicon_data_temp[9] = msg.vicon_subjects[0].subject_segments[1].segment_markers[0].marker_translation.x
        vicon_data_temp[10] = msg.vicon_subjects[0].subject_segments[1].segment_markers[0].marker_translation.y
        vicon_data_temp[11] = msg.vicon_subjects[0].subject_segments[1].segment_markers[0].marker_translation.z
        # Elbow medial
        vicon_data_temp[12] = msg.vicon_subjects[0].subject_segments[1].segment_markers[1].marker_translation.x
        vicon_data_temp[13] = msg.vicon_subjects[0].subject_segments[1].segment_markers[1].marker_translation.y
        vicon_data_temp[14] = msg.vicon_subjects[0].subject_segments[1].segment_markers[1].marker_translation.z
        # Wrist front thumb
        vicon_data_temp[15] = msg.vicon_subjects[0].subject_segments[2].segment_markers[1].marker_translation.x
        vicon_data_temp[16] = msg.vicon_subjects[0].subject_segments[2].segment_markers[1].marker_translation.y
        vicon_data_temp[17] = msg.vicon_subjects[0].subject_segments[2].segment_markers[1].marker_translation.z
        # Wrist back pinky
        vicon_data_temp[18] = msg.vicon_subjects[0].subject_segments[2].segment_markers[2].marker_translation.x
        vicon_data_temp[19] = msg.vicon_subjects[0].subject_segments[2].segment_markers[2].marker_translation.y
        vicon_data_temp[20] = msg.vicon_subjects[0].subject_segments[2].segment_markers[2].marker_translation.z
        # Hand
        vicon_data_temp[21] = msg.vicon_subjects[0].subject_segments[3].segment_markers[0].marker_translation.x
        vicon_data_temp[22] = msg.vicon_subjects[0].subject_segments[3].segment_markers[0].marker_translation.y
        vicon_data_temp[23] = msg.vicon_subjects[0].subject_segments[3].segment_markers[0].marker_translation.z
        self.vicon_data = vicon_data_temp


def main(args=None):
    args_parser = get_parser('config.yaml')
    rclpy.init(args=args)
    joints_pub_node = JointsPublisher(args_parser)
    joints_pub_node.set_args()
    joints_pub_node.node_init()
    rclpy.spin_once(joints_pub_node)
    joints_pub_node.calibrate_frame()
    joints_pub_node.start_timer()
    rclpy.spin(joints_pub_node)
    joints_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
