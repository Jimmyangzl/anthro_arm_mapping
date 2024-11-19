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
from multi_mode_control_msgs.msg import JointGoal # type: ignore


class JointsPublisher(Node):
    def __init__(self, args_parser):
        super().__init__('dual_joints_rt_client')
        self.parser_args = args_parser.parse_args()
        
    def set_args(self):
        parser_args_dict = vars(self.parser_args)
        config_path = os.path.join(
            get_package_share_directory('anthro_arm_mapping'),
            'config',
            'config_dual_client.yaml',
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
        self.pub_left_constraint = self.create_publisher(Float64MultiArray,
            getattr(self, "pub_left_constraint_topic"), 10)
        self.sub_left_q = self.create_subscription(Float64MultiArray,
            getattr(self, "sub_left_q_topic"), self.left_joint_callback, 1)
        # self.pub_dual_joints = self.create_publisher(Float64MultiArray,
        #     getattr(self, "pub_dual_joints_topic"), 10)
        self.pub_dual_joints = self.create_publisher(JointGoal,
            getattr(self, "pub_dual_joints_topic"), 10)
        self.sub_vicon = self.create_subscription(ViconFrame,
            getattr(self, "sub_vicon_topic"), self.viconCallback, 1)
        self.robot = fk.load_robot_model(self.robot_model)
        self.get_logger().info(f"Robot info: {self.robot}")
        self.dual_joints = np.array(self.q_init)
        self.vicon_data = np.zeros(48)
    
    def calibrate_frame(self):
        self.get_logger().info(f"Waiting for frame calibration...")
        flag_calibrated = False
        while not flag_calibrated:
            if any(self.vicon_data):
                self.right_R_ssr_const, self.right_R_wwr_const, self.right_length_scaler, self.left_R_ssr_const, self.left_R_wwr_const, self.left_length_scaler = v2c_rt.dual_frame2calibrate_rt(
                    self.vicon_data, self.robot)
                flag_calibrated = True
                self.get_logger().info("Frame calibrated.")
        return True
    
    def start_timer(self):
        timer_period = 1.0 / float(self.freq)
        self.timer = self.create_timer(timer_period, self.joint_pub)
        return True

    def joint_pub(self):
        # q2pub = Float64MultiArray()
        left_constraint_msg = Float64MultiArray()
        right_pose_d, right_swivel_d, left_pose_d, left_swivel_d = v2c_rt.dual_vicon2constraint_rt(
            self.vicon_data, self.right_R_ssr_const, self.right_R_wwr_const, self.right_length_scaler, self.left_R_ssr_const, self.left_R_wwr_const, self.left_length_scaler)
        left_constraint_msg.data = np.append(left_pose_d.flatten(), left_swivel_d)
        self.pub_left_constraint.publish(left_constraint_msg)
        self.dual_joints[:7] = ik_num.ik_numerical(pose_d=right_pose_d, swivel=right_swivel_d, robot=self.robot, q0=self.dual_joints[:7])
        self.dual_joints[7:14] = self.left_joint
        # q2pub.data = self.dual_joints.astype(np.float64).tolist()
        q2pub = JointGoal()
        # q2pub.data = self.dual_joints[:7].astype(np.float64).tolist()
        q2pub.q = self.dual_joints[:7].astype(np.float64).tolist()
        self.pub_dual_joints.publish(q2pub)
            
    def left_joint_callback(self, msg):
        self.left_joint = msg.data
    
    def viconCallback(self, msg):
        '''
        Callback to set vicon_data_ directly from vicon stream data
        '''
        vicon_data_temp = np.zeros(48)
        # Shoulder back
        # Right Shoulder back done
        vicon_data_temp[0] = msg.vicon_subjects[0].subject_segments[7].segment_markers[1].marker_translation.x
        vicon_data_temp[1] = msg.vicon_subjects[0].subject_segments[7].segment_markers[1].marker_translation.y
        vicon_data_temp[2] = msg.vicon_subjects[0].subject_segments[7].segment_markers[1].marker_translation.z
        # Right Shoulder front done
        vicon_data_temp[3] = msg.vicon_subjects[0].subject_segments[7].segment_markers[0].marker_translation.x
        vicon_data_temp[4] = msg.vicon_subjects[0].subject_segments[7].segment_markers[0].marker_translation.y
        vicon_data_temp[5] = msg.vicon_subjects[0].subject_segments[7].segment_markers[0].marker_translation.z
        # Right Shoulder up done
        vicon_data_temp[6] = msg.vicon_subjects[0].subject_segments[7].segment_markers[2].marker_translation.x
        vicon_data_temp[7] = msg.vicon_subjects[0].subject_segments[7].segment_markers[2].marker_translation.y
        vicon_data_temp[8] = msg.vicon_subjects[0].subject_segments[7].segment_markers[2].marker_translation.z
        # Right Elbow lateral done
        vicon_data_temp[9] = msg.vicon_subjects[0].subject_segments[6].segment_markers[1].marker_translation.x
        vicon_data_temp[10] = msg.vicon_subjects[0].subject_segments[6].segment_markers[1].marker_translation.y
        vicon_data_temp[11] = msg.vicon_subjects[0].subject_segments[6].segment_markers[1].marker_translation.z
        # Right Elbow medial done
        vicon_data_temp[12] = msg.vicon_subjects[0].subject_segments[6].segment_markers[0].marker_translation.x
        vicon_data_temp[13] = msg.vicon_subjects[0].subject_segments[6].segment_markers[0].marker_translation.y
        vicon_data_temp[14] = msg.vicon_subjects[0].subject_segments[6].segment_markers[0].marker_translation.z
        # Right Wrist front thumb done
        vicon_data_temp[15] = msg.vicon_subjects[0].subject_segments[0].segment_markers[2].marker_translation.x
        vicon_data_temp[16] = msg.vicon_subjects[0].subject_segments[0].segment_markers[2].marker_translation.y
        vicon_data_temp[17] = msg.vicon_subjects[0].subject_segments[0].segment_markers[2].marker_translation.z
        # Right Wrist back pinky done
        vicon_data_temp[18] = msg.vicon_subjects[0].subject_segments[0].segment_markers[0].marker_translation.x
        vicon_data_temp[19] = msg.vicon_subjects[0].subject_segments[0].segment_markers[0].marker_translation.y
        vicon_data_temp[20] = msg.vicon_subjects[0].subject_segments[0].segment_markers[0].marker_translation.z
        # Right Hand done
        vicon_data_temp[21] = msg.vicon_subjects[0].subject_segments[2].segment_markers[0].marker_translation.x
        vicon_data_temp[22] = msg.vicon_subjects[0].subject_segments[2].segment_markers[0].marker_translation.y
        vicon_data_temp[23] = msg.vicon_subjects[0].subject_segments[2].segment_markers[0].marker_translation.z
        
        # Left Shoulder back done
        vicon_data_temp[24] = msg.vicon_subjects[0].subject_segments[3].segment_markers[0].marker_translation.x
        vicon_data_temp[25] = msg.vicon_subjects[0].subject_segments[3].segment_markers[0].marker_translation.y
        vicon_data_temp[26] = msg.vicon_subjects[0].subject_segments[3].segment_markers[0].marker_translation.z
        # Left Shoulder front done
        vicon_data_temp[27] = msg.vicon_subjects[0].subject_segments[3].segment_markers[2].marker_translation.x
        vicon_data_temp[28] = msg.vicon_subjects[0].subject_segments[3].segment_markers[2].marker_translation.y
        vicon_data_temp[29] = msg.vicon_subjects[0].subject_segments[3].segment_markers[2].marker_translation.z
        # Left Shoulder up done
        vicon_data_temp[30] = msg.vicon_subjects[0].subject_segments[3].segment_markers[1].marker_translation.x
        vicon_data_temp[31] = msg.vicon_subjects[0].subject_segments[3].segment_markers[1].marker_translation.y
        vicon_data_temp[32] = msg.vicon_subjects[0].subject_segments[3].segment_markers[1].marker_translation.z
        # Left Elbow lateral done
        vicon_data_temp[33] = msg.vicon_subjects[0].subject_segments[5].segment_markers[0].marker_translation.x
        vicon_data_temp[34] = msg.vicon_subjects[0].subject_segments[5].segment_markers[0].marker_translation.y
        vicon_data_temp[35] = msg.vicon_subjects[0].subject_segments[5].segment_markers[0].marker_translation.z
        # Left Elbow medial done
        vicon_data_temp[36] = msg.vicon_subjects[0].subject_segments[5].segment_markers[1].marker_translation.x
        vicon_data_temp[37] = msg.vicon_subjects[0].subject_segments[5].segment_markers[1].marker_translation.y
        vicon_data_temp[38] = msg.vicon_subjects[0].subject_segments[5].segment_markers[1].marker_translation.z
        # Left Wrist front thumb done
        vicon_data_temp[39] = msg.vicon_subjects[0].subject_segments[1].segment_markers[0].marker_translation.x
        vicon_data_temp[40] = msg.vicon_subjects[0].subject_segments[1].segment_markers[0].marker_translation.y
        vicon_data_temp[41] = msg.vicon_subjects[0].subject_segments[1].segment_markers[0].marker_translation.z
        # Left Wrist back pinky done
        vicon_data_temp[42] = msg.vicon_subjects[0].subject_segments[1].segment_markers[1].marker_translation.x
        vicon_data_temp[43] = msg.vicon_subjects[0].subject_segments[1].segment_markers[1].marker_translation.y
        vicon_data_temp[44] = msg.vicon_subjects[0].subject_segments[1].segment_markers[1].marker_translation.z
        # Left Hand done
        vicon_data_temp[45] = msg.vicon_subjects[0].subject_segments[4].segment_markers[0].marker_translation.x
        vicon_data_temp[46] = msg.vicon_subjects[0].subject_segments[4].segment_markers[0].marker_translation.y
        vicon_data_temp[47] = msg.vicon_subjects[0].subject_segments[4].segment_markers[0].marker_translation.z
        self.vicon_data = vicon_data_temp


def main(args=None):
    args_parser = get_parser('config_dual_client.yaml')
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
