import numpy as np
# import anthro_func.vicon2config_rt as v2c_rt
import anthro_func.fk as fk
import anthro_func.ik_num_anthro as ik_num
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from anthro_func.funcs import get_parser


class JointsPublisher(Node):
    def __init__(self, args_parser):
        super().__init__('left_joint_rt_server')
        self.parser_args = args_parser.parse_args()
        
    def set_args(self):
        parser_args_dict = vars(self.parser_args)
        config_path = os.path.join(
            get_package_share_directory('anthro_arm_mapping'),
            'config',
            'config_left_joint_server.yaml',
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
        self.pub_left_q = self.create_publisher(Float64MultiArray,
            getattr(self, "pub_left_q_topic"), 10)
        self.sub_left_constraint = self.create_subscription(Float64MultiArray,
            getattr(self, "sub_left_constraint_topic"), self.constraintCallback, 1)
        self.robot = fk.load_robot_model(self.robot_model)
        self.get_logger().info(f"Robot info: {self.robot}")
        self.left_joint = np.array(self.q_init)
        self.left_pose_d = np.zeros((4,4))
        self.left_swivel_d = np.zeros(3)
            
    def constraintCallback(self, msg):
        '''
        Callback to set vicon_data_ directly from vicon stream data
        '''
        self.left_pose_d[0, :] = msg.data[:4]
        self.left_pose_d[1, :] = msg.data[4:8]
        self.left_pose_d[2, :] = msg.data[8:12]
        self.left_pose_d[3, :] = msg.data[12:16]
        self.left_swivel_d[:3] = msg.data[16:19]
        msg_left_q = Float64MultiArray()
        self.left_joint = ik_num.ik_numerical(
            pose_d=self.left_pose_d, swivel=self.left_swivel_d, robot=self.robot, q0=self.left_joint)
        msg_left_q.data = self.left_joint.astype(np.float64).tolist()
        self.pub_left_q.publish(msg_left_q)


def main(args=None):
    args_parser = get_parser('config_left_joint_server.yaml')
    rclpy.init(args=args)
    joints_pub_node = JointsPublisher(args_parser)
    joints_pub_node.set_args()
    joints_pub_node.node_init()
    rclpy.spin(joints_pub_node)
    joints_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
