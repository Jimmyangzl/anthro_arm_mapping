#!/usr/bin/python3
import numpy as np
import anthro_func.fk as fk
import rclpy
from rclpy.node import Node
from anthro_func.funcs import get_parser
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState # type: ignore


class InteractionNode(Node):
    def __init__(self, args_parser):
        super().__init__('interaction_detect_node')
        self.parser_args = args_parser.parse_args()
    
    def set_args(self):
        pass

    def node_init(self):
        self.franka_state_sub = self.create_subscription(FrankaState,                     
            "/franka_state_controller/franka_states", self.interactionCallback, 1)
        self.next_position_sub = self.create_subscription(Float64MultiArray,                     
            "/pose_next", self.poseUpdateCallback, 1)
        self.interaction_info_pub = self.create_publisher(Float64MultiArray,
            '/interaction_info', 10)
        self.robot = fk.load_robot_model('panda')
        self.t_next = -np.ones(3)
        self.R_next = np.eye(3)
        self.interaction_flag = 0
        self.q_itt = np.zeros(7)
        # Position of EE and F_ext when translational interaction happened
        self.t_itt = np.zeros(3)
        self.F_ext_unit_itt = np.zeros(3)
        self.F_ext_thd = 10
        self.t_next = -np.ones(3)
        self.tag_translation = 0
        # Orientation axis of EE and M_ext when rotational interaction happened
        self.R_itt = np.eye(3)
        self.M_ext_unit_itt = np.zeros(3)
        self.M_ext_thd = 5
        self.R_next = np.eye(3)
        self.tag_rotation = 0
        self.get_logger().info(f"Monitering interaction...")
    
    def interactionCallback(self, msg):
        # Current F_ext and M_ext
        F_ext = -np.array(msg.O_F_ext_hat_K[:3])
        F_ext_size = np.linalg.norm(F_ext)
        M_ext = -np.array(msg.O_F_ext_hat_K[3:6])
        M_ext_size = np.linalg.norm(M_ext)
        # Check if F_ext large enough
        if F_ext_size > self.F_ext_thd:
            self.interaction_flag = 1
            self.tag_translation = 1
            self.F_ext_unit_itt = F_ext / F_ext_size
            # Set joint configuration and get the current wrist position
            self.robot.set_configuration(np.array(msg.q))
            self.t_ittt_itt = self.robot.get_ith_pose(7)[:3, -1]
            self.t_ittq_itt = np.array(msg.q)
            self.get_logger().info(f"Force exceeded threshold.")
        # Check if M_ext large enough
        if M_ext_size > self.M_ext_thd:
            self.interaction_flag = 1
            self.tag_rotation = 1
            self.M_ext_unit_itt = M_ext / M_ext_size
            # Set joint configuration and get the current wrist position
            self.robot.set_configuration(np.array(msg.q))
            self.R_itt = self.robot.get_ith_pose(7)[:3, :3]
            self.q_itt = np.array(msg.q)
            self.get_logger().info(f"Moment exceeded threshold.")
        # Check if the next move retrieves from translational interaction
        if np.dot(((self.t_next-self.t_itt)/np.linalg.norm(self.t_next-self.t_itt)), self.F_ext_unit_itt) > 0:
            self.tag_translation = 0
        # Check if the next move retrieves from rotational interaction
        if self.tag_rotation == 1:
            R_delta = self.R_next @ self.R_itt.T
            _, a_deltas = np.linalg.eig(R_delta)
            a_delta = a_deltas[:,0].real
            if np.dot(self.M_ext_unit_itt, a_delta) < 0:
                self.tag_rotation = 0
        if self.tag_translation == 0 and self.tag_rotation == 0:
            self.interaction_flag = 0
            self.get_logger().info(f"Recovered.")
        msg_interact_info = Float64MultiArray()
        msg_interact_info.data = np.append(self.q_itt, self.interaction_flag)
        self.interaction_info_pub.publish(msg_interact_info)
        
    def poseUpdateCallback(self, msg):
        pose_next = np.reshape(msg.data, (3,4))
        self.t_next = pose_next[:, -1]
        self.R_next = pose_next[:, :3]
        

def main(args=None):
    args_parser = get_parser()
    rclpy.init(args=args)
    interaction_node = InteractionNode(args_parser)
    interaction_node.set_args()
    interaction_node.node_init()
    rclpy.spin(interaction_node)
    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
