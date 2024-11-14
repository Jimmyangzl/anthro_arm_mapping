#!/usr/bin/python
'''
Takes in the vicon data from the given UDP server,
processes the information and publishes it as a ROS message.
'''

# import rospy
import rclpy
from rclpy.node import Node
import socket
import time
import json
# from extrinsic_calibration_suite.msg import ViconFrame, ViconSubject, ViconSegment, ViconMarker
from vicon_msgs.msg import ViconFrame, ViconSubject, ViconSegment, ViconMarker

class ViconUDPtoROS(Node):

	# UDP stuff
	start_msg_bytes = str.encode("start")
	end_msg_bytes = str.encode("end")
	server_address_port = None
	buffer_size = 2048
	UDPClientSocket = None

	# ROS stuff
	publisher_topic = None
	print_once = True
	
	def __init__(self, server_address, server_port,  buffer_size, publisher_topic):
		super().__init__('vicon_udp_ros2')
		self.buffer_size = buffer_size
		self.server_address_port = (server_address, server_port)
		self.UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
		self.ros_publisher = self.create_publisher(ViconFrame, publisher_topic, 1)
		# Send the trigger to receive the vicon data here
		self.UDPClientSocket.sendto(self.start_msg_bytes, self.server_address_port)
		self.recv_loop()
	def recv_loop(self):
		while True:
			# try:
			msg_from_server = self.UDPClientSocket.recvfrom(self.buffer_size)
			if(self.print_once):
				print("Received vicon data! Vicon data should start streaming.")
				self.print_once = False
			ros_msg = ViconFrame()
			# ros_msg.header.stamp = self.get_clock.Time()
			ros_msg.header.frame_id = 'vicon'

			msg = msg_from_server[0].decode('utf-8')
			msg_dict = json.loads(msg)
			for subject_key, subject_value in msg_dict['subjects'].items():
				ros_subject = ViconSubject()
				ros_subject.subject_name.data = subject_key
				for segment_key, segment_value in subject_value.items():
					ros_segment = ViconSegment()
					ros_segment.segment_name.data = segment_key
					for marker_key, marker_value in segment_value.items():
						ros_marker = ViconMarker()
						ros_marker.marker_name.data = marker_key
						# Markers are measured in mm. We are working with meters.
						ros_marker.marker_translation.x = marker_value['x']/1000
						ros_marker.marker_translation.y = marker_value['y']/1000
						ros_marker.marker_translation.z = marker_value['z']/1000
						ros_segment.segment_markers.append(ros_marker)
					pass
					ros_subject.subject_segments.append(ros_segment)
				ros_msg.vicon_subjects.append(ros_subject)
			self.ros_publisher.publish(ros_msg)
			# except:
			# 	print("excpet")
			# 	self.UDPClientSocket.sendto(self.end_msg_bytes, self.server_address_port)
			# 	break

def main(args=None):
    rclpy.init(args=args)
    vicon_udp_to_ros_node = ViconUDPtoROS("10.157.175.68", 20001, 8192, "vicon_frame")
    # vicon_udp_to_ros_node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
	main()	
	# no spin should be required