#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped


class Movement:
	pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

	x_pos = 0
	y_pos = 0
	z_pos = 0
	x_ang = 0
	y_ang = 0
	z_ang = 0

	x_delt = 0
	y_delt = 0
	theta_delt = 0

	new_pose = PoseStamped()

	def callback(self, data):
		self.x_pos = data.pose.pose.position.x
		self.y_pos = data.pose.pose.position.y
		self.z_pos = data.pose.pose.position.z
		self.x_ang = data.pose.pose.orientation.x
		self.y_ang = data.pose.pose.orientation.y
		self.z_ang = data.pose.pose.orientation.z

	def move(self, direction):
		self.theta_delt = 0
		self.x_delt = 0
		self.y_delt = 0
		if direction == "up":
			self.y_delt = 1
		if direction == "down":
			self.y_delt = -1
		if direction == "left":
			self.x_delt = -1
		if direction == "right":
			self.x_delt = 1
		if direction == "cw":
			self.theta_delt = -1.57
		if direction == "c":
			self.theta_delt = 1.57

		new_pose = PoseStamped()
		current_time = rospy.Time.now()
		self.new_pose.header.stamp = current_time
		self.new_pose.header.frame_id = "odom"
		self.new_pose.pose.position.x = self.x_pos + self.x_delt
		self.new_pose.pose.position.y = self.y_pos + self.y_delt
		self.new_pose.pose.position.z = self.z_pos
		self.new_pose.pose.orientation.x = self.x_ang
		self.new_pose.pose.orientation.y = self.y_ang
		self.new_pose.pose.orientation.z = self.z_ang + self.theta_delt
		
		self.pub.publish(self.new_pose)


	def listener(self):

		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('Mover', anonymous=True)

		rospy.Subscriber("/odometry/filtered", Odometry, self.callback)
		

		# spin() simply keeps python from exiting until this node is stopped
		#rospy.spin()

if __name__ == '__main__':
	new_movement = Movement()
	new_movement.listener()
	while not rospy.is_shutdown():
			txt = raw_input("?")
			new_movement.move(txt)
			
