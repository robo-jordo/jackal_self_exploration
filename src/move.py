#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion


class Movement:
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	scan_min = 0

	x_delt = 0
	theta_delt = 0

	stored_x = 0
	stored_y = 0
	stored_theta = 0

	new_twist = Twist()

	angles = [-3.14,-2.355,-1.57,-0.785,0,0.785,1.57,2.355,3.14]
	directions = {"N":1.57,"NE":0.785,"E":0,"SE":-0.785,"S":-1.57,"SW":-2.355,"W":3.14,"NW":2.355}

	def _odom_callback(self, data):
		orientation_q = data.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
		self.x_pos = data.pose.pose.position.x
		self.y_pos = data.pose.pose.position.y
		self.z_pos = data.pose.pose.position.z

	def _front_laser_callback(self, data):
		self.scan_min =  np.min(np.array([data.ranges]))
		# print(scan)

	def _rounder(self, number):
		a = [abs(x - number) for x in self.angles]
		index = np.argmin(np.array([a]))
		return self.angles[index]

	def _lidar_object_check(self):
		if self.scan_min < 1.2:
			return True
		else:
			return False
		

	def relative_move(self, direction):
		self.stored_theta = self._rounder(self.yaw)

		self.stored_x = self.x_pos
		self.stored_y = self.y_pos

		self.direc = 1

		self.new_twist.angular.z = 0
		self.new_twist.linear.x = 0

		self.target = self.directions[direction]

		if direction == "ccw":
			if ((self.stored_theta - 0.785)>=-3.14):
				self.target = self.stored_theta - 0.785
			else:
				self.target = 5.495 + self.stored_theta
		if direction == "cw":
			if ((self.stored_theta + 0.785)<=3.14):
				self.target = self.stored_theta + 0.785
			else:
				self.target = -5.495 + self.stored_theta

		if direction == "f":
			# self.target = self.stored_x + 1
			self.direc = 1
		if direction == "b":
			self.direc = -1
			# self.target = self.stored_x - 1

		if (direction == "cw" or direction == "ccw"):
			while(abs(self.yaw - self.target) > (0.005)):
				print(str(self.target) + " : " + str(self.yaw))
				self.new_twist.angular.z = 3 * (self.target-self.yaw)
				self.pub.publish(self.new_twist)
				rospy.sleep(0.05)

		if (direction == "f" or direction == "b"):
			while((math.sqrt((self.stored_x-self.x_pos)**2 + (self.stored_y-self.y_pos)**2)) < (1)):
				print(str(self.target) + " : " + str(self.x_pos))
				self.new_twist.linear.x = self.direc * 1 * (1.2 - math.sqrt((self.stored_x-self.x_pos)**2 + (self.stored_y-self.y_pos)**2))
				self.pub.publish(self.new_twist)
				rospy.sleep(0.05)


	def move(self, direction):

		self.stored_x = self.x_pos
		self.stored_y = self.y_pos

		self.direc = 1

		self.new_twist.angular.z = 0
		self.new_twist.linear.x = 0

		self.target = self.directions[direction]

		# Turn to correct orientation
		while(abs(self.yaw - self.target) > (0.005)):
			print(str(self.target) + " , " + str(self.yaw))
			sys.stdout.write("\033[F")
			self.new_twist.angular.z = 3 * (self.target-self.yaw)
			self.pub.publish(self.new_twist)
			rospy.sleep(0.05)
		self.new_twist.angular.z = 0
		self.new_twist.linear.x = 0
		self.pub.publish(self.new_twist)

		# Check for object
		if (self._lidar_object_check()):
			# Dont move if there is an obstacle
			print("")
			print("object")
			return -1 
		else:
			# Move forward if there isnt an obstacle 
			while((math.sqrt((self.stored_x-self.x_pos)**2 + (self.stored_y-self.y_pos)**2)) < (0.5)):
				print(str(self.target) + " : " + str(self.x_pos))
				sys.stdout.write("\r")
				self.new_twist.linear.x = self.direc * 1 * (1.2 - math.sqrt((self.stored_x-self.x_pos)**2 + (self.stored_y-self.y_pos)**2))
				self.pub.publish(self.new_twist)
				rospy.sleep(0.05)
			self.new_twist.angular.z = 0
			self.new_twist.linear.x = 0
			self.pub.publish(self.new_twist)
			return 1


	def listener(self):

		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('Mover', anonymous=True)

		rospy.Subscriber("/odometry/filtered", Odometry, self._odom_callback)
		rospy.Subscriber("/oct4", LaserScan, self._front_laser_callback)
		

		# spin() simply keeps python from exiting until this node is stopped
		#rospy.spin()

if __name__ == '__main__':
	new_movement = Movement()
	new_movement.listener()
	while not rospy.is_shutdown():
			txt = raw_input("?")
			new_movement.move(txt)
			
