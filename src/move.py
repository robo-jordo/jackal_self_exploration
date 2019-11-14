#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion


class Movement:
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	x_delt = 0
	theta_delt = 0

	stored_x = 0
	stored_y = 0
	stored_theta = 0

	new_twist = Twist()

	angles = [-3.14,-2.355,-1.57,-0.785,0,0.785,1.57,2.355,3.14]

	def callback(self, data):
		orientation_q = data.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
		self.x_pos = data.pose.pose.position.x
		self.y_pos = data.pose.pose.position.y
		self.z_pos = data.pose.pose.position.z

	def _rounder(self, number):
		a = [abs(x - number) for x in self.angles]
		index = np.argmin(np.array([a]))
		return self.angles[index]


	def move(self, direction):

		self.stored_theta = self._rounder(self.yaw)

		self.stored_x = self.x_pos
		self.stored_y = self.y_pos

		self.direc = 1

		self.new_twist.angular.z = 0
		self.new_twist.linear.x = 0

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
			self.target = self.stored_x + 1
			self.direc = 1
		if direction == "b":
			self.direc = -1
			self.target = self.stored_x - 1


		if (direction == "cw" or direction == "ccw"):
			while(abs(self.yaw - self.target) > (0.005)):
				print(str(self.target) + " : " + str(self.yaw))
				self.new_twist.angular.z = 5 * (self.target-self.yaw)
				self.pub.publish(self.new_twist)
				rospy.sleep(0.05)

		if (direction == "f" or direction == "b"):
			while((math.sqrt((self.stored_x-self.x_pos)**2 + (self.stored_y-self.y_pos)**2)) < (1)):
				#print(str(self.target) + " : " + str(self.x_pos))
				self.new_twist.linear.x = self.direc * 1 * (2 - math.sqrt((self.stored_x-self.x_pos)**2 + (self.stored_y-self.y_pos)**2))
				self.pub.publish(self.new_twist)
				rospy.sleep(0.05)





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
			
