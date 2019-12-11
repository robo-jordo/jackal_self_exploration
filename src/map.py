#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

class MapImage:

	map_width = 0
	map_height = 0

	current_x = 0
	current_y = 0

	map_x_origin = 0
	map_y_origin = 0

	x_in_map = 0
	y_in_map = 0 

	resolution = 0

	def map_callback(self, data):
		map_data = np.array(data.data)
		if (self.map_width*self.map_height != 0):
			map_data = np.reshape(map_data, (self.map_height, self.map_width))
			map_data[map_data == -1] = 150
			map_data[map_data == 100] = 255
			for i in range(20):
				for j in range(20):
					map_data[int(self.y_in_map)+i,int(self.x_in_map)+j] = 255
			im = np.array(map_data, dtype = np.uint8)
			img = cv2.resize(im,(640,640))
			horizontal_img = cv2.flip( img, 0 )
			# cv2.imshow('image',horizontal_img)
			# cv2.waitKey(2)

	def meta_callback(self, data):
		self.map_width = data.width
		self.map_height = data.height
		self.map_x_origin = data.origin.position.x
		self.map_y_origin = data.origin.position.y
		self.resolution = data.resolution

	def odom_callback(self, data):
		if data.child_frame_id == "base_link":
			self.current_x = data.pose.pose.position.x
			self.current_y = data.pose.pose.position.y

			self.x_in_map = (self.current_x - self.map_x_origin)/self.resolution
			self.y_in_map = (self.current_y - self.map_y_origin)/self.resolution


	def listener(self):

		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('listener', anonymous=True)

		rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
		rospy.Subscriber("/map_metadata", MapMetaData, self.meta_callback)
		rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

if __name__ == '__main__':
	mi = MapImage()
	mi.listener()
