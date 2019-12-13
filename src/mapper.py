#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

class MapImage:

	def __init__(self):
		self.listener()

	# Class attributes required for the calculations to create the image
	map_width = 0
	map_height = 0

	current_x = 0
	current_y = 0

	map_x_origin = 0
	map_y_origin = 0

	x_in_map = 0
	y_in_map = 0 

	resolution = 0

	horizontal_img = np.array([])


	def map_callback(self, data):
		""" Callback to get map data into numpy array (horizontal_img) and shows image if 
		script is run as main.

		Args:
			data: ros messsage of type nav_msgs/OccupancyGrid

		Returns:
			None
		"""
		map_data = np.array(data.data)
		# Check that we have map meta_data
		if (self.map_width*self.map_height != 0):
			# turn 1D array into 2D array for image
			map_data = np.reshape(map_data, (self.map_height, self.map_width))
			
			# convert values in costmap
			map_data[map_data == -1] = 150
			map_data[map_data == 100] = 255
			# Add square where robot is
			for i in range(20):
				for j in range(20):
					map_data[int(self.y_in_map)+i,int(self.x_in_map)+j] = 255
			im = np.array(map_data, dtype = np.uint8)
			
			# resize and flip image
			img = cv2.resize(im,(64,64))
			self.horizontal_img = cv2.flip( img, 0 )

			# show image if run as main
			if __name__ == '__main__':
				cv2.imshow('image',horizontal_img)
				cv2.waitKey(2)

	def meta_callback(self, data):
		""" Callback to get the width, height, origin and resolution of the map

		Args:
			data: ros messsage of type nav_msgs/MapMetaData

		Returns:
			None

		"""	

		# Store the desired values to class attributes
		self.map_width = data.width
		self.map_height = data.height
		self.map_x_origin = data.origin.position.x
		self.map_y_origin = data.origin.position.y
		self.resolution = data.resolution

	def odom_callback(self, data):
		""" Callback to get the x,y co-ords of the robot from the odometry/filtered topic.
		This also converts these co-ords to be from the map grid origin (bottom left corner) 
		rather than the map cantre origin

		Args:
			data: ros message of type nav_msgs/Odometry

		Returns:
			None

		"""
		if data.child_frame_id == "base_link":
			self.current_x = data.pose.pose.position.x
			self.current_y = data.pose.pose.position.y

			# Convert position to be relative to bottom left of map
			self.x_in_map = (self.current_x - self.map_x_origin)/self.resolution
			self.y_in_map = (self.current_y - self.map_y_origin)/self.resolution


	def listener(self):
		# Init node if script run as main
		if __name__ == '__main__':
			rospy.init_node('map_to_image', anonymous=True)

		# Subscribe to relevant topics
		rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
		rospy.Subscriber("/map_metadata", MapMetaData, self.meta_callback)
		rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)

		# Keep running node if main script
		if __name__ == '__main__':
			rospy.spin()

# code to test the image output if run as main script
if __name__ == '__main__':
	mi = MapImage()