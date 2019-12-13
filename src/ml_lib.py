#!/usr/bin/env python
import rospy
import os
import numpy as np
from numpy import inf
import move
import mapper
import roslaunch
import sys
import tf
import random
import rospkg
import resource

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from robot_localization.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController

class MachineLearning:

	def __init__(self):
		self.listener()

	rospack = rospkg.RosPack()

	# Publisher to reset gmapping each episode
	pub = rospy.Publisher('/syscommand', String, queue_size=1)

	# Gazebo services
	set_gaz_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	get_gaz_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	set_state = rospy.ServiceProxy('/set_pose', SetPose)
	delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
	spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
	unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
	switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

	# URDF and pose variables
	p = os.popen(rospack.get_path('jackal_description')+'/scripts/env_run '+rospack.get_path('jackal_description')+'/urdf/configs/front_laser rosrun xacro xacro.py ' + rospack.get_path('jackal_description') +'/urdf/jackal.urdf.xacro')
	xml_string = p.read()
	p.close()

	# Empty pose for ekf localization set pose
	pose2 = PoseWithCovarianceStamped()
	pose2.pose.pose.position.x = 0
	pose2.pose.pose.position.y = 0
	pose2.pose.pose.position.z = 0.2
	pose2.pose.pose.orientation.x = 0
	pose2.pose.pose.orientation.y = 0
	pose2.pose.pose.orientation.z = 0
	pose2.pose.pose.orientation.w = 1
	pose2.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	# Empty pose for gazebo set_model_state
	state_msg = ModelState()
	state_msg.model_name = 'jackal'
	state_msg.pose.position.x = 0
	state_msg.pose.position.y = 0
	state_msg.pose.position.z = 0.3
	state_msg.pose.orientation.x = 0
	state_msg.pose.orientation.y = 0
	state_msg.pose.orientation.z = 0
	state_msg.pose.orientation.w = 1

	# List of valid start points in jackal race world
	random_spawns = 1
	start_points = [(-2,-1.15),(-4.5,-1.3),(-4.5,8.2),(-1.9,1.6),(5.5,-5.3),(5.75,-8.5),(-6.75,-4.5)]

	# Global variables and data structures

	actions = np.array(["N","NW","W","SW","S","SE","E","NE"])

	observation = np.array([0,0,0,0,0,0,0,0])
	scans = [0,0,0,0,0,0,0,0]

	collisions = 0


	information_metric = 0
	old_info_metric = 0

	current_co_ord = []
	obs_type = "scan"

	# Import movement object to move model
	new_movement = move.Movement()
	new_movement.listener()

	mi = mapper.MapImage()
	mi.listener()


	# Callbacks
	def _avs_callback(self,data):
		""" Callback to take in the average lidar values produced by the points node
			and split it into 8 values.

		Args:
			data: The string data passed from the points node

		Returns:
			None

		"""
		self.scans = str(data.data).split("&")

	def _pos_callback(self,data):
		""" Callback to get the x,y co-ords of the robot from the odometry/filtered topic.

		Args:
			data: ros message of type nav_msgs/Odometry

		Returns:
			None

		"""
		if data.child_frame_id == 'base_link':
			self.current_co_ord = [data.pose.pose.position.x , data.pose.pose.position.y]

	def _reward_callback(self, data):
		""" Callback to calculate how many blocks of the map have known contents

		Args:
			data: ros messsage of type nav_msgs/OccupancyGrid

		Returns:
			None

		"""
		map_data = np.array(data.data)
		unknowns = np.count_nonzero(map_data == -1)
		self.information_metric = len(map_data)-unknowns

	# function to set up callbacks
	def listener(self):
		rospy.Subscriber("/map", OccupancyGrid, self._reward_callback)
		rospy.Subscriber("/scan_avs", String, self._avs_callback)
		rospy.Subscriber("/odometry/filtered", Odometry, self._pos_callback)

	# Usable methods from this class to assist machine learning algorithms

	def _new_model(self):
		""" Function to reset model state in gazebo and ekf_localization.
		    This function also resets the map being output by gmapping

		Args:
			None

		Returns:
			None

		"""
		rospy.wait_for_service("gazebo/set_model_state", timeout=20)

		self.pose2.header.stamp = rospy.Time.now()

		# Choose random point from defined safe respawn points
		if self.random_spawns:
			start_pose = random.choice(self.start_points)

			self.state_msg.pose.position.x = start_pose[0]
			self.state_msg.pose.position.y = start_pose[1]

			self.pose2.pose.pose.position.x = start_pose[0]
			self.pose2.pose.pose.position.y = start_pose[1]

		# Reset Gazebo
		resp = self.set_gaz_state( self.state_msg )

		# Reset ekf_localization
		resp = self.set_state(self.pose2)

		# Reset Gmapping map
		self.pub.publish("reset")

		# pause for map to clear
		rospy.sleep(2)

		# reset information metrics
		self.information_metric = 0
		self.old_info_metric = 0


	def get_observation(self):
		""" Get observation of the relevant type 

		Args:
			None

		Returns:
			Observation: This observation will be of the type specified by the
						 obs_type attribute of this class.
						 - for obs_type = scan: list of 8 floats (8 scan segment averages)
						 - for obs_type = pos: list of co-ordinates of the form [x,y]
						 - for obs_type = img: 64x64 numpy array representing an image of 
						                       the robots current understanding of the map


		"""

		# Observation for scan type
		if (self.obs_type == "scan"):
			temp = self.scans
			for scan in range(len(temp)):
				if float(temp[scan]) >100:
					temp[scan] = 0
				else:
					temp[scan] = float(temp[scan])
			return temp

		# observation for pos type
		elif (self.obs_type == "pos"):
			object_coords = self.get_gaz_state("jackal","")
			return [object_coords.pose.position.x , object_coords.pose.position.y]
		
		# Observation for img type
		elif (self.obs_type == "img"):
			return self.mi.horizontal_img

		# Catch all for problems seting obs_type
		else:
			print("Set observation type") 

	def move_model(self, action):
		""" Function to move the gazebo model using Movement object.

		Args:
			action (str): The direction in which to move

		Returns:
			result (int): -1 if the requested move would have cause a collision
 						   1 if the requested move was succesful
		"""
		result = self.new_movement.move(action)
		return result

	def delta_score(self):
		""" Function to return a reward

		Args:
			None

		Returns:
			value (int): The new information gathered in the map

		"""
		value = self.information_metric - self.old_info_metric
		self.old_info_metric = self.information_metric
		return value

	def reset(self):
		""" Function reset simulation

		Returns:
			Observation (list): A list of float values of the average value
								of each lidar segment

		"""
		self._new_model()
		self.collisions = 0
		return self.get_observation()

	def step(self, action):
		""" Function to move model and return useful info (like the OpenAi gym does)

		Args:
			action (str): The direction in which to move

		Returns:
			Observation (list): A list of float values of the average value
								of each lidar segment
			value (int): The new information gathered in the map
			done (bool): Whether the simulation in a termination state
			info (int): Nothing yet just mirrors OpenAi gym setup

		"""

		result = self.move_model(action)
		reward = self.delta_score()

		# set the reward scalar here 
		#############################
		if reward <5:
			reward = -0.5
		else:
			reward = 0.5
		if result == -1:
			reward = -1
		#############################

		# Increment collisions 
		if result == -1:
			self.collisions = self.collisions + 1
		return self.get_observation(), reward, 0, 0

