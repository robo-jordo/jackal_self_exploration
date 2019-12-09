#!/usr/bin/env python
import rospy
import os
import numpy as np
from numpy import inf
import move
import roslaunch
import sys
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from robot_localization.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController
import resource

# function to assist in checking if two circular states are equal
def circular_equality(states, observation):
	exists =  False
	index = -1
	doubled_obs = np.concatenate((observation, observation))
	print(doubled_obs)
	doubled_obs = ''.join(map(str, doubled_obs))
	for i in range(len(states)):
		comp_string = ''.join(map(str, states[i]))
		if comp_string in doubled_obs:
			exists = True
			index = i
			break
	return exists, index

class MachineLearning:

	def __init__(self):
		self.listener()


	# Gazebo services
	set_gaz_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	set_state = rospy.ServiceProxy('/set_pose', SetPose)
	delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
	spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
	unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
	switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

	# URDF and pose variables

	p = os.popen("~/jackal_ws/src/jackal/jackal_description/scripts/env_run ~/jackal_ws/src/jackal/jackal_description/urdf/configs/front_laser rosrun xacro xacro.py " + "~/jackal_ws/src/jackal/jackal_description/urdf/jackal.urdf.xacro")
	xml_string = p.read()
	p.close()
	pose = Pose()
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 0.2
	pose.orientation.x = 0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 1

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

	state_msg = ModelState()
	state_msg.model_name = 'jackal'
	state_msg.pose.position.x = 0
	state_msg.pose.position.y = 0
	state_msg.pose.position.z = 0.3
	state_msg.pose.orientation.x = 0
	state_msg.pose.orientation.y = 0
	state_msg.pose.orientation.z = 0
	state_msg.pose.orientation.w = 1

	# Roslaunch api usage for gmapping reset

	pub = rospy.Publisher('/syscommand', String, queue_size=1)
	# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	# roslaunch.configure_logging(uuid)
	# launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/jordan/jackal_ws/src/jackal_exploration/launch/gmapping_no_output.launch'])
	# launch.start()
	# cli_args = ['/home/jordan/jackal_ws/src/jackal_exploration/launch/gmapping_no_output.launch']
	# roslaunch_args = cli_args[1:]
	# roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

	# parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

	# Global variables and data structures

	actions = np.array(["N","NW","W","SW","S","SE","E","NE"])

	observation = np.array([0,0,0,0,0,0,0,0])
	scans = [0,0,0,0,0,0,0,0]

	episodes = 3
	discount = 0.5
	alpha = 0.1
	collisions = 0

	information_metric = 0
	old_info_metric = 0

	# Import movement object to move model

	new_movement = move.Movement()
	new_movement.listener()

	odom_broadcaster = tf.TransformBroadcaster()


	# Callbacks
	def _avs_callback(self,data):
		""" Function to spawn new model and controllers in gazebo and start the controllers.

		Args:
			param1: The first parameter.
			param2: The second parameter.

		Returns:
			The return value. True for success, False otherwise.

		"""
		self.scans = str(data.data).split("&")

	def _reward_callback(self, data):
		""" Function to spawn new model and controllers in gazebo and start the controllers.

		Args:
			param1: The first parameter.
			param2: The second parameter.

		Returns:
			The return value. True for success, False otherwise.

		"""
		map_data = np.array(data.data)
		unknowns = np.count_nonzero(map_data == -1)
		self.information_metric = len(map_data)-unknowns

	# function to set up callbacks
	def listener(self):
		rospy.Subscriber("/map", OccupancyGrid, self._reward_callback)
		rospy.Subscriber("scan_avs", String, self._avs_callback)


	# Usable methods from this class to assist machine learning algorithms
	#@profile
	def new_model(self):
		""" Function to spawn new model and controllers in gazebo and start the controllers.

		Args:
			param1: The first parameter.
			param2: The second parameter.

		Returns:
			The return value. True for success, False otherwise.

		"""
		# try:
		# 	self.delete_model("jackal")
		# except:
		# 	print("None")


		#try:
		rospy.wait_for_service("gazebo/set_model_state", timeout=20)
		#rospy.wait_for_service("set_pose", timeout=20)
		self.pose2.header.stamp = rospy.Time.now()
		resp = self.set_gaz_state( self.state_msg )
		resp = self.set_state(self.pose2)


			# response = self.spawn_model("jackal",self.xml_string,"",self.pose,"world")
			# print(response.success)
		# except (rospy.ServiceException), e:
		# 	print(e)
		# except:
		# 	print("Issue")

		# Wait for services to be available
		# rospy.wait_for_service('controller_manager/load_controller')
		# rospy.wait_for_service('controller_manager/switch_controller')
		# rospy.wait_for_service('controller_manager/unload_controller')
		# # Unload controllers
		# self.unload_controller("jackal_joint_publisher")
		# self.unload_controller("jackal_velocity_controller")
		# # Load controllers
		# self.load_controller("jackal_velocity_controller")
		# self.load_controller("jackal_joint_publisher")
		# # Start controllers
		# self.switch_controller(["jackal_velocity_controller","jackal_joint_publisher"],[],2)

		# try:
		# 	self.launch.shutdown()
		# except:
		# 	print("Na")
		# self.launch.start()
		# print(sys.getsizeof(self.launch))
		self.pub.publish("reset")
		rospy.sleep(2)


		self.information_metric = 0
		self.old_info_metric = 0
		#print("New Jackal ready")

	def get_observation(self):
		""" Function to spawn new model and controllers in gazebo and start the controllers.

		Args:
			param1: The first parameter.
			param2: The second parameter.

		Returns:
			The return value. True for success, False otherwise.

		"""
		temp = self.scans
		for scan in range(len(temp)):
			#print(temp[scan])
			if float(temp[scan]) >100:
				temp[scan] = 0
				#print("GOT ONE")
			else:
				temp[scan] = float(temp[scan])
		#print("temp" + str(temp))
		return temp

	def move_model(self, action):
		""" Function to move the gazebo model using Movement object.

		Args:
			action (str): The direction in which to move

		Returns:
			None

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
		self.new_model()
		self.collisions = 0
		return self.get_observation()

	def step(self, action):
		""" Function to move model and return useful info like the OpenAi gym

		Args:
			action (str): The direction in which to move

		Returns:
			Observation (list): A list of float values of the average value
								of each lidar segment
			value (int): The new information gathered in the map
			done (bool): Whether the simulation in a termination state
			info (int): ???? Nothing yet just mirrors OpenAi gym setup

		"""
		result = self.move_model(action)
		reward = self.delta_score()
		if result == -1:
			reward = -10000
			self.collisions = self.collisions + 1
		# else:
		# 	reward = 1
		return self.get_observation(), reward, 0, 0

# Test case stuff
observation = np.array([2,1,5,1,1,3,3])
states = np.array([[1,3,2,2,1,5,1],[1,3,2,1,1,5,1],[1,3,3,2,1,5,1]])

def numpy_test():

	Qs = np.array([[[1,3,2,2,1,5,1],"NW",2,1],
				   [[1,3,2,1,1,5,1],"N",3,2],
				   [[1,3,3,2,1,5,1],"W",3,1]])
	print(Qs[:,0])
	print(circular_equality(Qs[:,0],observation))

	Qs = np.vstack((Qs, [[1,3,2,2,1,5,5],"S",3,1]))
	print(Qs)

if __name__ == '__main__':
	numpy_test()
