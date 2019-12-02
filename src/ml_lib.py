#!/usr/bin/env python
import rospy
import os
import numpy as np
from numpy import inf
import move
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController

observation = np.array([2,1,5,1,1,3,3])
states = np.array([[1,3,2,2,1,5,1],[1,3,2,1,1,5,1],[1,3,3,2,1,5,1]])

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

	# Roslaunch api usage for gmapping reset

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

	cli_args = ['/home/jordan/jackal_ws/src/jackal_exploration/launch/gmapping_no_output.launch']
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

	# Global variables and data structures

	actions = np.array(["N","NW","W","SW","S","SE","E","NE"])
	  
	observation = np.array([0,0,0,0,0,0,0,0])
	scans = [0,0,0,0,0,0,0,0]

	episodes = 3
	discount = 0.5
	alpha = 0.1

	information_metric = 0
	old_info_metric = 0

	# Import movement object to move model

	new_movement = move.Movement()
	new_movement.listener()


	# Usable methods from this class to assist machine learning algorithms

	# Function to spawn new model and reset gmapping
	# need to add option for new world here

	def rotate(l, n):
		return l[n:] + l[:n]

	# Callbacks

	
	def _avs_callback(self,data):
		self.scans = str(data.data).split("&")
		
	def _reward_callback(self, data):
		map_data = np.array(data.data)
		unknowns = np.count_nonzero(map_data == -1)
		self.information_metric = len(map_data)-unknowns

	def listener(self):
		rospy.Subscriber("/map", OccupancyGrid, self._reward_callback)
		rospy.Subscriber("scan_avs", String, self._avs_callback)
		# rospy.Subscriber("/odometry/filtered", Odometry, self._odom_callback)


	def new_model(self):
		try:
			self.delete_model("jackal")
		except:
			print("None")
		self.spawn_model("jackal",self.xml_string,"",self.pose,"world")
		# Wait for services to be available
		rospy.wait_for_service('controller_manager/load_controller')
		rospy.wait_for_service('controller_manager/switch_controller')
		rospy.wait_for_service('controller_manager/unload_controller')
		# Unload controllers
		self.unload_controller("jackal_joint_publisher")
		self.unload_controller("jackal_velocity_controller")
		# Load controllers
		self.load_controller("jackal_velocity_controller")
		self.load_controller("jackal_joint_publisher")
		# Start controllers
		self.switch_controller(["jackal_velocity_controller","jackal_joint_publisher"],[],2)
		
		self.parent.shutdown()
		self.parent.start()
		self.information_metric = 0
		self.old_info_metric = 0
		print("New Jackal ready")

	def get_observation(self):

		temp = self.scans
		# heading = self.new_movement.heading
		# temp = self.rotate(temp,)
		for scan in range(len(temp)):
			if temp[scan] == inf:
				temp[scan] = 0
			else:
				temp[scan] = float(temp[scan])
		return temp

	def move_model(self, action):
		self.new_movement.move(action)

	def delta_score(self):
		value = self.information_metric - self.old_info_metric
		self.old_info_metric = self.information_metric
		return value

	def reset(self):
		self.new_model()
		return self.get_observation()

	def step(self, action):
		self.move_model(action)


		reward = self.delta_score()

		return self.get_observation(), reward, 0, 0


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

