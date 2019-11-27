#!/usr/bin/env python
import rospy
import os
import numpy as np
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

	# Callbacks

	def _avs_callback(self,data):
		self.scans = data.split("&")

	def _reward_callback(self, data):
		map_data = np.array(data.data)
		unknowns = np.count_nonzero(map_data == -1)
		self.information_metric = len(map_data)-unknowns

	# Gazebo services

	delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
	spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
	unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
	switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

	# Subscribers 
	rospy.Subscriber("/map", OccupancyGrid, _reward_callback)
	rospy.Subscriber("scan_avs", String, _avs_callback)

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

	cli_args = ['/home/jordan/jackal_ws/src/jackal/jackal_navigation/launch/include/gmapping.launch','>/dev/null']
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

	# Global variables and data structures

	actions = np.array(["N","NW","W","SW","S","SE","E","NE"])
	observation = np.array([0,0,0,0,0,0,0,0])
	scans = []

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
		print("New Jackal ready")

	def get_observation(self):
		return self.scans

	def move_model(self, action):
		self.new_movement.move(action)

	def delta_score(self):
		value = self.information_metric - self.old_info_metric
		self.old_info_metric = self.information_metric
		return value


# function to assist in choosing a new action 
def _choose_action(method):
	if method == "epsilon":
		pass
	elif method == "random":
		return np.random.choice(self.actions)

Q_table = np.array([])
ml = MachineLearning()

# actual machine learning algorithm
def run(self):
	
	for i in range(self.episodes):

		# start with fresh states and a new jackal
		# self.state_action_pairs = np.array([[]])
		ml.new_model()

		# Loop for set amount of time or untill not learning anything new
		steps = 0
		while(steps < 1):
			steps = steps + 1

			# Get state observation
			observation = [ml.get_observation]
			exists, index = circular_equality(Q_table[:][0], observation)

			# Choose action

			action = _choose_action("random")
			print(action)

			# Add to Q_table if isnt present

			if not exists:
				Q_table = np.vstack((Q_table, [observation,action,0,0]))

			# Observe reward

			# Calculate Q Value for action and add to Q-table

			ml.move_model(action)

			# Update policy here for TD

		# Update policy here for MC

# rm = reinforcement_model()
# rm.run()


def numpy_test():

	Qs = np.array([[[1,3,2,2,1,5,1],"NW",2,1],
				   [[1,3,2,1,1,5,1],"N",3,2],
				   [[1,3,3,2,1,5,1],"W",3,1]])
	print(Qs[:,0])
	print(circular_equality(Qs[:,0],observation))

	Qs = np.vstack((Qs, [[1,3,2,2,1,5,5],"S",3,1]))
	print(Qs)



numpy_test()

