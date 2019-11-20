#!/usr/bin/env python
import rospy
import os
import numpy as np
import move
import roslaunch
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController

observation = np.array([2,1,5,1,1,3,3])
states = np.array([[1,3,2,2,1,5,1],[1,3,2,1,1,5,1],[1,3,3,2,1,5,1]])


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

class reinforcement_model:

	def _scan_avs_callback(self, data):
		print(data)

	delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
	spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
	unload_controller = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
	switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

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

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	# #roslaunch.configure_logging(uuid)
	# launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jordan/jackal_ws/src/jackal/jackal_navigation/launch/include/gmapping.launch"])
	# launch.start()

	cli_args = ['/home/jordan/jackal_ws/src/jackal/jackal_navigation/launch/include/gmapping.launch','>/dev/null']
	roslaunch_args = cli_args[1:]
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

	parent.start()

	rospy.Subscriber("scan_avs", String, _scan_avs_callback)

	state_action_pairs = np.array([[]])
	actions = np.array(["N","NW","W","SW","S","SE","E","NE"])
	observation = np.array([0,0,0,0,0,0,0,0])

	episodes = 3
	discount = 0.5
	alpha = 0.1

	new_movement = move.Movement()
	new_movement.listener()


	def _new_jackal(self):
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
		print("New Jackal ready")

	def _choose_action(self, method):
		if method == "epsilon":
			pass
		elif method == "random":
			return np.random.choice(self.actions)

	def run(self):
		for i in range(self.episodes):

			# start with fresh states and a new jackal
			self.state_action_pairs = np.array([[]])
			self._new_jackal()

			# Loop for set amount of time or untill not learning anything new
			steps = 0
			while(steps < 1):
				steps = steps + 1
				action = self._choose_action("random")
				print(action)

				self.new_movement.move(action)

				# Add current state to list if not seen before 
				# Act based on policy 
				# See reward
				# Update policy

rm = reinforcement_model()
rm.run()


