#!/usr/bin/env python
import rospy
import os
import numpy as np
import ml_lib as ml
import collections
import random

import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController

from keras import optimizers
from keras.models import Sequential
from keras.layers import Dense, Activation

# global variables
learning_rate = 0.001
action_size = 8
state_size = 8
epsilon = 1
epsilon_min = 0.01
epsilon_decay = 0.9995
gamma = 0.95

rospy.init_node('RL', anonymous=True)
memory = collections.deque(maxlen=5000)

# Create openai environment
env = ml.MachineLearning()

# env = gym.make("CartPole-v0")
# Reset environment 
observation = env.reset()

# Neural Net for Deep Q Learning
# Sequential() creates the foundation of the layers.
model = Sequential()
# 'Dense' is the basic form of a neural network layer
# Input Layer of state size(4) and Hidden Layer with 24 nodes
model.add(Dense(24, input_dim=state_size, activation='relu'))
# Hidden layer with 24 nodes
model.add(Dense(24, activation='relu'))
# Output Layer with # of actions: 2 nodes (left, right)
model.add(Dense(action_size, activation='linear'))
# Create the model based on the information above
model.compile(loss='mse',optimizer=optimizers.Adam(lr=learning_rate))

indo = {"N":0,"NW":1,"W":2,"SW":3,"S":4,"SE":5,"E":6,"NE":7}

def act(state):
	if np.random.rand() <= epsilon:
		# The agent acts randomly
		return np.random.choice(env.actions)

	# Predict the reward value based on the given state
	act_values = model.predict(state)
	# Pick the action based on the predicted reward
	return env.actions[np.argmax(act_values[0])]

def remember(state, action, reward, next_state, done):
		memory.append((state, action, reward, next_state, done))

def replay(batch_size):
		global epsilon
		minibatch = random.sample(memory, batch_size)
		for state, action, reward, next_state, done in minibatch:
			target = reward
			if not done:
			  target = reward + gamma * np.amax(model.predict(next_state)[0])
			target_f = model.predict(state)

			target_f[0][indo[action]] = target
			model.fit(state, target_f, epochs=1, verbose=0)
		if epsilon > epsilon_min:
			epsilon *= epsilon_decay

#env._max_episode_steps = 2000

# Loop through episodes
for e in range(10000):
	state = env.reset()
	assert(state != [0]*state_size)
	state = np.reshape(state, [1, 8])

	for t in range(2000):
		state = env.get_observation()
		state = np.reshape(state, [1, 8])

	 	action = act(state)

	 	observation, reward, done, info = env.step(action)

	 	observation = np.reshape(observation, [1, 8])

	 	remember(state, action, reward, observation, done)

	 	state = observation

		if done or t == 30:
			
			print("episode: {}, score: {}, eps: {}, memory: {}".format(e, env.information_metric, epsilon, len(memory)))
			observation = env.reset()
			break

		if len(memory) > 16:
			print("########################3")
			replay(16)

		
env.close()


# # function to assist in choosing a new action 
# def _choose_action(method):
# 	if method == "epsilon":
# 		pass
# 	elif method == "random":
# 		return np.random.choice(self.actions)

# ml = MachineLearning()

# # actual machine learning algorithm
# def run(self):
	
# 	for i in range(self.episodes):

# 		# start with fresh states and a new jackal
# 		# self.state_action_pairs = np.array([[]])
# 		ml.new_model()

# 		# Loop for set amount of time or untill not learning anything new
# 		steps = 0
# 		while(steps < 1):
# 			steps = steps + 1

# 			# Get state observation
# 			observation = [ml.get_observation]
# 			exists, index = circular_equality(Q_table[:][0], observation)

# 			# Choose action

# 			action = _choose_action("random")
# 			print(action)

# 			# Add to Q_table if isnt present

# 			if not exists:
# 				Q_table = np.vstack((Q_table, [observation,action,0,0]))

# 			# Observe reward

# 			# Calculate Q Value for action and add to Q-table

# 			ml.move_model(action)

# 			# Update policy here for TD

# 		# Update policy here for MC

# # rm = reinforcement_model()
# # rm.run()
