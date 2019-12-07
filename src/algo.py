#!/usr/bin/env python
import rospy
import os
import numpy as np
import ml_lib as ml
import collections
import random
import sys
from memory_profiler import profile
import gc

import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController

from keras import optimizers
from keras.models import Sequential
from keras.layers import Dense, Activation

import resource


# global variables
learning_rate = 0.001
action_size = 8
state_size = 8
epsilon = 1
epsilon_min = 0.01
epsilon_decay = 0.995
gamma = 0.95

rospy.init_node('RL', anonymous=True)
memory = collections.deque(maxlen=40)

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
model.add(Dense(48, input_dim=state_size, activation='relu'))
# Hidden layer with 24 nodes
model.add(Dense(48, activation='relu'))

model.add(Dense(24, activation='relu'))
# Output Layer with # of actions: 2 nodes (left, right)
model.add(Dense(action_size, activation='linear'))
# Create the model based on the information above
model.compile(loss='mse',optimizer=optimizers.Adam(lr=learning_rate))

indo = {"N":0,"NW":1,"W":2,"SW":3,"S":4,"SE":5,"E":6,"NE":7}

def sizeof_fmt(num, suffix='B'):
	''' by Fred Cirera,  https://stackoverflow.com/a/1094933/1870254, modified'''
	for unit in ['','Ki','Mi','Gi','Ti','Pi','Ei','Zi']:
		if abs(num) < 1024.0:
			return "%3.1f %s%s" % (num, unit, suffix)
		num /= 1024.0
	return "%.1f %s%s" % (num, 'Yi', suffix)


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

# Loop through episodes

def main():
	for e in range(3000):
		state = env.reset()
		if (state == [0]*state_size):
			print("points not being published")
		#assert(state != [0]*state_size)
		state = np.reshape(state, [1, 8])
		running_rew = 0
		for t in range(2000):
			result = gc.collect()
			print(str(t+1)+"/30")
			sys.stdout.write("\033[F")
			state = env.get_observation()
			state = np.reshape(state, [1, 8])

			action = act(state)
			#print '1 Memory usage: %s (kb)' % resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

			observation, reward, done, info = env.step(action)
			#print '2 Memory usage: %s (kb)' % resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
			observation = np.reshape(observation, [1, 8])

			remember(state, action, reward, observation, done)
			running_rew = running_rew + reward

			state = observation

			if done or t == 50:
				print("episode: {}, score: {}, eps: {}, memory: {}".format(e, running_rew, epsilon, len(memory)))
				# observation = env.reset()
				break

		if len(memory) > 32:
			replay(32)
		model.save_weights("weights/"+str(e)+'_my_model_weights.h5')

main()
		
#env.close()