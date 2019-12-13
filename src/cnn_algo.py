#!/usr/bin/env python
import rospy
import os
import numpy as np
import ml_lib as ml
import collections
import random
import sys
import gc
import csv

import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController

from keras import optimizers, losses
from keras.models import Sequential
from keras.layers import Dense, Activation, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D

import resource


# global variables
learning_rate = 0.000001
action_size = 8

epsilon = 1
epsilon_min = 0.01
epsilon_decay = 0.995
gamma = 0.95

rospy.init_node('RL', anonymous=True)
memory = collections.deque(maxlen=5000)

# Create openai environment
env = ml.MachineLearning()

# env = gym.make("CartPole-v0")
# Reset environment
# env.obs_type = "pos"
observation = env.reset()
env.obs_type = "img"

model = Sequential()
model.add(Conv2D(32, 8, 8, subsample=(4, 4), input_shape=(64, 64, 1)))
model.add(Activation('relu'))
model.add(Conv2D(64, 4, 4, subsample=(2, 2)))
model.add(Activation('relu'))
model.add(Flatten())
model.add(Dense(512))
model.add(Activation('linear'))
model.add(Dense(action_size))
model.compile(loss='mse',optimizer=optimizers.Adam(lr=0.00001))


target_model = Sequential()
target_model.add(Conv2D(32, 8, 8, subsample=(4, 4), input_shape=(64, 64, 1)))
target_model.add(Activation('relu'))
target_model.add(Conv2D(64, 4, 4, subsample=(2, 2)))
target_model.add(Activation('relu'))
target_model.add(Flatten())
target_model.add(Dense(512))
target_model.add(Activation('linear'))
target_model.add(Dense(action_size))
target_model.compile(loss='mse',optimizer=optimizers.Adam(lr=0.00001))
# Neural Net for Deep Q Learning
# Sequential() creates the foundation of the layers.
# model = Sequential()
# # 'Dense' is the basic form of a neural network layer
# # Input Layer of state size(4) and Hidden Layer with 24 nodes
# model.add(Dense(16, input_dim=state_size, activation='tanh'))
# # Hidden layer with 24 nodes
# #model.add(Dense(16, activation='relu'))
# # Output Layer with # of actions: 2 nodes (left, right)
# model.add(Dense(action_size, activation='linear'))
# Create the model based on the information above
#model.compile(loss='mse',optimizer=optimizers.Adam(lr=learning_rate))
#model.load_weights('weights/2157_my_model_weights.h5')

indo = {"N":0,"NW":1,"W":2,"SW":3,"S":4,"SE":5,"E":6,"NE":7}

log_values = []

def act(state):
	global log_values
	if np.random.rand() <= epsilon:
		# The agent acts randomly
		return np.random.choice(env.actions)

	# Predict the reward value based on the given state
	act_values = model.predict(state)
	# Pick the action based on the predicted reward
	log_values.append(env.actions[np.argmax(act_values[0])])
	return env.actions[np.argmax(act_values[0])]

def remember(state, action, reward, next_state, done):
		memory.append((state, action, reward, next_state, done))

def update_weights():
	global target_model
	target_model.set_weights(model.get_weights())

def replay(batch_size):
		global epsilon
		global model
		minibatch = random.sample(memory, batch_size)
		for state, action, reward, next_state, done in minibatch:
			target = reward
			if not done:
			  target_next = reward + gamma * np.amax(target_model.predict(next_state)[0])
			target_now = model.predict(state)

			target_now[0][indo[action]] = target_next
			model.fit(state, target_now, epochs=1, verbose=0)
		if epsilon > epsilon_min:
			epsilon *= epsilon_decay

# Loop through episodes

def main():

	global epsilon
	global log_values
	f= open("status.txt","w+")
	for e in range(3000):
		log_values = []
		state = env.reset()
		# if (state == [0]*state_size):
		# 	print("points not being published")
		#assert(state != [0]*state_size)
		state = np.reshape(state, [1, 64, 64, 1])
		running_rew = 0

		if epsilon == 0:
			epsilon =  stored_eps
			print("reverting to training")
		if ((e % 10)==0 and e > 0 ):
			stored_eps = epsilon
			epsilon = 0
			print("Trying policy")
		for t in range(2000):
			result = gc.collect()
			print(str(t+1)+"/100")
			sys.stdout.write("\033[F")
			state = env.get_observation()
			state = np.reshape(state, [1, 64, 64, 1])
			# state = np.reshape(state, [1, state_size])

			action = act(state)
			#print '1 Memory usage: %s (kb)' % resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
			#print("action: "+str(action))
			observation, reward, done, info = env.step(action)
			#print '2 Memory usage: %s (kb)' % resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
			# observation = np.reshape(observation, [1, state_size])
			observation = np.reshape(observation, [1, 64, 64, 1])

			remember(state, action, reward, observation, done)
			running_rew = running_rew + reward

			state = observation

			if done or t == 30:
				print("episode: {}, score: {}, eps: {}, memory: {}, collisions: {}".format(e, running_rew, epsilon, len(memory), env.collisions))
				f.write("episode: {}, score: {}, eps: {}, memory: {}, collisions: {} \r\n".format(e, running_rew, epsilon, len(memory), env.collisions))
				f.flush()
				# observation = env.reset()
				break

			if len(memory) > 32:
				replay(32)
		update_weights()
		model.save_weights("weights/"+str(e)+'_my_model_weights.h5')
		with open('moves/'+str(e) +'_file.csv', mode='w') as moves_file:
    			employee_writer = csv.writer(moves_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    			employee_writer.writerow(log_values)
	f.close()
main()

#env.close()