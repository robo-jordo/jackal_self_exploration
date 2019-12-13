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
import resource
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

# global variables
learning_rate = 0.000001
action_size = 8

epsilon = 1
epsilon_min = 0.01
epsilon_decay = 0.995
gamma = 0.95

rospy.init_node('RL', anonymous=True)
memory = collections.deque(maxlen=5000)

# Create ml environment
env = ml.MachineLearning()

env.obs_type = "img"

# observation = env.reset()

# Function to return standard nn
def create_nn():
	nn = Sequential()
	nn.add(Dense(16, input_dim=state_size, activation='tanh'))
	nn.add(Dense(action_size, activation='linear'))
	nn.compile(loss='mse',optimizer=optimizers.Adam(lr=learning_rate))
	return nn

# Function to return convolutional nn
def create_cnn():
	nn = Sequential()
	nn.add(Conv2D(32, 8, 8, subsample=(4, 4), input_shape=(64, 64, 1)))
	nn.add(Activation('relu'))
	nn.add(Conv2D(64, 4, 4, subsample=(2, 2)))
	nn.add(Activation('relu'))
	nn.add(Flatten())
	nn.add(Dense(512))
	nn.add(Activation('linear'))
	nn.add(Dense(action_size))
	nn.compile(loss='mse',optimizer=optimizers.Adam(lr=0.00001))
	return nn



# dict to map actions to index of nn output
indo = {"N":0,"NW":1,"W":2,"SW":3,"S":4,"SE":5,"E":6,"NE":7}

log_values = []

def act(state):
	global log_values
	if np.random.rand() <= epsilon:
		# return random action from action space
		return np.random.choice(env.actions)

	# Predict expected reward based on the given state
	act_values = model.predict(state)
	log_values.append(env.actions[np.argmax(act_values[0])])
	# Return best action based on the prediction
	return env.actions[np.argmax(act_values[0])]

def remember(state, action, reward, next_state, done):
		memory.append((state, action, reward, next_state, done))

def update_weights():
	# Copy weights from main network to target network
	global target_model
	target_model.set_weights(model.get_weights())

def replay(batch_size):
	# Memory replay
	global epsilon
	global model
	# Get random batch from memory buffer
	minibatch = random.sample(memory, batch_size)
	for state, action, reward, next_state, done in minibatch:
		target_next = reward
		if not done:
			# Target value calculation
			target_next = reward + gamma * np.amax(target_model.predict(next_state)[0])
		# Value prediction for all actions
		target_now = model.predict(state)
		# Insert value of action taken
		target_now[0][indo[action]] = target_next
		# Fit neural net 
		model.fit(state, target_now, epochs=1, verbose=0)
	if epsilon > epsilon_min:
		epsilon *= epsilon_decay


def main():
	global epsilon
	global log_values
	# open file to log progress
	f= open("status.txt","w+")

	for e in range(3000):
		# init vars
		log_values = []
		running_rew = 0

		# reset environment and store observed state
		state = env.reset()
		if env.obs_type == "img":
			state = np.reshape(state, [1, 64, 64, 1])
		else:
			state = np.reshape(state, [1, state_size])

		# Try learned policy without random actions every 10 epsiodes
		if epsilon == 0:
			epsilon =  stored_eps
			print("reverting to training")
		if ((e % 10)==0 and e > 0 ):
			stored_eps = epsilon
			epsilon = 0
			print("Trying policy")

		# loop for steps in episode
		for t in range(2000):

			result = gc.collect()

			# dynamically show progress
			print(str(t+1)+"/100")
			sys.stdout.write("\033[F")

			# Observe current state
			state = env.get_observation()
			if env.obs_type == "img":
				state = np.reshape(state, [1, 64, 64, 1])
			else:
				state = np.reshape(state, [1, state_size])

			# Choose next action to take
			action = act(state)

			# Step environment and store result
			observation, reward, done, info = env.step(action)
			if env.obs_type == "img":
				observation = np.reshape(observation, [1, 64, 64, 1])
			else:
				state = np.reshape(state, [1, state_size])

			# Store experience to memory buffer
			remember(state, action, reward, observation, done)

			# Increment reward
			running_rew = running_rew + reward

			state = observation

			# End episode print progress and write to file
			if done or t == 30:
				print("episode: {}, score: {}, eps: {}, memory: {}, collisions: {}".format(e, running_rew, epsilon, len(memory), env.collisions))
				f.write("episode: {}, score: {}, eps: {}, memory: {}, collisions: {} \r\n".format(e, running_rew, epsilon, len(memory), env.collisions))
				f.flush()
				# observation = env.reset()
				break

			# Train on memory buffer
			if len(memory) > 32:
				replay(32)

		# Copy weights from main network to target network
		update_weights()

		# store weights
		model.save_weights("weights/"+str(e)+'_my_model_weights.h5')

		# Store policy based moves taken
		with open('moves/'+str(e) +'_file.csv', mode='w') as moves_file:
				employee_writer = csv.writer(moves_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				employee_writer.writerow(log_values)
	f.close()


if __name__=="__main__":
	try:
		env.obs_type = rospy.get_param("/observation_type")
	else:
		env.obs_type = "scan"
	print("using " + env.obs_type + " as observation")
	if env.obs_type == "img":
		model = create_cnn()
		target_model = create_cnn()
	elif env.obs_type == "pos":
		state_size = 2
		model = create_cnn()
		target_model = create_cnn()
	elif env.obs_type == "scan":
		state_size = 8
		model = create_cnn()
		target_model = create_cnn()
	main()
