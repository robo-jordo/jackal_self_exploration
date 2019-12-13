# Jackal autonomous exploration
The aim of this project was to use reinforcement learning to develop an exploration policy for a mobile robot. This is a broad goal that can have many different learning goals. Some of these learning goals include:
* Learning basic obstacle avoidance using LIDAR data as state information
* Learning to explore a static environment using (x,y) coordinates as state information.
* Learning a general exploration policy for arbitrary environments using either LIDAR data or an image of the current understanding of the map as state information.

## Motivation
This project was completed as part of my final Masters in Robotics program at Northwestern university. This project was completed in partnership with a lab interested in the effect that increased range of sight owing to evolution had on the ability and necessity for animals to plan once they started to live on land where range of sight was greatly increased. This project aims to align with these interests by investigating the way that these learned policies change if the range of sight of the robot is changed.

!!!!!!!!!!!!!!!Put images/ videos here!!!!!!!!!!!!!!!!!!!!!!!!!1

## Requirements:

* Linux 18.04
* ROS melodic
* Forked Gmapping repo with restart topic
* Forked LMS1xx repo for 360 degree Lidar

for real world run:

* Jackal and 360 planar Lidar or 3D Lidar

## Contents of repo:
#### * launch

    * x_y.launch: launches everything for a run of deep Q-learning using x,y position as state.
    * scan_segs.launch: launches everything for a run of deep Q-learning using the averages of 8 scan segments as state.
    * cnn.launch: launches everything for a run of deep Q-learning using images of the map from gmapping as state

#### * src

    * **algo.py:**
    * ml_lib.py:
    * move.py:
    * points.py:
    * mapper.py:

## Hardware
The platform used for this project is the ClearPath Jackal. This is owing to the fact that the Northwestern MSR lab has a ClearPath Jackal With a 3D Velodyne LIDAR and pointgrey camera available for use. This provides a platform with easily accessible sensors that are well integrated and maintained with ROS. Additionally CLearPath provide Gazebo simulation resources for the Jackal, which is very useful for reinforcement learning where harm may come to a real world robot. As a run up to this project I put together a [GitHub repo](https://github.com/robo-jordo/jackal_melodic_bringup) to bring a Jackal up with ROS melodic  and the Velodyne LIDAR. (ClearPath does not yet provide a melodic image for the Jackal)


## Simulation set up and explanation

In order to run reinforcement learning episodes safely, a gazebo simulation of the Jackal was used. Instructions for simulating the Jackal can be found on [ClearPaths website](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). Some other packages such as Gmapping were used for this project ## Deps ??? ##

I have modified various packages to suit the needs of my simulation.
The LMS1xx package that I have modified has been forked to my GitHub account and are linked with ##DEPS###. This packages need to be cloned into a catkin workspace with this repo and built from source by running catkin_make.

A forked version of [slam_gmapping](https://github.com/jukindle/slam_gmapping) created by [jukindle](https://github.com/jukindle) was used in order to allow Gmapping to be reset by publishing to a topic (/syscommand). This is done in favor of stopping and starting Gmapping with the roslaunch API after each episode as the Gmapping processes were not dying correctly with the latter approach.

## Algorithms
Multiple stages of algorithms were implemented in the course of this project:

* Deep Q-learning
* target network deep Q-learning
* Image based CNN Deep Q-learning

Each of these approaches can be implemented by editing ### in [file](/src).
Each algorithm had a specific goal or was used to fix the shortcomings of another algorithm for a specific goal.

For all algorithms the action space is 8 equally spaced movements. 
i.e. N,NE,E,SE,S,SW,W,NW

In order to reduce the state size of LIDAR observations, some of the algorithms below make use of segmenting and/or binning the LIDAR readings. A figure showing an example of this can be seen below:

!!!!! image of segments !!!!!

For the CNN implementation images of the robots current understanding of the map created by gmapping were fed to the CNN. These look like so:

!!!!! image of map !!!!!

### * Standard Q-learning /SARSA

**Applicable state representations:**
(x, y) co-ordinates of the world

**Learning goals and reward forms:**
Optimal exploration of a single environment, using a reward function that gives negative rewards for bumping objects and each time step passed and positive rewards for new map information gained.

**Overview:**
This implementation of Q-learning is only applicable to relatively small and discrete state and action spaces. 

If the LIDAR scans are segmented into 8 segments and binned into 5 discrete values this results in ???? states. Even if the number of states is reduced by recognizing the robot is omni directional so that states that are shifted around the circle are still equivalent there are still ??? states. This along with 8 actions for each state is an infeasible number of states to visit enough times to learn a policy. For this reason Q-learning with a table is only applicable to a state representation of discrete (x,y) co-ordinate values. 

This means that this algorithm implementation can realistically only be used to learn policies for individual static environments as LDIAR readings cannot be used in the state.

### * Deep Q-learning variations

**Applicable state representations:**
* (x,y) co-ordinates of the world
* full LIDAR data
* Segmented and/binned LIDAR data 
* Image of current understanding of map (Costmap output of Gmapping)

**Learning goals and reward forms:**
* Obstacle avoidance, using any form of LIDAR information as state with negative rewards for bumping into objects.
* Optimal exploration of a single environment (using x, y coordinates ), using a reward function that gives negative rewards for bumping objects and each time step passed and positive rewards for new map information gained.
* Optimal exploration of a arbitrary environments (Using any form of LIDAR as state or Image of current understanding of map), using a reward function that gives negative rewards for bumping objects and each time step passed and positive rewards for new map information gained.


**Overview of Deep Q-learning:**
Deep Q-Learning aims to solve issue of needing to store a Q-table in memory and visit all the states and try out actions from each state. This is achieved by using function approximators to approximate Q-values. This not only solves the memory issue of storing a Q-table but it also means that we do not need to exhaustively sample all possible states as the function approximator should be able to interpolate once it has been fitted well. The implementation of Q-learning in this project also makes use of memory replay.
Changing the structure of the Keras neural network model can be easily done and allows for the use of classic neural networks, convolutional neural networks and recurrent neural networks

**Overview of target network deep Q-learning**
Using a target network is a solution brought forward by Deep Mind. This approach is used for Deep Q-learning implementations that use Neural networks. This aims to stabilize the training of the neural network in order to assist convergence. The basic idea is to use two neural networks one to #### and the other to ###. With the target neural network being updated with the weights from the main network every c iterations, where c is a tunable parameter. A more in depth guide to this can be found at [link target network page](blah blah blah)

**Overview of double deep Q learning**



## Results

### Standard Q-learning /SARSA

### Deep Q-learning

### Target network deep Q learning

### Double deep Q learning


## Future work
* Dockerization
* Changing LIDAR range

A note on memory leaks
A memory leak type problem occurred where all the computers memory was eventually being used up by the long running code, this was a hard problem to trace and took up much time. As a result I discovered that spawning and deleting models in gazebo repetitively can cause gazebos memory usage to increase to avoid this, the same modle is imply respawned which caused the issue of it respawning in gazebo but not in the ROS ekf_localization node which needs to be respawned seperately useing the set_pose service. this incorrect respawing caused issues in the data structure that holds the map data. As the robot was drifting away from the fixed centre of the map frame which caused the map to keep grwoing to accomodate this even though most of the map was empty. Finally the roslaunch api was problematic in shutting down nodes, this could have been fixed but luckily a forked version of gmapping with the opton to reset the map has been created, so I used that instead in order to save time.
