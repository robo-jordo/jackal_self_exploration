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

for real world run:

* Jackal and 360 Lidar

## Contents of repo:
#### * launch
#### * src
#### * scripts


## Hardware
The platform used for this project is the ClearPath Jackal. This is owing to the fact that the Northwestern MSR lab has a ClearPath Jackal With a 3D Velodyne LIDAR and pointgrey camera available for use. This provides a platform with easily accessible sensors well integrated and maintained with ROS. Additionally CLearPath provide Gazebo simulation resources for the Jackal, which is very useful for reinforcement learning where harm may come to a real world robot. As a run up to this project I put together a [GitHub repo](https://github.com/robo-jordo/jackal_melodic_bringup) to bring a Jackal up with ROS melodic (ClearPath does not yet provide a melodic image for the Jackal) and the Velodyne LIDAR.


## Simulation set up and explanation

In order to use run reinforcement learning episodes safely, a gazebo simulation of the Jackal was used. Instructions for simulating the Jackal can be found on [ClearPaths website](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). Some other packages such as Gmapping were used for this project ## Deps ??? ##

I have modified various packages to suit the needs of my simulation.
The packages that I have modified have been forked to my GitHub account and are linked with ##DEPS###. These packages need to be cloned into a catkin workspace with this repo and built from source by running catkin_make.

A forked version of [slam_gmapping](https://github.com/jukindle/slam_gmapping) created by [jukindle](https://github.com/jukindle) was used in order to allow Gmapping to be reset by publishing to a topic (/syscommand). This is done in favor of stopping and starting Gmapping with the roslaunch API after each episode as I suspect that the Gmapping processes were not dying correctly with the latter approach.

## Algorithms
Multiple stages of algorithms were implemented in the course of this project:
* Standard Q-learning (Q-table based) /SARSA
* Deep Q-learning
* target network deep Q-learning target network
* Double deep Q learning

Each of these approaches has its own python file in /src.
Each algorithm had a specific goal or was used to fix the shortcomings of another algorithm for a specific goal.

For all algorithms the action space is 8 equally spaced movements. 
i.e. N,NE,E,SE,S,SW,W,NW

In order to reduce the state size of LIDAR observations, some of the algorithms below make use of segmenting and/or binning the LIDAR readings. A figure showing an example of this can be seen below:

!!!!! image of segments !!!!!

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

**Overview of target network deep Q-learning**

**Overview of double deep Q learning**


## Results

### Standard Q-learning /SARSA

### Deep Q-learning

### Target network deep Q learning

### Double deep Q learning


## Future work
* Dockerization
* CNN
* Changing LIDAR range