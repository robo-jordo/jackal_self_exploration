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

#### Standard Q-learning /SARSA
######Applicable state representation:
######Learning goals and reward form:
######Overview:

#### Deep Q-learning
#####Applicable state representation:
#####Learning goals and reward form:
#####Overview:

#### Target network deep Q learning
######Applicable state representation:
######Learning goals and reward form:
######Overview:

#### Double deep Q learning
######Applicable state representation:
######Learning goals and reward form:
######Overview:


## Results
#### Standard Q-learning /SARSA

#### Deep Q-learning

#### Target network deep Q learning

#### Double deep Q learning


## Future work