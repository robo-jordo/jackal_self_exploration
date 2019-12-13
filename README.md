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

## Hardware
The platform used for this project is the ClearPath Jackal. This is owing to the fact that the Northwestern MSR lab has a ClearPath Jackal With a 3D Velodyne LIDAR and pointgrey camera available for use. This provides a platform with easily accessible sensors that are well integrated and maintained with ROS. Additionally CLearPath provide Gazebo simulation resources for the Jackal, which is very useful for reinforcement learning where harm may come to a real world robot. As a run up to this project I put together a [GitHub repo](https://github.com/robo-jordo/jackal_melodic_bringup) to bring a Jackal up with ROS melodic  and the Velodyne LIDAR. (ClearPath does not yet provide a melodic image for the Jackal)

## Simulation set up and explanation

In order to run reinforcement learning episodes safely, a gazebo simulation of the Jackal was used. Instructions for simulating the Jackal can be found on [ClearPaths website](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). Some other packages such as Gmapping were used for this project ## Deps ??? ##

I have modified various packages to suit the needs of my simulation.
The LMS1xx package that I have modified has been forked to my GitHub account and are linked with ##DEPS###. This packages need to be cloned into a catkin workspace with this repo and built from source by running catkin_make.

A forked version of [slam_gmapping](https://github.com/jukindle/slam_gmapping) created by [jukindle](https://github.com/jukindle) was used in order to allow Gmapping to be reset by publishing to a topic (/syscommand). This is done in favor of stopping and starting Gmapping with the roslaunch API after each episode as the Gmapping processes were not dying correctly with the latter approach.

## Contents of repo:
#### launch

* **x_y.launch:**  
    Launches everything for a run of deep Q-learning using x,y position as state.

* **scan_segs.launch:**  
    Launches everything for a run of deep Q-learning using the averages of 8 scan segments as state.

* **cnn.launch:**  
    Launches everything for a run of deep Q-learning using images of the map from gmapping as state

#### src

* **ml_lib.py:**
    This file implements a MachineLearning class which can be imported by other scripts. This class aims to mirror the environments that are set up with [OpenAi gym](https://gym.openai.com/). This class implements things like resetting the environment, requesting an observation of state form the environment and commanding an action step to the environment.
    Calls to these functions can be seen in [algo.py](src/algo.py)

    This Class uses instances of the Movement class from [move.py](src/move.py) and MapImage from [mapper.py](src/mapper.py).

* **algo.py:**
    This file implements the actual reinforcement learning algorithms. This is done by creating an object of the MachineLearning class from the [ml_lib.py](src/ml_lib.py) file which allows for easy interaction with the Gazebo simulated environment.

    This file takes arguments to determine which variant of the RL algorithm to run.
    ## PUT ARGS HERE###

* **move.py:**
    This file implements the Movement class used to command movements of the robot in the Gazebo simulation. 

    This file can be run separately as a standalone node if a Gazebo simulation is running along with a [points node](src/points.py). If run as main the script prompts the user to pass the Gazebo simulation individual movement commands. The options are N,NE,E,SE,S,SW,W,NW.

* **points.py:**
    This file is intended to be run as its own node, there are no importable classes from this file.

    The node created by this file takes in a LaserScan ROS message on topic (/front/scan) and splits it into 8 segments which are then republished on their own topics (/oct<N>) along with a topic of their averages (/scan_avs) and a dedicated topic for the scan data from the 45 degree cone in front of the robot (/heading_scan).

* **mapper.py:**
    This file implements the MapImage class used in [algo.py](src/algo.py) to create images of the map data output by Gmapping.

    The image created is stored as an attribute of the class called horizontal_img and the class can simply be polled for the image after it has been instantiated.
    e.g.
    ```
    mi = MapImage()
    for i in range(10):
        image = mi.horizontal_img
    ```

    This file can also be run as its own node in order to see the image being output from the Gmapping data with openCV

    
## Installing this package and dependencies


## How to use


## Future work
* Dockerization
* Changing LIDAR range

A note on memory leaks
A memory leak type problem occurred where all the computers memory was eventually being used up by the long running code, this was a hard problem to trace and took up much time. As a result I discovered that spawning and deleting models in gazebo repetitively can cause gazebos memory usage to increase to avoid this, the same modle is imply respawned which caused the issue of it respawning in gazebo but not in the ROS ekf_localization node which needs to be respawned seperately useing the set_pose service. this incorrect respawing caused issues in the data structure that holds the map data. As the robot was drifting away from the fixed centre of the map frame which caused the map to keep grwoing to accomodate this even though most of the map was empty. Finally the roslaunch api was problematic in shutting down nodes, this could have been fixed but luckily a forked version of gmapping with the opton to reset the map has been created, so I used that instead in order to save time.
