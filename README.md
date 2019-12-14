# Jackal autonomous exploration
The aim of this project was to use reinforcement learning to develop an exploration policy for a mobile robot. This is a broad goal that can have many different learning goals. Some of these learning goals include:
* Learning basic obstacle avoidance using LIDAR data as state information
* Learning to explore a static environment using (x,y) coordinates as state information.
* Learning a general exploration policy for arbitrary environments using either LIDAR data or an image of the current understanding of the map as state information.

## Motivation
This project was completed as part of my final Masters in Robotics program at Northwestern university. This project was completed in partnership with a lab interested in the effect that increased range of sight owing to evolution had on the ability and necessity for animals to plan once they started to live on land where range of sight was greatly increased. This project aims to align with these interests by investigating the way that these learned policies change if the range of sight of the robot is changed.

For an explanation of the algorithms in more detail please see my [portfolio post](https://robo-jordo.github.io/portfolio/#portfolioModal12)

[Demo video](https://www.youtube.com/watch?v=YdH4bE5dZ6M)

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

In order to run reinforcement learning episodes safely, a gazebo simulation of the Jackal was used. Instructions for simulating the Jackal can be found on [ClearPaths website](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html).

I have modified various packages to suit the needs of my simulation.
I have modified and forked a version of the LMS1xx package from Clearpath.
[Here is my forked version](https://github.com/robo-jordo/LMS1xx).

A forked version of [slam_gmapping](https://github.com/jukindle/slam_gmapping) created by [jukindle](https://github.com/jukindle) was used in order to allow Gmapping to be reset by publishing to a topic (/syscommand). This is done in favor of stopping and starting Gmapping with the roslaunch API after each episode as the Gmapping processes were not dying correctly with the latter approach.

## Contents of repo:
#### launch
All these launch files launch gazebo headless without a GUI, this is an arg that can be changed when launching or the Gazebo gui can be started with 
```$ gzclient.```
I would also recommend using rviz to view the maps, scan and movements of the robot.
``` $roslaunch jackal_viz view_robot.launch config:=gmapping```

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

    This file uses ros parameters to determine which variant of the RL algorithm to run. The ros parameter /observation type can be set to "scan", "pos" or "img" to use the scan, positional or map image state versions respectively.

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

    This file can also be run as its own node in order to see the image being output from the Gmapping data with openCV.


## Installing this package and dependencies
I would recommend following the [ClearPath tutorials](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html) first to set up all their simulation packages. 

There are two forked packages that need to be used rather than their original versions. First check if you have the original versions in your catkin_workspace or installed via apt-get. This can be done with 
```
    $ rospack find gmapping
    $ rospack find lms1xx
```
To be safe if you have these already installed I would recommend removing them to be sure that you are using the correct version. If the result of the rospack commands above showed they were in a catkin workspace simply remove them from the workspace and run ```$ catkin_make```
If they were shown to be in some /opt/ros directory they can be removed with 
```
    $ sudo apt-get remove ros-melodic-gmapping
    $ sudo apt-get remove ros-melodic-slam-gmapping
    $ sudo apt-get remove ros-melodic-lms1xx
```

Once this has been done clone the two forked versions of those packages into the same catkin workspace as this package and run catkin_make.

```
    $ cd ~/jackal_ws/src
    $ git clone https://github.com/jukindle/slam_gmapping
    $ git clone https://github.com/robo-jordo/LMS1xx.git
    $ git clone https://github.com/robo-jordo/jackal_self_exploration.git
    $ cd ~/jackal_ws && catkin_make
```

## How to use

Once this package and its dependencies have been installed as instructed above. The different algorithms can be run from the launch files:

```
    $ source ~/jackal_ws/devel/setup.bash
    $ roslaunch jackal_self_exploration cnn.launch
```

To view gazebo and speed up the physics engine:
```
    $ gzclient
```

* Click the physics tab on the left panel and change the max step size parameter. I found that changing it from 0.001 to 0.005 increased the speed without degrading the accuracy of calculations.

To view the map and laser scan topics in rviz

```
    $ roslaunch jackal_viz view_robot.launch config:=gmapping
```

To run the gazebo environment with the single instruction movement node
```
    $ roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```

* In another terminal
```
    $ rosrun jackal_self_exploration points.py
```
* In yet another terminal 
```
    $ rosrun jackal_self_exploration move.py
```
* You may enter single directions such as N, NE etc..


## Future work
* Dockerization: It would be helpful to make a standardized docker container to bring up all the dependencies of his simulation for ease of use. Docker containers are also useful for running multiple simulations at once.

* Changing LIDAR range: Given more time exploring the differences in policies that may occur as a result of changing the the LIDAR range could be an interesting application of this project.
