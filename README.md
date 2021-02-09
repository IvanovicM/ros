# ros
Simple projects in Robot Operating System (ROS).

ROS is an open-source system used for programming robots. In this project you can find various packages implemented for *an autonomous robot*, specifically *the Turtlebot* (the Turtlebot2 was used in this project, which could be easily adapted for the Turtlebot3).

# What can I find here?

Note that *Gazebo* is used in all the packages for the world and the robot simulation (with an exception of *Intro to ROS*). The communication with it is achieved through the topic */cmd_vel* (for publishing commands) and through the topics */odom*  and */kobuki/laser/scan* (for collecting information).

## Intro to ROS

Firstly, you can find a very simple example with 4 types of ROS Nodes in the package *sensors*. The 4 types are:
* Publisher
* Subscriber
* Service
* Action

The way these nodes communicate is shown in the picture below. The node */measurement* gets and publishes the data from the sensors, while the node */processing* processes it and sends it further on.

<img src="images/intro_to_ros.png">

## Movement control

The robot movement control is implemented in the package *kinematics*, where you could find the controllers for both manual and autonomous movement.

To communicate with the controller just type in the mode you would like to use ('manual' or 'auto'), followed by a manually given velocity command (for manual mode) or the target position (for auto mode). Below you can find an example of the robot position before and after the commands sent by an user have been executed.

| <img src="images/kinematics.png" width="30%">|
|:---:|
| Robot's initial position (<img src="https://render.githubusercontent.com/render/math?math=x=0,y=0,\theta=0">) and target position (<img src="https://render.githubusercontent.com/render/math?math=x=1.5,y=2.5,\theta=\frac{\pi}{2}">) |

## Obstacles detection

An obstacles detection is implemented in the package *perception*. *Split and Merge* algorithm is applied to the data retrived from the laser scan (through the topic */kobuki/laser/scan*). This data is further used for extracting the lines of the walls.

If you ran *Rviz* in parallel, you could see the laser scan and visualized walls. On the following image(s) you can see the current position of a in the world, and all the detected walls visualized in Rviz.

| <img src="images/maze.png" width="80%">| <img src="images/walls.png" width="80%">|
|:---:|:---:|
| Robot's position in the world| Detected walls, visualized in Rviz |

## Kalman Filter

Kalman Filter Localization is implemented in the package *kalman*. The implementation is based on the [Autonomous Mobile Robots Course](https://asl.ethz.ch/education/lectures/autonomous_mobile_robots.html) from ETH Zurich.

Note that this package depends on the laser line extraction from [this repo](https://github.com/kam3k/laser_line_extraction).

# How to run these scripts?

To run any script, simply type in the following command(s) in a terminal:

```shell
  roscore # Type this only before running the very first script.
  rosrun <package> <script.py>
```

You can run any top-level script from the packages: *sensors*, *kinematics* and *perception*.

Note that if you wanted to perform a world and a robot simulation you should have **Gazebo**. Therefore, before running the aforementioned scripts you should run Gazebo by typing in the following command(s) in a terminal:

```shell
  roslaunch turtlebot_gazebo turtlebot_world.launch
```

If you want to visualize obstacles detected by the laser scan, simply type in the following command(a) in a terminal, in order to run **Rviz**. After that, add the  topics */kobuki/laser/scan* and */visualization_marker_array*.

```shell
  roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Have fun!

