# Home-Service_Robot

## A part of Udacity Nanodegree Program

### Introduction
This project will be based on a Home Service Robot. The robot moves to point A and picks up the box, then it delivers it to the goal location.

### Description
This Home Service Robot interfaces with different ROS packages to obtain the required functionality. Some of these packages are official ROS packages which offer great tools.

The list of the official ROS packages used in the project, and other packages and directories that was created in the implementation.

**Official ROS packages**

**gmapping:** With the gmapping_demo.launch file, we will perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.

**turtlebot_teleop:** With the keyboard_teleop.launch file, we can manually control a robot using keyboard commands.

**turtlebot_rviz_launchers:** With the view_navigation.launch file, we will load a preconfigured rviz workspace. We’ll be saving a lot of time by launching this file, because it will automatically load the robot model, trajectories, and the map.

**turtlebot_gazebo:** With the turtlebot_world.launch, we can deploy a turtlebot in a gazebo environment by linking the world file to it.

**Your Packages and Directories**

Custom built ROS Packages and Directories for the Home Service Robot.

**map:** This directory contains the gazebo world file and the map generated from SLAM.

**scripts:** This directory will store the shell scripts.

**rvizConfig:** This directory contains the customized rviz configuration files.

**pick_objects:** This node will command the robot to drive to the pickup and drop off zones.

**add_markers:** This node will model the object with a marker in rviz.
