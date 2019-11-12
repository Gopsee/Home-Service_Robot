#!/bin/sh

#Launch turtlebot in MapMyWorld
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/world/world.world" &
sleep 5

#Launch Gmapping
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

#Launch rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#Launch turtlebot teleop
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"


