#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" & #world_file:=$(rospack find my_robot)/world/MapMyWorld.world" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" & #map_file:=$(rospack find my_robot)/world/MapMyWorld.yaml " &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" 
sleep 5

xterm -e " rosrun add_markers add_markers_node.cpp"

