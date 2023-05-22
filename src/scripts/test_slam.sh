#!/bin/sh

# source /opt/ros/kinetic/setup.bash
# source devel/setup.bash

ws=/home/workspace/catkin_ws/
# xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/playground.world" &
xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${ws}/src/map/cho_world.world" &
# xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/cho_world.world" &
# xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_gazebo turtlebot_world.launch"&
sleep 5

xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e "cd ${ws}; source/devel/setub.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
