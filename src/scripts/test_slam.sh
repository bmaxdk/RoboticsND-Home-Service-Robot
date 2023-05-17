#!/bin/sh

xterm -e "cd ../..; source/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/corridor.world" &

sleep 5

xterm -e "cd ../..; source/devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm -e "cd ../..; source/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm -e "cd ../..; source/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
