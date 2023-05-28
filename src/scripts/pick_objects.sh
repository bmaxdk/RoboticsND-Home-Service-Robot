#!/bin/sh

ws="/home/workspace/catkin_ws"

xterm -e "cd ${ws} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${ws}/src/map/cho_world.world" &
sleep 6

xterm -e "cd ${ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${ws}/src/map/map.yaml" &
sleep 6

xterm -e "cd ${ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 6

xterm -e "cd ${ws} && source devel/setup.bash && rosrun pick_objects pick_objects"
