# RoboticsND-Home-Service-Robot

Home Service Robot in Kinetic ROS


# Shell Scripts
A ***shell script*** is a file containing a series of commands and could be executed. Use shell scripts when developing robotic software with different packages to avoid getting harder to track errors and bugs generated from different nodes. After you create a shell script file to launch one or many nodes each in separate terminals, you will have the power to track the output of different nodes and keep the convenience of running a single command to launch all nodes.

Install **xterm**: 'sudo apt-get install xterm'
## `launch.sh` Script
Goal here is to launch Gazebo and Rviz in separate instances of terminals.
```sh
#!/bin/sh
xterm  -e  " gazebo " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " rosrun rviz rviz" 
```
Here, in the `launch.sh` shell script launches three terminals and issues one or multiple commands in each terminal. To save my script file and give it ***execute*** permission: `chmod +x launch.sh`. Then to launch the shell: `./launch.sh`

# Simulation Set Up
