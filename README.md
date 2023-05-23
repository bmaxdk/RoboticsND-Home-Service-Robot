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

# SLAM Testing
shell script `test_slam.sh` that will deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in `rviz`.
```bash
$ source devel/setup.bash
$ cd src/script
$ ./test_slam.sh
```
Use `keyboard_teleop` node to map the world.

# Localization and Navigation Testing
![Localization and Navigation Testing][image1]
<!-- ![Localization and Navigation Testing2][image2] -->

[//]: # (Image References)
[image1]: https://github.com/bmaxdk/RoboticsND-Home-Service-Robot/blob/main/img/Nav3.gif "Localization and Navigation Testing"
<!-- [image2]: https://github.com/bmaxdk/RoboticsND-Home-Service-Robot/blob/main/img/a.gif "Localization and Navigation Testing2" -->

RTAB-Map is SLAM algorithm that can be used to create a map of an environment. [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) package for simultaneous localization and mapping (SLAM) has some advantages over the gmapping package, such as the ability to create 3D maps and handle loop closures more effectively.

To save the RTAM-Map 2D or 3D:
* Open the database: Run RTAB-Map standalone application (just type rtabmap in your terminal). Once the GUI is open, click on "File" -> "Open" in the top menu and select your map.db file.
* Export the 2D map: Go to "File" -> "Export 2D grid map..." and a dialogue window will open. In this window, you can adjust parameters such as the resolution of your map. Click "OK" and then you will be asked to specify the path and name of the output file. You can save the map as .png or .pgm, with an accompanying .yaml file for map metadata.
* Export the 3D map: If you want to export a 3D map (if you've been mapping in 3D), go to "File" -> "Export 3D cloud..." or "File" -> "Export 3D mesh...". This will allow you to save the 3D map in various formats (like .pcd, .ply, .vtk, etc.).

To run:
```bash
$ source devel/setup.bash
$ cd src/script
$ ./test_navigation.sh
```

**Robot Position Updated** in [amcl_demo.launch](https://github.com/bmaxdk/RoboticsND-Home-Service-Robot/blob/main/src/turtlebot_simulator/turtlebot_gazebo/launch/amcl_demo.launch) to correct map where you can find in 'localization' section.
* More detalils about creating Map

# Navigation Goal Node
## Sending Goals to the Navigation Stack
The ROS navigation stack creates a path for your robot based on `Dijkstra's algorithm`, a variant of the **Uniform Cost Search algorithm**, while avoiding obstacles on its path.
[Tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals): Official ROS tutorial that teaches you how to send a single goal position and orientation to the navigation stack.
**Sample Code**
```cpp
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
```