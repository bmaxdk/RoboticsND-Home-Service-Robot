#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <map>
#include <string>

using namespace std;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBaseSequence {
public:
    //tell the action client that we want to spin a thread by default
    MoveBaseSequence() : ac_("move_base", true), count(0) 
    {
        // Setting up my waypoints
        waypoints = {{5.0, -6.0}, {12.0, -2.0}, {0.0, 0.0}};
        station_map = {{0, "First Waypoint"}, {1, "Second Waypoint"}, {2, "Third Waypoint"}};
        // station_map[0] = "First Waypoint";
        // station_map[1] = "Second Waypoint";
        // station_map[2] = "Third Waypoint";

        // Wait 5 sec for move_base action server to come up
        // while(!ac_.waitForServer(ros::Duration(5.0))) 
        //     ROS_INFO("Waiting for the move_base action server to come up");
        ROS_INFO("Waypoints will be: \n");
        int i = 0;
        for (auto &wp : waypoints)
            ROS_INFO("[%s] (x, y) = (%f, %f)\n", station_map[i++].c_str(), wp[0], wp[1]);

        ROS_INFO("Waiting for the move_base action server to come up");
        ros::Duration(5.0).sleep();
    }

    void moveToWaypoints() 
    {
        for(auto &wp : waypoints) 
        {
            move_base_msgs::MoveBaseGoal goal;

            // set up the frame parameters
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Define a position and orientation for the robot to reach
            goal.target_pose.pose.position.x = wp[0];
            goal.target_pose.pose.position.y = wp[1];
            goal.target_pose.pose.orientation.w = 1.0;

            // Send the goal position and orientation for the robot to reach
            ROS_INFO("Wait for 5.0 seconds before sending %s.\n", station_map[count].c_str());
            ros::Duration(5.0).sleep();
            ROS_INFO("Now Sending %s goal!\n", station_map[count].c_str());

            ac_.sendGoal(goal);

            // Wait an infinite time for the results
            ac_.waitForResult();

            // Check if the robot reached its goal
            if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray! It reached to %s. The base moved  x = %f, y = %f\n", station_map[count].c_str(), wp[0], wp[1]);
            else
                ROS_INFO("The base failed to reach %s for some reason", station_map[count].c_str());
            count++;
        }
    }
    ~MoveBaseSequence()
    {
        ROS_INFO("Completed the task");
    }

private:
    MoveBaseClient ac_;
    vector<vector<double>> waypoints;
    map<uint, string> station_map;
    uint count;
};

int main(int argc, char** argv) 
{
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    MoveBaseSequence mbs;
    mbs.moveToWaypoints();

    return 0;
}
