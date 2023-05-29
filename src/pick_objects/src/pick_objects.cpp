#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

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
        // waypoints = {{5.0, -6.0}, {12.0, -2.0}, {0.0, 0.0}};
        station_map = {{0, "First Waypoint"}, {1, "Second Waypoint"}, {2, "Third Waypoint"}};
 
        ROS_INFO("Waiting for the move_base action server to come up");
        ros::Duration(5.0).sleep();
        ROS_INFO("Waiting for waypoint..");
        waypoint_sub_ = n.subscribe("waypoints", 1, &MoveBaseSequence::moveToWaypointsCallback, this);
        stage_pub_ = n.advertise<std_msgs::String>("stage", 1);
    }
    void moveToWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        ROS_INFO("Received %s message: x: %f, y: %f", station_map[count].c_str(), msg->pose.position.x, msg->pose.position.y);

        move_base_msgs::MoveBaseGoal goal;

        // set up the frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        // Define a position and orientation for the robot to reach
        goal.target_pose.pose.position.x = msg->pose.position.x;
        goal.target_pose.pose.position.y = msg->pose.position.y;
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
        {
            ROS_INFO("Hooray! It reached to %s. The base moved  x = %f, y = %f\n", station_map[count].c_str(), msg->pose.position.x, msg->pose.position.y);

            std_msgs::String msg;
            msg.data = station_map[count++];
            stage_pub_.publish(msg);
        }
        else
            ROS_INFO("The base failed to reach %s for some reason", station_map[count].c_str());
        
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
    ros::NodeHandle n;
    ros::Subscriber waypoint_sub_;
    ros::Publisher stage_pub_;
};

int main(int argc, char** argv) 
{
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    MoveBaseSequence mbs;
    ros::spin();

    return 0;
}
