#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>

using namespace std;

class AddMarkers
{
public:
    AddMarkers()
    {
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("waypoints", 1);
        stage_sub = n.subscribe("stage", 1, &AddMarkers::stageCallback, this);

        vector<vector<double>> m = {{5.0, -6.0}, {12.0, -2.0}, {0.0, 0.0}};
        
        for(auto &wp : m) 
        {
            geometry_msgs::PoseStamped waypoint;
            waypoint.header.frame_id = "map";
            waypoint.pose.position.x = wp[0];
            waypoint.pose.position.y = wp[1];
            waypoint.pose.position.z = 0;
            waypoint.pose.orientation.w = 1.0;
            waypoints.push_back(waypoint);
        }

        // Initialize markers
        initializeMarker(marker1, 0, m[0][0], m[0][1]);
        initializeMarker(marker2, 1, m[1][0], m[1][1]);
        
        marker1.type = visualization_msgs::Marker::CYLINDER;
        marker1.color.g = 0.0f;
        marker1.color.b = 1.0f;

        // Publish the markers and the first waypoint
        publishMarker(marker1);
        publishMarker(marker2);
        if (waypoints.size() > 0)
            waypoint_pub.publish(waypoints[0]);
    }

private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker1, marker2;
    ros::Publisher waypoint_pub;
    vector<geometry_msgs::PoseStamped> waypoints;
    ros::Subscriber stage_sub;

    void stageCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (msg->data == "First Waypoint") 
        {
            marker1.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker1);
            ROS_INFO("First Package is picked up.");

            waypoint_pub.publish(waypoints[1]);

        } 
        else if (msg->data == "Second Waypoint") 
        {
            marker2.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker2);
            ROS_INFO("Second Package is picked up.");

            waypoint_pub.publish(waypoints[2]);
        } 
        else if (msg->data == "Third Waypoint") 
        {   
            marker1.pose.position.x = marker2.pose.position.x = waypoints[2].pose.position.x;
            marker1.pose.position.y = waypoints[2].pose.position.y +0.5;
            marker2.pose.position.y = waypoints[2].pose.position.y +1.;
            marker1.action = marker2.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker1);
            marker_pub.publish(marker2);
            ROS_INFO("Successfuly drop off packages.");

        }
    }

    void initializeMarker(visualization_msgs::Marker& marker, int id, double x, double y)
    {
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
        marker.id = id;


        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;
        
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
    }

    void publishMarker(visualization_msgs::Marker& marker)
    {
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers");
    AddMarkers add_markers;
    ros::spin();
    return 0;
}
