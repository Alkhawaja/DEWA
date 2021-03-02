/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <nav_msgs/Odometry.h>


static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;


void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped poseStamped;

//pose.pose.position.x,
 
      poseStamped.pose.position.x = 10;
      poseStamped.pose.position.y = 0;
      poseStamped.pose.position.z = 0;

      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 0;
      poseStamped.pose.orientation.z = 0;
      poseStamped.pose.orientation.w = 1;

    odom_pub.publish(poseStamped);
  }


int main (int argc, char **argv) {
  ros::init(argc, argv, "Waypoints_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  odom_pub = node.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 10);

  ros::Subscriber odom = node.subscribe("robot_1/odom", 10, callback);

  ros::spin();
}

