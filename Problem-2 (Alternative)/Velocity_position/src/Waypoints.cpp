/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <nav_msgs/Odometry.h>

int x=0;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;


void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped poseStamped;
    /*
      poseStamped.pose.position.z = 0;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 0;
      poseStamped.pose.orientation.z = 0;
  
 if (msg->pose.pose.position.x>=-0.1 && msg->pose.pose.position.x<0.1 && msg->pose.pose.position.y>=-0.1 && msg->pose.pose.position.y<0.1 && x==0){
    ROS_INFO("One");
      poseStamped.pose.position.x = 2;
      poseStamped.pose.position.y = 0;
      poseStamped.pose.orientation.w = 1;  
      odom_pub.publish(poseStamped);x=1;
}
  if (msg->pose.pose.position.x>=1.7 && msg->pose.pose.position.x<2.3 && msg->pose.pose.position.y>=0 && msg->pose.pose.position.y<0.5 && x==1){
    ROS_INFO("Two");
      poseStamped.pose.position.x = 2;
      poseStamped.pose.position.y = 2;
      poseStamped.pose.orientation.w = 1;  
      odom_pub.publish(poseStamped);x=2;
}
if (msg->pose.pose.position.x>=1.7 && msg->pose.pose.position.x<2.3 && msg->pose.pose.position.y>=1.7 && msg->pose.pose.position.y<2.3 && x==2){
    ROS_INFO("Two");
      poseStamped.pose.position.x = 0;
      poseStamped.pose.position.y = 2;
      poseStamped.pose.orientation.w = 1;  
      odom_pub.publish(poseStamped);x=3;
}

 if (msg->pose.pose.position.x>=0 && msg->pose.pose.position.x<0.1 && msg->pose.pose.position.y>=1.7 && msg->pose.pose.position.y<2.3 && x==3){
    ROS_INFO("Four");
      poseStamped.pose.position.x = 2;
      poseStamped.pose.position.y = 2;
      poseStamped.pose.orientation.w = 1;  }*/
    if(x==0)
{    poseStamped.header.frame_id="robot_1/map";
      poseStamped.pose.position.x = 2;
      poseStamped.pose.position.y = 2;
      poseStamped.pose.orientation.w = 1;  
      poseStamped.pose.position.z = 0;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 0;
      poseStamped.pose.orientation.z = 0;

      odom_pub.publish(poseStamped);
x=1;}

}



int main (int argc, char **argv) {
  ros::init(argc, argv, "Waypoints_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  odom_pub = node.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 10);

  ros::Subscriber odom = node.subscribe("robot_1/odom", 10, callback);

  ros::spin();
}

