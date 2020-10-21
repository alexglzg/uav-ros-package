#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

class TfAndPath{
public:
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  TfAndPath(){
  path_pub = node.advertise<nav_msgs::Path>("/uav_ros_pkg/broadcaster/uav_path", 1000);
  sub = node.subscribe("/uav_model/odom", 1000, &TfAndPath::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "uav";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = -msg->pose.pose.position.y;
    transformStamped.transform.translation.z = -msg->pose.pose.position.z;
    
    tf2::Quaternion q_orig(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Quaternion q_rot, q;

    double r=3.141592, p=0, y=0;
    q_rot.setRPY(r,p,y);

    q = q_rot*q_orig;
    q.normalize();
    
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose.position.y = -msg->pose.pose.position.y;
    pose.pose.position.z = -msg->pose.pose.position.z;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.push_back(pose);
    
    path_pub.publish(path);
  }

private:
  ros::NodeHandle node;
  ros::Publisher path_pub;
  ros::Subscriber sub;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "uav_tf2_broadcaster");
  TfAndPath tfAndPath;

  while (ros::ok()){
    ros::Rate(100).sleep();
    ros::spinOnce();
  }

  return 0;
}