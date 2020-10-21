#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "gazebo_msgs/ModelState.h"

class States{
public:
  gazebo_msgs::ModelState states;
  States(){
  gazebo_pub = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  sub = node.subscribe("/uav_model/odom", 1000, &States::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

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
    
    states.model_name = "parrot";

    states.pose.position.x = msg->pose.pose.position.x;
    states.pose.position.y = -msg->pose.pose.position.y;
    states.pose.position.z = -msg->pose.pose.position.z;

    states.pose.orientation.x = q.x();
    states.pose.orientation.y = q.y();
    states.pose.orientation.z = q.z();
    states.pose.orientation.w = q.w();

    /*states.pose.orientation.x = msg->pose.pose.orientation.x;
    states.pose.orientation.y = pose.pose.orientation.y;
    states.pose.orientation.z = msg->pose.pose.orientation.z;
    states.pose.orientation.w = msg->pose.pose.orientation.w);*/

    gazebo_pub.publish(states);

  }

private:
  ros::NodeHandle node;
  ros::Publisher gazebo_pub;
  ros::Subscriber sub;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "uav_gazebo_broadcaster");
  States states_;

  while (ros::ok()){
    ros::Rate(100).sleep();
    ros::spinOnce();
  }

  return 0;
}