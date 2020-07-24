#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"


class ProportionalIntegralDerivative
{
public:
  //Thruster outputs
  float tau_phi;
  float tau_theta;
  float tau_psi;

  //Sensor feedback
  float phi;
  float theta;
  float psi;
  float phi_dot;
  float theta_dot;
  float psi_dot;

  static const float time_step = 0.001;

  //Tracking variables
  float phi_d;
  float theta_d;
  float psi_d;

  //Auxiliry variables
  float phi_d_dot;
  float theta_d_dot;
  float psi_d_dot;
  float phi_d_last;
  float theta_d_last;
  float psi_d_last;

  //Controller gains
  static const float kp_phi = 0.2;
  static const float kd_phi = 0.2;
  static const float kp_theta = 0.2;
  static const float kd_theta = 0.2;
  static const float kp_psi = 2.0;
  static const float kd_psi = 3.0;

  ProportionalIntegralDerivative()
  {
    //ROS Publishers for each required sensor data
    uav_torque_pub = n.advertise<geometry_msgs::Vector3>("/uav_control/torque", 1000);
    error_pub = n.advertise<geometry_msgs::Vector3>("/uav_control/attitude_error", 1000);

    desired_attitude_sub = n.subscribe("/guidance/desired_attitude", 1000, &ProportionalIntegralDerivative::desiredAttitudeCallback, this);
    pose_sub = n.subscribe("/uav_model/pose", 1000, &ProportionalIntegralDerivative::poseCallback, this);
    vel_sub = n.subscribe("/uav_model/vel", 1000, &ProportionalIntegralDerivative::velocityCallback, this);

    phi = 0;
    theta = 0;
    psi = 0;
    phi_d = 0;
    theta_d = 0;
    psi_d = 0;
    phi_dot = 0;
    theta_dot = 0;
    psi_dot = 0;
    phi_d_dot = 0;
    theta_d_dot = 0;
    psi_d_dot = 0;
  }

  void desiredAttitudeCallback(const geometry_msgs::Vector3::ConstPtr& _attitude_d)
  {
    phi_d = _attitude_d -> x;
    theta_d = _attitude_d -> y;
    psi_d = _attitude_d -> z;
  }

  void poseCallback(const geometry_msgs::Pose::ConstPtr& _pose)
  {
    phi = _pose -> orientation.x;
    theta = _pose -> orientation.y;
    psi = _pose -> orientation.z;
  }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& _vel)
  {
    phi_dot = _vel -> angular.x;
    theta_dot = _vel -> angular.y;
    psi_dot = _vel -> angular.z;
  }

  void control()
  {
    
    float e_phi = phi_d - phi;
    float e_theta = theta_d - theta;
    float e_psi = psi_d - psi;

    phi_d_dot = (phi_d - phi_d_last) / time_step;
    phi_d_last = phi_d;
    theta_d_dot = (theta_d - theta_d_last) / time_step;
    theta_d_last = theta_d;
    psi_d_dot = (psi_d - psi_d_last) / time_step;
    psi_d_last = psi_d;

    if (std::abs(e_phi) > 3.141592){
        e_phi = (e_phi/std::abs(e_phi))*(std::abs(e_phi)-2*3.141592);
    }
    if (std::abs(e_theta) > 3.141592){
        e_theta = (e_theta/std::abs(e_theta))*(std::abs(e_theta)-2*3.141592);
    }
    if (std::abs(e_psi) > 3.141592){
        e_psi = (e_psi/std::abs(e_psi))*(std::abs(e_psi)-2*3.141592);
    }
    
    float e_phi_dot = 0 - phi_dot;
    float e_theta_dot = 0 - theta_dot;
    float e_psi_dot = psi_d_dot - psi_dot;

    tau_phi = (kp_phi * e_phi) + (kd_phi * e_phi_dot);
    tau_theta = (kp_theta * e_theta) + (kd_theta * e_theta_dot);
    tau_psi = (kp_psi * e_psi) + (kd_psi * e_psi_dot);
  
    //Data publishing
    geometry_msgs::Vector3 torques;
    geometry_msgs::Vector3 errors;

    torques.x = tau_phi;
    torques.y = tau_theta;
    torques.z = tau_psi;
    errors.x = e_phi;
    errors.y = e_theta;
    errors.z = e_psi;

    uav_torque_pub.publish(torques);
    error_pub.publish(errors);
  }


private:
  ros::NodeHandle n;

  ros::Publisher uav_torque_pub;
  ros::Publisher error_pub;

  ros::Subscriber desired_attitude_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber vel_sub;

};

//Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "uav_pid");
  ProportionalIntegralDerivative proportionalIntegralDerivative;
  int rate = 1000;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    proportionalIntegralDerivative.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}