/*
----------------------------------------------------------
    @file: uav_model.cpp
    @date: Thu Jun 4, 2020
    @author: Alejandro Gonzalez
    @e-mail: alexglzg97@gmail.com
    @brief: Mathematical simulation of a quadrotor UAV.
    Open source
----------------------------------------------------------
*/

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace Eigen;

class DynamicModel
{
public:

	float time_step;
	float tau_phi;
	float tau_theta;
    float tau_psi;
    float U_1;

	static const float m;
	static const float Ixx;
	static const float Iyy;
	static const float Izz;
    static const float g;

	tf2::Quaternion myQuaternion;

	Vector3f d;
	Vector3f d_dot_last;
	Vector3f d_dot;
    Vector3f Phi;
    Vector3f Phi_dot;
    Vector3f Phi_dot_last;
    Vector3f w;
	Vector3f w_dot_last;
	Vector3f w_dot;
	Vector3f v;
	Vector3f v_dot_last;
	Vector3f v_dot;
	Vector3f e_3;
	Vector3f f;
    Vector3f tau;
	Matrix3f R;
    Matrix3f R_2;
    Matrix3f J;

	geometry_msgs::Pose pose;
	geometry_msgs::Twist vel;
	nav_msgs::Odometry odom;

	DynamicModel()
	{

		uav_pose_pub = n.advertise<geometry_msgs::Pose>("/uav_model/pose", 1000);
		uav_vel_pub = n.advertise<geometry_msgs::Twist>("/uav_model/vel", 1000);
		uav_odom_pub = n.advertise<nav_msgs::Odometry>("/uav_model/odom", 1000);

		force_sub = n.subscribe("/uav_control/force", 1000, &DynamicModel::force_callback, this);
		torque_sub = n.subscribe("/uav_control/torque", 1000, &DynamicModel::torque_callback, this);

		d << 0, 0, -5;
		d_dot_last << 0, 0, 0;
        Phi << 0, 0, 0;
        Phi_dot_last << 0, 0, 0;
		w << 0, 0, 0;
		w_dot_last << 0, 0, 0;
        v << 0, 0, 0;
		v_dot_last << 0, 0, 0;
        e_3 << 0, 0, 1;

		J << Ixx, 0, 0,
			0, Iyy, 0,
			0, 0, Izz;

	}

	void force_callback(const std_msgs::Float64::ConstPtr& force)
	{
		U_1 = force->data;
	}

	void torque_callback(const geometry_msgs::Vector3::ConstPtr& torque)
	{
		tau_phi = torque->x;
        tau_theta = torque->y;
        tau_psi = torque->z;
	}

	void step()
	{

		float phi = Phi(0);
		float theta = Phi(1);
		float psi = Phi(2);
        R << cos(psi)*cos(theta), -sin(psi)*sin(phi) + cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi),
			sin(psi)*cos(theta), cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi),
			-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);

        R_2 << 1, 0, -sin(theta),
			0, cos(phi), cos(theta)*sin(phi),
			0, -sin(phi), cos(theta)*cos(phi);

		tau << tau_phi, tau_theta, tau_psi;

        f = -U_1*e_3 + m*g*R.inverse()*e_3;
		if (d(2) >= 0){
			if (U_1 <= m*g){
				f = f - f;
			}
		}
		f = f - f;

		v_dot = (1/m)*f - w.cross(v);
        v = time_step*(v_dot + v_dot_last)/2 + v;
        v_dot_last = v_dot;

		if (d(2) >= 0){
			if (v(2) > 0){
				v = v - v;
			}
		}

        d_dot = R*v;
        d = time_step*(d_dot + d_dot_last)/2 + d;
        d_dot_last = d_dot;

		if (d(2) >= 0){
			d(2) = 0;
		}

        w_dot = J.inverse()*(tau - w.cross(J*w));
        w = time_step*(w_dot + w_dot_last)/2 + w;
		if (d(2) >= 0){
			w = w - w;
		}
		w_dot_last = w_dot;
        Phi_dot = R_2.inverse()*w;
        Phi = time_step*(Phi_dot + Phi_dot_last)/2 + Phi;
        Phi_dot_last = Phi_dot;

        for (int i=0; i <3; i++){
            if (abs(Phi(i)) > 3.141592){
			    Phi(i) = (Phi(i)/abs(Phi(i)))*(abs(Phi(i))-2*3.141592);
		    }
        }

		pose.position.x = d(0);
		pose.position.y = d(1);
		pose.position.z = d(2);
		odom.pose.pose.position.x = d(0);
		odom.pose.pose.position.y = d(1);
		odom.pose.pose.position.z = d(2);

		myQuaternion.setRPY(Phi(0),Phi(1),Phi(2));

		pose.orientation.x = Phi(0);
		pose.orientation.y = Phi(1);
		pose.orientation.z = Phi(2);
		odom.pose.pose.orientation.x = myQuaternion[0];
		odom.pose.pose.orientation.y = myQuaternion[1];
		odom.pose.pose.orientation.z = myQuaternion[2];
		odom.pose.pose.orientation.w = myQuaternion[3];

		vel.linear.x = v(0);
		vel.linear.y = v(1);
		vel.linear.z = v(2);
		odom.twist.twist.linear.x = v(0);
		odom.twist.twist.linear.y = v(1);
		odom.twist.twist.linear.z = v(2);

		vel.angular.x = w(0);
		vel.angular.y = w(1);
		vel.angular.z = w(2);
		odom.twist.twist.angular.x = w(0);
		odom.twist.twist.angular.y = w(1);
		odom.twist.twist.angular.z = w(2);

		//Data publishing
		uav_pose_pub.publish(pose);
		uav_vel_pub.publish(vel);
		uav_odom_pub.publish(odom);
	}

private:
	ros::NodeHandle n;

	ros::Publisher uav_pose_pub;
	ros::Publisher uav_vel_pub;
	ros::Publisher uav_odom_pub;

	ros::Subscriber force_sub;
	ros::Subscriber torque_sub;

};

const float DynamicModel::m = 2;
const float DynamicModel::Ixx = 0.0081;
const float DynamicModel::Iyy = 0.0081;
const float DynamicModel::Izz = 0.0142;
const float DynamicModel::g = 9.81;

//Main
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "uav_model");
	DynamicModel dynamicModel;
	dynamicModel.time_step = 0.01;
	dynamicModel.U_1 = 0;
	int rate = 100;
	ros::Rate loop_rate(rate);

  while (ros::ok())
  {
	dynamicModel.step();
	ros::spinOnce();
	loop_rate.sleep();
  }

	return 0;
}