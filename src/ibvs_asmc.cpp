#include <iostream>
#include <math.h>
#include <fstream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>


Eigen::Matrix3d R_psi_T(double psi)
{
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(psi), -sin(psi), 0,
             sin(psi), cos(psi), 0,
             0.0, 0.0, 1.0;

     return R_yaw.transpose();
}

Eigen::Vector2d Centroid(Eigen::MatrixXd Virtual_pix)
{
    double m10 = 0;
    double m01 = 0;
    Eigen::VectorXd u = Virtual_pix.row(0);
    Eigen::VectorXd n = Virtual_pix.row(1);

    double ug = u.sum() / u.size();
    double ng = n.sum() / u.size();

    Eigen::Vector2d Center;
    Center << ug, ng;

    return Center;
}

Eigen::Vector4d ImFeat(Eigen::Vector2d Centroid, Eigen::MatrixXd Virtual_pixels, double focal_length, double a_desired)
{
    Eigen::VectorXd u = Virtual_pixels.row(0);
    Eigen::VectorXd n = Virtual_pixels.row(1);
    double mu20 = 0;
    double mu02 = 0;
    double mu11 = 0;

    for (int i = 0; i < u.size(); i++)
    {
        double mu_20 = pow((u(i) - Centroid(0)), 2);
        double mu_02 = pow((n(i) - Centroid(1)), 2);
        double mu_11 = (u(i) - Centroid(0)) * (n(i) - Centroid(1));
        
        mu20 = mu20 + mu_20;
        mu02 = mu02 + mu_02;
        mu11 = mu11 + mu_11;
    }

    double a = mu20 + mu02;
    double qz = sqrt(a_desired / a);
    double qx = qz * (Centroid(0) / focal_length);
    double qy = qz * (Centroid(1) / focal_length);
    double q_psi = 0.5 * atan(2 * mu11 / (mu20 - mu02));

    Eigen::Vector4d ImageFeatures;
    ImageFeatures<< qx, qy, qz, q_psi;

    return ImageFeatures;
}

Eigen::Vector4d ImFeat_dot(Eigen::Vector4d ImFeat, Eigen::Vector4d UAV_velocity, Eigen::Vector4d Tgt_velocity, double Upsilon_est)
{
    Eigen::Vector3d Feat_linear;
    Feat_linear << ImFeat(0), ImFeat(1), ImFeat(2);

    Eigen::Vector3d UAV_linear_velocity;
    Eigen::Vector3d Tgt_linear_velocity;

    UAV_linear_velocity << UAV_velocity(0), UAV_velocity(1), UAV_velocity(2);
    Tgt_linear_velocity << Tgt_velocity(0), Tgt_velocity(1), Tgt_velocity(2);

    Eigen::Matrix3d sk_psi;
    sk_psi << 0, -UAV_velocity(3), 0,
        UAV_velocity(3), 0, 0,
        0, 0, 0;

    Eigen::Vector3d qlin_dot;
    qlin_dot = -sk_psi * Feat_linear - Upsilon_est * UAV_linear_velocity + Upsilon_est * Tgt_linear_velocity;

    double qpsi_dot = Tgt_velocity(3) - UAV_velocity(3);

    Eigen::Vector4d Image_Features_dot;
    Image_Features_dot << qlin_dot, qpsi_dot; 

    return Image_Features_dot;

}

int sign(double var)
{
    int res;
    if (var > 0)
    {
        res = 1;
    }

    if (var < 0) 
    {
        res = -1;
    }

    if (var == 0) 
    {
        res = 0;
    }

    return res;
}

Eigen::Vector4d error_dot(double Upsilon_estimated, Eigen::Vector4d Features_vector, Eigen::Vector4d xi, Eigen::Vector4d kappa)
{
    Eigen::Vector4d e_dot;
    Eigen::Matrix4d Lambda;
    Lambda << -Upsilon_estimated, 0, 0, Features_vector(1),
        0, -Upsilon_estimated, 0, -Features_vector(0),
        0, 0, -Upsilon_estimated, 0,
        0, 0, 0, -1;
    e_dot << Lambda * xi + kappa;

    return e_dot;
}

double Upsilon_estimate_dot(double gamma, Eigen::Vector4d xi, Eigen::Vector4d xi_dot, Eigen::Matrix4d I, double eta1, Eigen::Matrix4d eta2, Eigen::Matrix4d Gamma, Eigen::Vector4d delta)
{
    double Upsilon_est_dot = (1 / gamma) * (xi_dot.transpose() * I * eta1 + xi.transpose() * I * eta2) * Gamma * delta;
    

    return Upsilon_est_dot;
}


Eigen::Matrix4d Lambda_est(double Upsilon_estimate, Eigen::Vector4d Features_vector)
{
    Eigen::Matrix4d Lambda_estimated;
    Lambda_estimated << -Upsilon_estimate, 0, 0, Features_vector(1),
                        0, -Upsilon_estimate, 0, -Features_vector(0),
                        0, 0, -Upsilon_estimate, 0,
                        0, 0, 0, -1;

    return Lambda_estimated;
}

Eigen::Vector4d Xi_dot(Eigen::Matrix4d Lambda_estimada, Eigen::Matrix4d Lambda_estimada_punto, double eta1, Eigen::Matrix4d eta2, double r1, double r2, double r3, Eigen::Vector4d delta, Eigen::Matrix4d Gamma, Eigen::Vector4d Pii, Eigen::Vector4d rho, Eigen::Vector4d xi)
{
    Eigen::Matrix4d a = eta1 * Lambda_estimada;
    Eigen::Vector4d b = (-r1 * delta) + (Gamma.inverse() * r2 * Pii) + (r3 * rho);
    Eigen::Matrix4d c = eta1 * Lambda_estimada_punto + eta2 * Lambda_estimada;

    Eigen::Vector4d xi_dot = a.inverse() * (b - c * xi);

    return xi_dot;

}

Eigen::Matrix3d Rtp(double pitch, double roll)
{
    Eigen::Matrix3d pitch_mat;
    pitch_mat << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);


    Eigen::Matrix3d roll_mat;
    roll_mat << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    Eigen::Matrix3d R_tp;
    R_tp = pitch_mat * roll_mat;

    return R_tp;

}

Eigen::Vector3d Force(Eigen::Vector4d xi_punto, Eigen::Vector4d xi, double psi_p, double uav_mass)
{
    Eigen::Matrix3d skew;
    Eigen::Vector3d xi_linear;
    Eigen::Vector3d xi_punto_linear;

    Eigen::Vector3d F;

    skew << 0, -psi_p, 0,
        -psi_p, 0, 0,
        0, 0, 0;

    xi_punto_linear << xi_punto(0), xi_punto(1), xi_punto(2);
    xi_linear << xi(0), xi(1), xi(2);

    F = uav_mass * xi_punto_linear + uav_mass * skew * xi_linear;

    return F;
}

double Thrust(double pitch, double roll, double uav_mass, Eigen::Vector3d Force)
{
    Eigen::RowVector3d e3;
    Eigen::Matrix3d R_tp;
    Eigen::Vector3d mg;

    e3 << 0, 0, 1;
    mg << 0, 0, uav_mass * 9.81;
    R_tp = Rtp(pitch, roll);

    double Th = e3 * R_tp.transpose() * (mg - Force);

    return Th;
}

double Desired_roll(Eigen::Vector3d Force, double Thrust)
{
    double Fy = Force(1);
    double phi_des = asin(Fy / Thrust);
    return phi_des;

}

double Desired_pitch(Eigen::Vector3d Force, double Thrust, double Roll_Desired)
{
    double Fx = Force(0);
    double theta_des = sin(-Fx / (Thrust * cos(Roll_Desired)));
    return theta_des;

}

Eigen::Matrix3d RotationMatrix(double roll, double pitch, double yaw)
{
    double cx = cos(roll);
    double sx = sin(roll);
    double cy = cos(pitch);
    double sy = sin(pitch);
    double cz = cos(yaw);
    double sz = sin(yaw);

    Eigen::Matrix3d R1;
    R1 << cy * cz, sx* sy* cz - cx * sz, cx* sy* cz + sx * sz, cy* sz, sx* sy* sz + cx * cz, cx* sy* sz - sx * cz, -sy, sx* cy, cx* cy;

    return R1;
}

Eigen::Matrix3d KinematicOperator(double roll, double pitch)
{
    double cx = cos(roll);
    double sx = sin(roll);
    double cy = cos(pitch);
    double ty = tan(pitch);

    Eigen::Matrix3d R2;
    R2 << 1, sx * ty, cx * ty,
          0, cx, -sx, 
          0, sx / cy, cx / cy;

    return R2;
}


double x = 0;
double y = 0;
double psi_usv = 0;
double u = 0;
double v = 0;
double r = 0;
double dAD = 0.000000568;

void ins_callback(const geometry_msgs::Pose2D::ConstPtr& ins)
{
  x = ins->x;
  y = ins->y;
  psi_usv = ins->theta;
}

void vel_callback(const geometry_msgs::Vector3::ConstPtr& vel)
{
  u = vel->x;
  v = vel->y; 
  r = vel->z;
}

void ad_callback(const std_msgs::Float64::ConstPtr& ad)
{
  dAD = ad->data;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "ibvs_asmc");
    ros::NodeHandle n;

    ros::Publisher uav_pose_pub = n.advertise<geometry_msgs::Pose>("/uav_model/pose", 1000);
	ros::Publisher uav_vel_pub = n.advertise<geometry_msgs::Twist>("/uav_model/vel", 1000);
	ros::Publisher uav_odom_pub = n.advertise<nav_msgs::Odometry>("/uav_model/odom", 1000);
    ros::Publisher uav_feat_pub = n.advertise<geometry_msgs::Pose2D>("/uav_control/features", 1000);
    ros::Publisher uav_gains_pub = n.advertise<geometry_msgs::Quaternion>("/uav_control/gains", 1000);
    ros::Publisher uav_control_pub = n.advertise<geometry_msgs::Quaternion>("/uav_control/input", 1000);
    ros::Publisher uav_sigma_pub = n.advertise<geometry_msgs::Quaternion>("/uav_control/sigma", 1000);

    ros::Subscriber ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, ins_callback);
    ros::Subscriber local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, vel_callback);
    ros::Subscriber des_ad_sub = n.subscribe("/ad", 1000, ad_callback);

    int rate = 1000;
    ros::Rate loop_rate(rate);

    geometry_msgs::Pose pose;
    geometry_msgs::Twist vel;
    nav_msgs::Odometry odom;
    tf2::Quaternion myQuaternion;
    geometry_msgs::Pose2D feat;
    geometry_msgs::Quaternion gains;
    geometry_msgs::Quaternion input;
    geometry_msgs::Quaternion uav_sigma;

    //UAV inicial conditions 
    Eigen::Vector3d UAV;
    UAV << 2, 1, -4;
    
    Eigen::Vector3d UAV_dot_last;
    UAV_dot_last << 0, 0, 0;
    
    Eigen::Vector4d UAV_Vel;
    UAV_Vel << 0, 0, 0, 0;  //vx,vy,vz,psi_p
    
    Eigen::Vector3d UAV_Vel_lin;
    UAV_Vel_lin << 0, 0, 0;
    
    Eigen::Vector3d UAV_Vel_body;
    UAV_Vel_body << 0, 0, 0;
    
    //UAV Parameters
    double UAV_mass = 2;

    Eigen::Matrix3d Inertia;
    Inertia << 0.0081, 0, 0,
        0, 0.0081, 0,
        0, 0, 0.0142;

    //UAV Dynamics
    Eigen::Vector3d omega;
    omega << 0, 0, 0;

    Eigen::Vector3d Tau; //Motor Torques Vector
    Tau << 0, 0, 0;

    Eigen::Matrix3d Omega_Cross;
    Omega_Cross << 0, -omega(2), omega(1),
                   omega(2), 0, -omega(0),
                   -omega(1), omega(0), 0;

    Eigen::Vector3d omega_dot;
    omega_dot << 0, 0, 0;

    Eigen::Vector3d omega_dot_last;
    omega_dot_last << 0, 0, 0;

    Eigen::Vector3d Attitude_dot;
    Attitude_dot << 0, 0, 0;

    Eigen::Vector3d Attitude_dot_last;
    Attitude_dot_last << 0, 0, 0;

    Eigen::Vector3d Attitude;
    Attitude << 0, 0, 0;

    Eigen::Vector3d f;
    
    double phi = 0; //Roll
    double theta = 0; //Pitch
    double psi = 0; //Yaw
    

    double phid_des = 0;
    double thetad_des = 0;
    double psi_des = 0;

    double phi_dot = 0;
    double theta_dot = 0;
    double psi_dot = 0;
    double psid_des = 0;
    double psi_dot_last = 0;

    double roll_dot = 0;
    double pitch_dot = 0;
     
    //Target
    double radius = 0.25;
    Eigen::MatrixXd POI(3, 4);
    POI << radius, -radius, -radius, radius,
        radius, radius, -radius, -radius,
        0, 0, 0, 0;
    
    Eigen::Vector4d Tgt_Vel;
    Tgt_Vel << 0, 0, 0, 0;
    
    Eigen::MatrixXd POI_UAV_diff(3, 4);
    Eigen::MatrixXd POI_vir(3, 4);

    //Image Features 
    Eigen::MatrixXd pix_vir(2, 4);
    Eigen::Vector2d Centre;
    Eigen::Vector4d FeatVect;
    Eigen::Vector4d FeatVect_dot;
    Eigen::Vector4d DesFeat;
    DesFeat << 0, 0, 1, 0;

    //Error Signal
    Eigen::Vector4d Error;
    Eigen::Vector4d Error_d;

    Eigen::Vector4d kappa;
    kappa << 0, 0, 0, 0;

    //IBVS Gains
    double r1 = 0.04;//0.15;//0.1;
    double r2 = 0.002;//0.3;//0.2;
    double r3 = 0.3;//0.3;
    double gamma = 0.1;
    double eta_1 = 0.003;
    double ai = 1;

    Eigen::Matrix4d I;
    I << -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 0;

    Eigen::Matrix4d eta_2;
    eta_2 << 0.002, 0, 0, 0,
            0, 0.002, 0, 0,
            0, 0, 0.002, 0,
            0, 0, 0, 0.002;

    Eigen::Matrix4d Gamma;
    Gamma << 0.001, 0, 0, 0,
             0, 0.001, 0, 0,
             0, 0, 0.001, 0,
             0, 0, 0, 0.001;

    Eigen::Matrix4d Lambda_estimate;
    Eigen::Matrix4d Lambda_estimate_dot;
    Eigen::Vector4d PI;

    //Rotation Matrices
    Eigen::Matrix3d RP_Mat;

    //ASMC 
    Eigen::Vector4d sigma;
    Eigen::Vector4d K1;
    Eigen::Vector4d K1_d;
    Eigen::Vector4d K2;
    Eigen::Vector4d k;
    Eigen::Vector4d kmin;
    Eigen::Vector4d mu;
    Eigen::Vector4d lambda;
    Eigen::Vector4d K1_d_last;
    Eigen::Vector4d asmc;

    K2 << 0.6, 0.6, 0.6, 1;
    k << 0.1, 0.1, 0.1, 0.1;//0.2, 0.2, 0.2, 0.2;
    kmin << 0.01, 0.01, 0.01, 0.01;//0.1, 0.1, 0.1, 0.1;
    mu << 0.2, 0.2, 0.2, 0.2;
    lambda << 4.0, 8.0, 4.0, 4.0;//2.5, 2.5, 2.5, 20; //4, 4, 2, 20

    K1 << 0, 0, 0, 0;
    K1_d << 0, 0, 0, 0;
    sigma << 0, 0, 0, 0;

    K1_d_last << 0, 0, 0, 0;
    asmc << 0, 0, 0, 0;

    //Image Filter
    Eigen::Vector4d PHI;
    Eigen::Vector4d PHI_d;
    Eigen::Vector4d PHI_d_last;
    Eigen::Vector4d RHO;

    PHI << 0, 0, 0, 0;
    PHI_d << 0, 0, 0, 0;
    PHI_d_last << 0, 0, 0, 0;
    RHO << 0, 0, 0, 0;

    //Control Law
    Eigen::Vector4d xi;
    Eigen::Vector4d xi_d;
    Eigen::Vector4d xi_d_last;
    Eigen::Vector3d xi_lin;

    xi << 0, 0, 0, 0;
    xi_d << 0, 0, 0, 0;
    xi_d_last << 0, 0, 0, 0;

    //PD Controller Gains
    double Kp1 = 0.15;//0.2;
    double Kp2 = 0.15;//0.2;
    double Kp3 = 4.0;//2;

    double Kd1 = 0.2;
    double Kd2 = 0.2;
    double Kd3 = 3;

    //MORE PARAMETERS
    double focal = 0.0032; //Camera's focal length in meters.
    double aD = dAD; //0.0000003; //Desired value of altitude feature.
    double Upsi_est = (1 / 4.5);
    double Upsi_est_d = 0;
    double step = 0.001; //Integration step size
    double q4_dot = 0;
    double q4_last = 0;
    double Upsi_est_d_last = 0;
    
    //USV Auxiliary variables
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;
    Eigen::Vector3d p4;
    p1 << radius, radius, 0;
    p2 << -radius, radius, 0;
    p3 << -radius, -radius, 0;
    p4 << radius, -radius, 0;
    Eigen::Vector3d p1_n;
    Eigen::Vector3d p2_n;
    Eigen::Vector3d p3_n;
    Eigen::Vector3d p4_n;
    p1_n << radius + x, radius + y, 0;
    p2_n << -radius + x, radius + y, 0;
    p3_n << -radius + x, -radius + y, 0;
    p4_n << radius + x, -radius + y, 0;
    Eigen::Vector3d USV;
    USV << x, y, 0;
    Eigen::Vector3d USV_Vel_body;
    USV_Vel_body << u, v, r;
    Eigen::Vector3d USV_Vel_lin;
    USV_Vel_lin = R_psi_T(psi_usv).transpose()*USV_Vel_body;

    pose.position.x = UAV(0);
    pose.position.y = UAV(1);
    pose.position.z = UAV(2);
    odom.pose.pose.position.x = UAV(0);
    odom.pose.pose.position.y = UAV(1);
    odom.pose.pose.position.z = UAV(2);

    myQuaternion.setRPY(phi,theta,psi);

    pose.orientation.x = phi;
    pose.orientation.y = theta;
    pose.orientation.z = psi;
    odom.pose.pose.orientation.x = myQuaternion[0];
    odom.pose.pose.orientation.y = myQuaternion[1];
    odom.pose.pose.orientation.z = myQuaternion[2];
    odom.pose.pose.orientation.w = myQuaternion[3];

    vel.linear.x = UAV_Vel_body(0);
    vel.linear.y = UAV_Vel_body(1);
    vel.linear.z = UAV_Vel_body(2);
    odom.twist.twist.linear.x = UAV_Vel_body(0);
    odom.twist.twist.linear.y = UAV_Vel_body(1);
    odom.twist.twist.linear.z = UAV_Vel_body(2);

    vel.angular.x = omega(0);
    vel.angular.y = omega(1);
    vel.angular.z = omega(2);
    odom.twist.twist.angular.x = omega(0);
    odom.twist.twist.angular.y = omega(1);
    odom.twist.twist.angular.z = omega(2);

    //Data publishing
    uav_pose_pub.publish(pose);
    uav_vel_pub.publish(vel);
    uav_odom_pub.publish(odom);
    
    ros::Duration(1.0).sleep();

    //Data publishing
    uav_pose_pub.publish(pose);
    uav_vel_pub.publish(vel);
    uav_odom_pub.publish(odom);

    ros::Duration(1.0).sleep();

        //LOOP -.-.-.-.-.-.-.-.-.-.-.-.-.-.-LOOP.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-LOOP.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-

        while (ros::ok())
        {
            //******************************************************************************************
            //-------------------------------------Update Target----------------------------------------
            //******************************************************************************************
            aD = dAD;
            USV << x, y, 0;
            USV_Vel_body << u, v, r;
            p1_n = R_psi_T(psi_usv).transpose()*p1 + USV;
            p2_n = R_psi_T(psi_usv).transpose()*p2 + USV;
            p3_n = R_psi_T(psi_usv).transpose()*p3 + USV;
            p4_n = R_psi_T(psi_usv).transpose()*p4 + USV;

            POI << p1_n(0), p2_n(0), p3_n(0), p4_n(0),
                p1_n(1), p2_n(1), p3_n(1), p4_n(1),
                0, 0, 0, 0;
            USV_Vel_lin = R_psi_T(psi_usv).transpose()*USV_Vel_body;
            Tgt_Vel << USV_Vel_lin(0), USV_Vel_lin(1), 0, USV_Vel_lin(2);
            //******************************************************************************************
            //-----------------------------Image Features Extraction------------------------------------
            //******************************************************************************************

            //Inertial2Virtual Conversion
            POI_UAV_diff << POI.col(0) - UAV, POI.col(1) - UAV, POI.col(2) - UAV, POI.col(3) - UAV;
            POI_vir = R_psi_T(psi) * POI_UAV_diff;

            //Real2VirtualCamera Reprojection
            pix_vir << (focal / -UAV(2)) * POI_vir.row(0), (focal / -UAV(2))* POI_vir.row(1);

            //Features Extraction, Features Vector and Feature's Dynamics
            Centre << Centroid(pix_vir);
            FeatVect << ImFeat(Centre, pix_vir, focal, aD);
            FeatVect_dot << ImFeat_dot(FeatVect, UAV_Vel, Tgt_Vel, Upsi_est);

            double q4 = q4 + (step / 2) * (q4_last + FeatVect_dot(3)); //Integral de q4_dot
            q4_last = FeatVect_dot(3); //integral de q4_dot en n-1

            FeatVect(3) = q4;

            //******************************************************************************************
            //-----------------------------Error Signal-------------------------------------------------
            //******************************************************************************************
            Error = FeatVect - DesFeat;
            Error_d = error_dot(Upsi_est, FeatVect, xi, kappa);

            //******************************************************************************************
            //-----------------------------ASMC + Image Filter------------------------------------------
            //******************************************************************************************

            //Sliding Surface
            sigma(0) = lambda(0) * Error(0) + Error_d(0);
            sigma(1) = lambda(1) * Error(1) + Error_d(1);
            sigma(2) = lambda(2) * Error(2) + Error_d(2);
            sigma(3) = lambda(3) * Error(3) + Error_d(3);

            //Adaptive Gains
            if (K1(0) > kmin(0))
            {
                K1_d(0) = k(0) * sign(abs(sigma(0)) - mu(0));
            }

            else
            {
                K1_d(0) = kmin(0);
            }

            if (K1(1) > kmin(1))
            {
                K1_d(1) = k(1) * sign(abs(sigma(1)) - mu(1));
            }

            else
            {
                K1_d(1) = kmin(1);
            }

            if (K1(2) > kmin(2))
            {
                K1_d(2) = k(2) * sign(abs(sigma(2)) - mu(2));
            }

            else
            {
                K1_d(2) = kmin(2);
            }

            if (K1(3) > kmin(3))
            {
                K1_d(3) = k(3) * sign(abs(sigma(3)) - mu(3));
            }

            else
            {
                K1_d(3) = kmin(3);
            }

            //Integrating K1_d

            for (int i = 0; i < K1_d.size(); i++)
            {
                K1(i) = K1(i) + (step / 2) * (K1_d_last(i) + K1_d(i)); //Integral de K1_d
            }

            for (int j = 0; j < K1_d.size(); j++)
            {
                K1_d_last(j) = K1_d(j);
            }

            //ASMC Calculus
            asmc(0) = K1(0) * sqrt(abs(sigma(0))) * sign(sigma(0)) + K2(0) * sigma(0) + PHI(0);
            asmc(1) = K1(1) * sqrt(abs(sigma(1))) * sign(sigma(1)) + K2(1) * sigma(1) + PHI(1);
            asmc(2) = K1(2) * sqrt(abs(sigma(2))) * sign(sigma(2)) + K2(2) * sigma(2) + PHI(2);
            asmc(3) = K1(3) * sqrt(abs(sigma(3))) * sign(sigma(3)) + K2(3) * sigma(3) + PHI(3);

            //Image Filter (PHI)
            RHO << atan(PHI(0)), atan(PHI(1)), atan(PHI(2)), atan(PHI(3));
            PHI_d = -r2 * asmc - r3 * RHO;

            for (int i = 0; i < PHI_d.size(); i++)
            {
                PHI(i) = PHI(i) + (step / 2) * (PHI_d_last(i) + PHI_d(i)); //Integral de PHI_d
            }

            for (int j = 0; j < PHI_d.size(); j++)
            {
                PHI_d_last(j) = PHI_d(j);
            }

            //******************************************************************************************
            //-----------------------------------Control Law--------------------------------------------
            //---------------------Variable xi is the desired velocity of the UAV-----------------------
            //******************************************************************************************
            Lambda_estimate << Lambda_est(Upsi_est, FeatVect);
            PI << tanh(ai * PHI(0)), tanh(ai * PHI(1)), tanh(ai * PHI(2)), tanh(ai * PHI(3));
            Lambda_estimate_dot << Lambda_est(Upsi_est_d, FeatVect_dot);
            xi_d = Xi_dot(Lambda_estimate, Lambda_estimate_dot, eta_1, eta_2, r1, r2, r3, asmc, Gamma, PI, RHO, xi);

            for (int i = 0; i < xi_d.size(); i++)
            {
                xi(i) = xi(i) + (step / 2) * (xi_d_last(i) + xi_d(i)); //Integral de xi_d
            }

            for (int j = 0; j < xi_d.size(); j++)
            {
                xi_d_last(j) = xi_d(j);
            }

            xi_lin << xi(0), xi(1), xi(2);
            psid_des = xi(3);

            psi_des = psi_des + (step / 2) * (psi_dot_last + psid_des); //Integral de xi_d
            psi_dot_last = psid_des;



            //******************************************************************************************
            //-----------------------------Update Law--------------------------------------------------
            //******************************************************************************************
            Upsi_est_d = Upsilon_estimate_dot(gamma, xi, xi_d, I, eta_1, eta_2, Gamma, asmc);

            Upsi_est = Upsi_est + (step / 2) * (Upsi_est_d_last + Upsi_est_d); //Integral de Upsilon_est
            Upsi_est_d_last = Upsi_est_d;

            //******************************************************************************************
            //-----------------------------Thrust, Desired Roll and Pitch-------------------------------
            //******************************************************************************************

            f << Force(xi_d, xi, UAV_Vel(3), UAV_mass);
            double thrust = Thrust(theta, phi, UAV_mass, f);
            double phi_des = Desired_roll(f, thrust);

            bool phi_nan = std::isnan(phi_des);
            if (phi_nan == true)
            {
                phi_des = 1.5 * sign(f(1)/thrust);
            }
           
            if (phi_des > 1.5 )
            {
                phi_des = 1.5;
            }
            if (phi_des < -1.5)
            {
                phi_des = -1.5;
            }
            
            double theta_des = Desired_pitch(f, thrust, phi_des);

            bool theta_nan = std::isnan(theta_des);
            if (theta_nan == true)
            {
                theta_des = 1.5 * sign(f(0)/ (thrust * cos(phi_des)));
            }

            if (theta_des > 1.5)
            {
                theta_des = 1.5;
            }
            if (theta_des < -1.5)
            {
                theta_des = -1.5;
            }


            
            //******************************************************************************************
            //-------------------------------------UAV Dynamics-----------------------------------------
            //******************************************************************************************
            //------------------------------------Linear Dynamics---------------------------------------
            //******************************************************************************************    
            RP_Mat << Rtp(theta, phi).inverse();
            UAV_Vel_body = RP_Mat * xi_lin;     //Virtual2Body frame conversion
            UAV_Vel_lin = RotationMatrix(phi, theta, psi) * UAV_Vel_body;       //Body2Inertial frame conversion

            for (int i = 0; i < UAV.size(); i++)
            {
                UAV(i) = UAV(i) + (step / 2) * (UAV_dot_last(i) + UAV_Vel_lin(i));
            }

            for (int j = 0; j < UAV.size(); j++)
            {
                UAV_dot_last(j) = UAV_Vel_lin(j);
            }

            //******************************************************************************************
            //-------------------------------------Attitude---------------------------------------------
            //******************************************************************************************  
            //-------------------------------------PD Controller---------------------------------------- 
            //******************************************************************************************

            double Roll_error = phi_des - phi;
            double Pitch_error = theta_des - theta;
            double Yaw_error = psi_des - psi;

            double Roll_error_dot = phid_des - phi_dot;
            double Pitch_error_dot = thetad_des - theta_dot;
            double Yaw_error_dot = psid_des - psi_dot;

            Tau(0) = Kp1 * Roll_error + Kd1 * Roll_error_dot;
            Tau(1) = Kp2 * Pitch_error + Kd2 * Pitch_error_dot;
            Tau(2) = Kp3 * Yaw_error + Kd3 * Yaw_error_dot;

            //******************************************************************************************  
            //-------------------------------------Angular Dynamics------------------------------------- 
            //******************************************************************************************    

            omega_dot = Inertia.inverse() * (Tau - Omega_Cross * Inertia * omega);

            for (int i = 0; i < omega.size(); i++)
            {
                omega(i) = omega(i) + (step / 2) * (omega_dot_last(i) + omega_dot(i));
            }

            for (int j = 0; j < omega.size(); j++)
            {
                omega_dot_last(j) = omega_dot(j);
            }

            Attitude_dot = KinematicOperator(phi, theta) * omega;

            for (int i = 0; i < Attitude_dot.size(); i++)
            {
                Attitude(i) = Attitude(i) + (step / 2) * (Attitude_dot_last(i) + Attitude_dot(i));
            }

            for (int j = 0; j < Attitude.size(); j++)
            {
                Attitude_dot_last(j) = Attitude_dot(j);
            }

            phi = Attitude(0);
            theta = Attitude(1);
            psi = Attitude(2);

            phi_dot = Attitude_dot(0);
            theta_dot = Attitude_dot(1);
            psi_dot = Attitude_dot(2);

            UAV_Vel << UAV_Vel_lin(0), UAV_Vel_lin(1), UAV_Vel_lin(2), psi_dot;

            pose.position.x = UAV(0);
            pose.position.y = UAV(1);
            pose.position.z = UAV(2);
            odom.pose.pose.position.x = UAV(0);
            odom.pose.pose.position.y = UAV(1);
            odom.pose.pose.position.z = UAV(2);

            myQuaternion.setRPY(phi,theta,psi);

            pose.orientation.x = phi;
            pose.orientation.y = theta;
            pose.orientation.z = psi;
            odom.pose.pose.orientation.x = myQuaternion[0];
            odom.pose.pose.orientation.y = myQuaternion[1];
            odom.pose.pose.orientation.z = myQuaternion[2];
            odom.pose.pose.orientation.w = myQuaternion[3];

            vel.linear.x = UAV_Vel_body(0);
            vel.linear.y = UAV_Vel_body(1);
            vel.linear.z = UAV_Vel_body(2);
            odom.twist.twist.linear.x = UAV_Vel_body(0);
            odom.twist.twist.linear.y = UAV_Vel_body(1);
            odom.twist.twist.linear.z = UAV_Vel_body(2);

            vel.angular.x = omega(0);
            vel.angular.y = omega(1);
            vel.angular.z = omega(2);
            odom.twist.twist.angular.x = omega(0);
            odom.twist.twist.angular.y = omega(1);
            odom.twist.twist.angular.z = omega(2);

            feat.x = Centre(0);
            feat.y = Centre(1);

            gains.x = K1(0);
            gains.y = K1(1);
            gains.z = K1(2);
            gains.w = K1(3);

            input.x = asmc(0);
            input.y = asmc(1);
            input.z = asmc(2);
            input.w = asmc(3);

            uav_sigma.x = sigma(0);
            uav_sigma.y = sigma(1);
            uav_sigma.z = sigma(2);
            uav_sigma.w = sigma(3);

            //Data publishing
            uav_pose_pub.publish(pose);
            uav_vel_pub.publish(vel);
            uav_odom_pub.publish(odom);
            uav_feat_pub.publish(feat);
            uav_gains_pub.publish(gains);
            uav_control_pub.publish(input);
            uav_sigma_pub.publish(uav_sigma);

            ros::spinOnce();
	        loop_rate.sleep();
        }
        return 0;
}
