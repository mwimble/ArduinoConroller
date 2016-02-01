#ifndef __EWYN_SKIDD_STEER_DRIVE_H
#define __EWYN_SKIDD_STEER_DRIVE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

class EwynSkidSteerDrive {
public:
	EwynSkidSteerDrive();
	~EwynSkidSteerDrive();

private:
	void publishOdometry();
	void getWheelVelocities();

	string leftFrontJointName_;
	string rightFrontJointName_;
	string leftRearJointName_;
	string rightRearJointName_;

	double wheelSeparation_;
	double wheelDiameter_;
	double wheelSpeed_[4];

	ros::NodeHandle *rosNode_;
	ros::Publisher odometryPublisher_;
	ros::Subscriber cmdVelSubscriber_;

	tf::TransformBroadcaster *transformBroadcaster_;

	nav_mssgs::Odometry odom_;
};

#endif