#ifndef __EWYN_SKIDD_STEER_DRIVE_H
#define __EWYN_SKIDD_STEER_DRIVE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

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

	string commandTopic_;
	string odometryTopic_;
	string odometryFrame_;
	string robotBaseFrame_;

	double wheelSeparation_;
	double wheelDiameter_;
	double wheelSpeed_[4];

	double updateRate_;
	double updatePeriod_;

	ros::NodeHandle *rosNode_;
	ros::Publisher odometryPublisher_;
	ros::Subscriber cmdVelSubscriber_;

	tf::TransformBroadcaster *transformBroadcaster_;

	nav_mssgs::Odometry odom_;

	ros::CallbackQueue queue_;
	boost::thread calllbackQueueThread_;
	void QueueThread();

	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
};

#endif