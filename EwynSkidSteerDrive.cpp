#include "EwynSkidSteerDrive.h"

 enum {
	RIGHT_FRONT = 0,
    LEFT_FRONT = 1,
    RIGHT_REAR = 2,
    LEFT_REAR = 3,
  };

EwynSkidSteerDrive::EwynSkidSteerDrive() {
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM("A ROS Node for EwynSkidSteerDrive has not been unitialized.");
		throw new Exception("A ROS Node for EwynSkidSteerDrive has not been unitialized.");
	}
	leftFrontJointName_ = "leftFrontWheelJoint";
	leftReadJointName_ = "leftRearWheelJoint";
	rightFrontJointName_ = "rightFrontWheelJoint";
	rightRearJointName_ = "rightRearWheelJoint";

	wheelSeparation_ = 8 * 0.0254; //#####
	wheelDiameter_ = 5.9 * 0.0254; //#####
	for (int i = 0; i < 4; i++) {
		wheelSpeed_[i] = 0.0;
	}

	commandTopic "cmd_vel";
	odometryTopic_ = "odom";
	odometryFrame_ = "odom";
	robotBaseFrame_ = "robot_footprint";

	updateRate_ = 100.0;
	updatePeriod_ = 1.0 / updateRate_;
}

EwynSkidSteerDrive::~EwynSkidSteerDrive() {
	delete rosNode_;
	delete transformBroadcaster_;
}