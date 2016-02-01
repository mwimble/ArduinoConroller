#include "ros/ros.h"
#include "line_sensor/LineSensor.h"

#include "LineSense.h"

#include <sstream>

int main(int argc, char ** argv) {
	ros::init(argc, argv, "LineSensor");
	ros::NodeHandle node;
	ros::Publisher lineSensorPublisher = node.advertise<line_sensor::LineSensor>("lineSensor", 10);
	ros::Rate loopRate(2);

	LineSense lineSense = LineSense();

	int count = 0;
	while (ros::ok()) {
		ROS_INFO("lineSensor count: %d", count);
		line_sensor::LineSensor msg;
		unsigned int position = lineSense.Read();
		const LineSense::TSensorArray& sv = lineSense.SensorValues();
		ROS_INFO("position: %d, isOnLine: %d, L[%d, %d, %d, %d, %d, %d, %d, %d]R",
				 position,
				 lineSense.IsOnLine(),
				 sv[0],
				 sv[1],
				 sv[2],
				 sv[3],
				 sv[4],
				 sv[5],
				 sv[6],
				 sv[7],
				 sv[8]);
		msg.count = position;
		lineSensorPublisher.publish(msg);
		
		
		ros::spinOnce();
		loopRate.sleep();
		count++;
	}

	return 0;
}