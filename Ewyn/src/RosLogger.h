#ifndef __ROS_LOGGER_H
#define __ROS_LOGGER_H

#include <ros.h>
#include <std_msgs/String.h>

#include <Arduino.h>
#include <stdarg.h>

class RosLogger {
	private:
		ros::NodeHandle& _nh;
		std_msgs::String msg;
		ros::Publisher topic;

	public:
		RosLogger(ros::NodeHandle& nh) :
			_nh(nh), topic("motorLog", &msg) {
				_nh.advertise(topic);
		}

		void info(const char* format, ...) {
			va_list	myArgs;
			char buffer[256];
			va_start(myArgs, format);
            vsnprintf(buffer, sizeof(buffer), format, myArgs);
			va_end(myArgs);
			msg.data = buffer;
			topic.publish(&msg);
		}
};

#endif
