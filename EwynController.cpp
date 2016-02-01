#include <ros/ros.h>

using namespace std;

class EwynControllerClass {
private:
	const string NAMESPACE = "ewyn_robot_control";

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;

	string robot_model_; // Robot model.

	// Publishers.
	ros::Publisher ref_vel_left_wheels;
	ros::Publisher ref_vel_right_wheels;

public:
	EwynControllerClass(ros::NodeHandle nodeHandle) :
		node_handle_(nodeHandle), private_node_handle_("~") {
		ROS_INFO("EwynControllerClass constructor");

		ros::NodeHandle ewyn_robot_control_node_handle(node_handle_, NAMESPACE);

		if (!private_node_handle_.getParam("model", robot_model_)) {
			ROS_ERROR("'model' not defined in namespace: '%s'", NAMESPACE);
			exit(-1);
		} else {
			ROS_INFO("[%s] PARAM model: %s", NAMESPACE, robot_model_);
		}
	}
};