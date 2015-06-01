#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <QueueList.h>
#include <ThreadController.h>
#include <Thread.h>

#include "ArleneMotor.h"

ros::NodeHandle  nh;
ArleneMotor arleneMotor(nh);

void messageCb( const geometry_msgs::Twist& twistMsg) {
  ArleneMotor::Command command;
  if (twistMsg.linear.x > 0) {
    command.direction = ArleneMotor::FORWARD;
  } else if (twistMsg.linear.x < 0) {
    command.direction = ArleneMotor::BACKWARD;
  } else if (twistMsg.angular.z > 0) {
    command.direction = ArleneMotor::RIGHT_TURN;
  } else if (twistMsg.angular.z < 0) {
    command.direction = ArleneMotor::LEFT_TURN;
  } else {
    command.direction = ArleneMotor::STOP;
  }
  
  arleneMotor.enqueue(command);
}

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", &messageCb);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

