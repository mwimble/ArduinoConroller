#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <QueueList.h>
#include <ThreadController.h>
#include <Thread.h>

#include "ArleneMotor.h"

ros::NodeHandle  nh;
ArleneMotor arleneMotor(nh);

void messageCb( const geometry_msgs::Twist& twistMsg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
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

