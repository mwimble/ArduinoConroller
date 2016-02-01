#include <DueTimer.h>

#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <UTFT.h>
#include <Thread.h>
#include <ThreadController.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>  

#include "Display.h"
#include "KaimiBase.h"

void geometryMessageCb( const geometry_msgs::Twist& twistMsg);

ros::NodeHandle  nh;  // ROS node handle.
ros::Subscriber<geometry_msgs::Twist> commandSubscriber("/cmd_vel", &geometryMessageCb);
KaimiBase* robotBase;
Display* robotDisplay;

// ThreadController that will controll all threads
ThreadController masterThreadController = ThreadController();


bool ledOn;

void geometryMessageCb(const geometry_msgs::Twist& twistMsg) {
  char status[128];
//  Serial.println("---- ---- ---- ----");
//  Serial.print("KaimiController new command, qlen: ");
//  Serial.print(KaimiBase::Singleton()->queueLength());
//  Serial.print(", x: ");
//  Serial.print(twistMsg.linear.x);
//  Serial.print(", y: ");
//  Serial.print(twistMsg.linear.y);
//  Serial.print(", z: ");
//  Serial.println(twistMsg.angular.z);
// 
  robotBase->enqueue(twistMsg);
  robotDisplay->enqueue(twistMsg.linear.x, twistMsg.angular.z);

  ledOn = !ledOn;   // blink the led
  digitalWrite(13, ledOn ? HIGH : LOW);  
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit 

  nh.subscribe(commandSubscriber);
  robotBase = KaimiBase::Singleton();
  robotDisplay = Display::Singleton();
  nh.loginfo("KaimiBase ready");
}

void loop() {
  nh.spinOnce();
  masterThreadController.run();
  //delay(1);
}
