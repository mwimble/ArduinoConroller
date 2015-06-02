#ifndef __ARLENE_MOTOR_H
#define __ARLENE_MOTOR_H

#include <ros.h>
#include <std_msgs/String.h>
#include <QueueList.h>
#include <ThreadController.h>
#include <Thread.h>

class ArleneMotor : ThreadController {
  public:
  
  enum Direction {
    FORWARD,
    BACKWARD,
    LEFT_TURN,
    RIGHT_TURN,
    STOP
  };
  
  typedef struct {
    Direction direction;
  } Command;
  
  private:
  
  static const int I1 = 8;
  static const int I2 = 11;
  static const int I3 = 12;
  static const int I4 = 13;
  static const int SPEED_A = 9;
  static const int SPEED_B = 10;
  int _speed = 127;  // Speed of motor.
  
  ros::NodeHandle&  _nh;
  
  Thread* motorThread;
  QueueList<Command> _commands;
  
  public:
  static std_msgs::String msg;
  static ros::Publisher motorLog;

  ArleneMotor(ros::NodeHandle&  nh);
  
  void enqueue(Command& command) {
    cli();
    _commands.push(command);
    sei();
  }
  
  void forward() {
    analogWrite(SPEED_A, _speed);
    analogWrite(SPEED_B, _speed);
    digitalWrite(I4, HIGH); // Motor B clockwise.
    digitalWrite(I3, LOW);
    digitalWrite(I2, LOW);  // Motor A clockwise.
    digitalWrite(I1, HIGH);
  }
  
  void backward() {
    analogWrite(SPEED_A, _speed);
    analogWrite(SPEED_B, _speed);
    digitalWrite(I4, LOW); // Motor B anticlockwise.
    digitalWrite(I3, LOW);
    digitalWrite(I2, HIGH);  // Motor A anticlockwise.
    digitalWrite(I1, LOW);
  }
  
  void left() {
    analogWrite(SPEED_A, _speed);
    analogWrite(SPEED_B, _speed);
    digitalWrite(I4, HIGH); // Motor B clockwise.
    digitalWrite(I3, LOW);
    digitalWrite(I2, HIGH);  // Motor A anticlockwise.
    digitalWrite(I1, LOW);
  }
  
  void right() {
    analogWrite(SPEED_A, _speed);
    analogWrite(SPEED_B, _speed);
    digitalWrite(I4, LOW); // Motor B anticlockwise.
    digitalWrite(I3, HIGH);
    digitalWrite(I2, LOW);  // Motor A clockwise.
    digitalWrite(I1, HIGH);
  }
  
  void stop() {
    digitalWrite(SPEED_A, LOW);
    digitalWrite(SPEED_B, LOW);
  }

};

#endif
