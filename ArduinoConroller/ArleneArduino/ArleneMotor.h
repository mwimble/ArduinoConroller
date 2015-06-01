#ifndef __ARLENE_MOTOR_H
#define __ARLENE_MOTOR_H

#include <ros.h>
#include <QueueList.h>
#include <ThreadController.h>
#include <Thread.h>

class ArleneMotor : ThreadController {
  private:
  
  enum Direction {
    FORWARD,
    BACKWARD,
    LEFT_TURN,
    RIGHT_TURN,
    STOP
  };
  
  struct Command {
    Direction direction;
  };
  
  static const int I1 = 8;
  static const int I2 = 11;
  static const int I3 = 12;
  static const int I4 = 13;
  static const int SPEED_A = 9;
  static const int SPEED_B = 10;
  int speed = 127;  // Speed of motor.
  const ros::NodeHandle&  _nh;
  
  Thread* motorThread;
  QueueList<Command> commands;
  
  public:
  
  ArleneMotor(const ros::NodeHandle&  nh);
  
  void forward();
};

#endif
