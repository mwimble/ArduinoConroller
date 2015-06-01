#ifndef __ARLENE_MOTOR_H
#define __ARLENE_MOTOR_H

#include <ros.h>
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
  int speed = 127;  // Speed of motor.
  const ros::NodeHandle&  _nh;
  
  Thread* motorThread;
  QueueList<Command> _commands;
  
  public:
  ArleneMotor(const ros::NodeHandle&  nh);
  
  void enqueue(Command& command) {
    cli();
    _commands.push(command);
    sei();
  }

};

#endif
