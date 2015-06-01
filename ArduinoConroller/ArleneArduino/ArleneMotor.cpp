#include "Arduino.h"

#include "ArleneMotor.h"

class MotorController : public Thread {
  private:
  const QueueList<Command>& _queue;
  
  public:
  MotorController(const QueueList<Command>& queue) : _queue(queue) {
  }
  
  void run() {
  }
  
  bool shouldRun() {
    return !queue.isEmpty;
  }
};

ArleneMotor::ArleneMotor(const ros::NodeHandle&  nh) : ThreadController(), _nh(nh) {
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  pinMode(SPEED_A, OUTPUT);
  pinMode(SPEED_B, OUTPUT);
  
  motorThread = new MotorController();
  add(motorThread);
}
