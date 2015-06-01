#include "Arduino.h"

#include "ArleneMotor.h"

class MotorController : public Thread {
  private:
  QueueList<ArleneMotor::Command>& _commands;
  
  public:
  MotorController(QueueList<ArleneMotor::Command>& commands) 
    : _commands(commands) {
  }
  
  void run() {
    if (!_commands.isEmpty()) {
      cli();
      ArleneMotor::Command command = _commands.pop();
      sei();
      digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    }
    
    runned();
  }
  
  bool shouldRun() {
    return !_commands.isEmpty();
  }
};

ArleneMotor::ArleneMotor(const ros::NodeHandle&  nh) : ThreadController(), _nh(nh) {
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  pinMode(SPEED_A, OUTPUT);
  pinMode(SPEED_B, OUTPUT);
  
  motorThread = new MotorController(_commands);
  motorThread->setInterval(10);
  add(motorThread);
  run();
}
