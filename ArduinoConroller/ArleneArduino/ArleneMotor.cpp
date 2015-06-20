#include <ros.h>
#include <Timer5.h>

#include "Arduino.h"

#include "ArleneMotor.h"

bool motorBusy = false;
ArleneMotor* motorStopper;

class MotorController : public Thread {
  private:
  ArleneMotor* _motor;
  QueueList<ArleneMotor::Command>& _commands;
  const ros::NodeHandle&  _nh;
  
  public:
  MotorController(const ros::NodeHandle&  nh, ArleneMotor* motor, QueueList<ArleneMotor::Command>& commands) 
    : _nh(nh), _motor(motor), _commands(commands) {
      motorStopper = motor;
  }
  
  void run() {
    
    if (!_commands.isEmpty()) {
      cli();
      ArleneMotor::Command command = _commands.pop();
      motorBusy = true;
      sei();
      char* cmdMsg = "run command 0";
      cmdMsg[strlen(cmdMsg) -1] += command.direction;
      ArleneMotor::msg.data = cmdMsg;
      ArleneMotor::motorLog.publish(&ArleneMotor::msg);
      //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
      switch (command.direction) {
        case ArleneMotor::FORWARD:
          _motor->forward();
          break;
          
        case ArleneMotor::BACKWARD:
          _motor->backward();
          break;
          
        case ArleneMotor::LEFT_TURN:
          _motor->left();
          break;
          
        case ArleneMotor::RIGHT_TURN:
          _motor->right();
          break;
      }
      
      startTimer5(250000L); // 0.25 sec.
    }
    
    runned();
  }
  
  bool shouldRun() {
    return !_commands.isEmpty() && !motorBusy;
  }
};

std_msgs::String ArleneMotor::msg;
ros::Publisher ArleneMotor::motorLog("motorLog", &msg);


ISR(timer5Event) {
  resetTimer5();
  motorBusy = false;
  motorStopper->stop();
  ArleneMotor::msg.data = "TEND";
  ArleneMotor::motorLog.publish(&ArleneMotor::msg);
}


ArleneMotor::ArleneMotor(ros::NodeHandle&  nh) : ThreadController(), _nh(nh) {
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  pinMode(SPEED_A, OUTPUT);
  pinMode(SPEED_B, OUTPUT);
  
  _nh.advertise(motorLog);

  motorThread = new MotorController(nh, this, _commands);
  motorThread->setInterval(10);
  add(motorThread);
  run();
}
