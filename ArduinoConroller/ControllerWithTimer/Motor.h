#ifndef __MOTOR_H
#define __MOTOR_H

#include <QueueList.h>

extern Log logger;

class Motor {
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
    int speed_a;
    int speed_b;
  } Command;
  
  private:
  
  static const int I1 = 8;
  static const int I2 = 11;
  static const int I3 = 12;
  static const int I4 = 13;
  static const int SPEED_A = 9;
  static const int SPEED_B = 10;
  
  static int speed_a_;
  static int speed_b_;
  
  static QueueList<Command> commands_;
  
  public:
  static bool motor_busy_;

  Motor() {
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);
    pinMode(SPEED_A, OUTPUT);
    pinMode(SPEED_B, OUTPUT);
  }
  
  static int SpeedA() { return speed_a_; }
  static int SpeedB() { return speed_a_; }
  
  static void Enqueue(Command& command) {
    cli();
    commands_.push(command);
    sei();
    logger.print("Command pushed, len: ");logger.println(commands_.count());
  }
  
  static void Process() {
    logger.print("Q len: " );logger.println(commands_.count());
    if (!commands_.isEmpty() && !motor_busy_) {
      cli();
      Command command = commands_.pop();
      motor_busy_ = true;
      sei();

      switch (command.direction) {
        case FORWARD:
          Forward(command.speed_a, command.speed_b);
          break;
          
        case BACKWARD:
          Backward(command.speed_a, command.speed_b);
          break;
          
        case LEFT_TURN:
          Left(command.speed_a, command.speed_b);
          break;
          
        case RIGHT_TURN:
          Right(command.speed_a, command.speed_b);
          break;
          
        case STOP:
          Stop();
          break;
          
      }
      
      startTimer4(1000000L); // 1.0 sec.
    }
  }
  
  static void Stop() {
    pauseTimer4();
    speed_a_ = 0;
    speed_b_ = 0;
    analogWrite(SPEED_A, 127);
    analogWrite(SPEED_B, 127);
    digitalWrite(I4, HIGH); // Motor B stop.
    digitalWrite(I3, HIGH);
    digitalWrite(I2, HIGH);  // Motor A stop.
    digitalWrite(I1, HIGH);
    motor_busy_ = false;
  }

  static void Forward(int speed_a, int speed_b) {
    //logger.println(">> FORWARD");
    speed_a_ = speed_a;
    speed_b_ = speed_b;
    analogWrite(SPEED_A, speed_a_);
    analogWrite(SPEED_B, speed_b_);
    digitalWrite(I4, HIGH); // Motor B clockwise.
    digitalWrite(I3, LOW);
    digitalWrite(I2, LOW);  // Motor A clockwise.
    digitalWrite(I1, HIGH);
  }
  
  static void Backward(int speed_a, int speed_b) {
    //logger.println(">> BACKWARD");
    speed_a_ = speed_a;
    speed_b_ = speed_b;
    analogWrite(SPEED_A, speed_a_);
    analogWrite(SPEED_B, speed_b_);
    digitalWrite(I4, LOW); // Motor B anticlockwise.
    digitalWrite(I3, HIGH);
    digitalWrite(I2, HIGH);  // Motor A anticlockwise.
    digitalWrite(I1, LOW);
  }
  
  static void Left(int speed_a, int speed_b) {
    //logger.println(">> LEFT");
    speed_a_ = speed_a;
    speed_b_ = speed_b;
    analogWrite(SPEED_A, speed_a_);
    analogWrite(SPEED_B, speed_b_);
    digitalWrite(I4, HIGH); // Motor B clockwise.
    digitalWrite(I3, LOW);
    digitalWrite(I2, HIGH);  // Motor A anticlockwise.
    digitalWrite(I1, LOW);
  }
  
  static void Right(int speed_a, int speed_b) {
    //logger.println(">> RIGHT");
    speed_a_ = speed_a;
    speed_b_ = speed_b;
    analogWrite(SPEED_A, speed_a_);
    analogWrite(SPEED_B, speed_b_);
    digitalWrite(I4, LOW); // Motor B anticlockwise.
    digitalWrite(I3, HIGH);
    digitalWrite(I2, LOW);  // Motor A clockwise.
    digitalWrite(I1, HIGH);
  }
  
};

bool Motor::motor_busy_ = false;
int Motor::speed_a_ = 64;
int Motor::speed_b_ = 64;  
QueueList<Motor::Command> Motor::commands_;

ISR(timer4Event) {
  pauseTimer4();
  resetTimer4();
  Motor::Stop();
  logger.println("timer4Event");
}

#endif

