#ifndef __MOTOR_H
#define __MOTOR_H

#include <ros.h>
#include <Arduino.h>
#include <QueueList.h>
#include <Timer5.h>
#include "RosLogger.h"

class Motor;

extern RosLogger* rlog;
extern Motor* motor;

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
    } Command;
  
private:
    static const char* DIRECTION_STR[5];

    static const int I1 = 8;
    static const int I2 = 11;
    static const int I3 = 12;
    static const int I4 = 13;
    static const int SPEED_A = 9;
    static const int SPEED_B = 10;
    int _speed;  // Speed of motor.

    const ros::NodeHandle&  _nh;

    static QueueList<Command> _commands;

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
        digitalWrite(I3, HIGH);
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
  
public:
    static bool motorBusy;

    Motor(const ros::NodeHandle&  nh) : _nh(nh) {
        _speed = 127;
        pinMode(I1, OUTPUT);
        pinMode(I2, OUTPUT);
        pinMode(I3, OUTPUT);
        pinMode(I4, OUTPUT);
        pinMode(SPEED_A, OUTPUT);
        pinMode(SPEED_B, OUTPUT);
    }
  
    void enqueue(Command& command) {
        cli();
        _commands.push(command);
        sei();
    }

    void run() {
        if (!_commands.isEmpty() && !motorBusy) {
            cli();
            Command command = _commands.pop();
            motorBusy = true;
            sei();
            rlog->info("Motor run command: %s", DIRECTION_STR[command.direction]);

            switch (command.direction) {
            case FORWARD:
                forward();
                break;

            case BACKWARD:
                backward();
                break;

            case LEFT_TURN:
                left();
                break;

            case RIGHT_TURN:
                right();
                break;

            case STOP:
                stop();
                break;
            }

            startTimer5(1000000/*250000L*/); // 0.25 sec. #####
        }
    }  
  
    void stop() {
        digitalWrite(SPEED_A, LOW);
        digitalWrite(SPEED_B, LOW);
    }

    int queueLength() {
        return _commands.count();
    }

};

QueueList<Motor::Command> Motor::_commands;
bool Motor::motorBusy = false;
const char* Motor::DIRECTION_STR[] = {
    "FORWARD",
    "BACKWARD",
    "LEFT_TURN",
    "RIGHT_TURN",
    "STOP"
};

ISR(timer5Event) {
  resetTimer5();
  Motor::motorBusy = false;
  motor->stop();
}

#endif
