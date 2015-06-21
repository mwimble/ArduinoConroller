#include <QueueList.h>
#include <QTRSensors.h>
#include <Thread.h>
#include <ThreadController.h>
#include <Timer4.h>
#include <Timer5.h>
#include <Wire.h>

#include "HMC5883L.h"
#include "I2Cdev.h"
#include "IMU.h"
#include "LineSensor.h"
#include "Motor.h"
#include "QuadratureEncoder.h"
#include "SensorStick.h"
#include "Strategy.h"


// ThreadController that will controll all threads
ThreadController controll = ThreadController();

// IMU
//Thread IMUThread = Thread();
//IMU imu = IMU();

// SensorStick.
Thread sensorStickThread = Thread();
SensorStick sensorStick = SensorStick();
void sensorStickThreadCallback() {
  SensorStick::Process();
}

// Line Sensor.
Thread lineSensorThread = Thread();
LineSensor lineSensor = LineSensor();
unsigned int linePosition;
const LineSensor::TSensorArray& lineSensorValues = lineSensor.SensorValues();

// Thread callback for LineSensor
void lineSensorThreadCallback() {
    lineSensor.Read();
    linePosition = lineSensor.Position();
}

// Motor.
Thread motorThread = Thread();
Motor motor = Motor();
void motorThreadCallback() {
//  Serial.println("motorThreadCallback");
//  Motor::Process();
}

// Quadrature Encoder.
QuadratureEncoder quadratureEncoder = QuadratureEncoder();

// Thread callback for IMU
//void IMUCallback() {
//  imu.processStream();
//}

Strategy strategy = Strategy();
	
// This is the callback for the Timer
ISR(timer5Event) {
  controll.run();
}

void waitSerial() {
  while (!Serial.available());
  delay(10);
  while (Serial.available() && Serial.read());	
}

void setup() {
  Serial.begin(57600);
  Serial.println("Start");
  Serial.println("Enter key to start");
  waitSerial();  

  lineSensor.Calibrate();

  lineSensorThread.onRun(lineSensorThreadCallback);
  lineSensorThread.setInterval(5);
  
  motorThread.onRun(motorThreadCallback);
  motorThread.setInterval(10);

  //IMUThread.onRun(IMUCallback);
  //IMUThread.setInterval(1);
  sensorStickThread.onRun(sensorStickThreadCallback);
  sensorStickThread.setInterval(2);


  controll.add(&lineSensorThread); // & to pass the pointer to it
  //controll.add(&IMUThread);
  controll.add(&sensorStickThread);
  controll.add(&motorThread);

  startTimer5(20); //#####
  
  Motor::Command bw = {Motor::BACKWARD, 127, 127};
  Motor::Command fw = {Motor::FORWARD, 127, 127};
  Motor::Command left = {Motor::LEFT_TURN, 127, 127};
  Motor::Command right = {Motor::RIGHT_TURN, 127, 127};
  Motor::Command mstop = {Motor::STOP, 0, 0};
  /*
  motor.Enqueue(mstop);
  motor.Enqueue(fw);
  motor.Enqueue(left);  
  motor.Enqueue(bw);
  motor.Enqueue(left);
  motor.Enqueue(fw);
  motor.Enqueue(right);
  motor.Enqueue(fw);
  */
  
  strategy.Initialize();
}


void loop() {
  int oldOdo = QuadratureEncoder::Counter();
  while ((Strategy::State() != Strategy::STOP) || (oldOdo != QuadratureEncoder::Counter())) {
    oldOdo = QuadratureEncoder::Counter();
    //Strategy::Dump("-loop");
    Strategy::Process();
    //delay(20);
    if (Strategy::State() == Strategy::STOP) { delay(20); }
  }
}
