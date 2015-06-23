#include <QueueList.h>
#include <QTRSensors.h>
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

const bool DEBUG = false;
const unsigned long DEBUG_TIME_USEC = 60000000;

extern unsigned long myMicros();
unsigned long microStart = myMicros();

LineSensor lineSensor = LineSensor();
Motor motor = Motor();
QuadratureEncoder quadratureEncoder = QuadratureEncoder();
SensorStick sensorStick = SensorStick();
Strategy strategy = Strategy();
	
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

  LineSensor::Calibrate();
  microStart = myMicros();
  strategy.Initialize();
}

unsigned long loopCount = 0;
unsigned long sumLineSensorRead = 0;
unsigned long sumSensorStickRead = 0;

unsigned int linePosition;

bool statsReported = false;
void loop() {
  int oldOdo = QuadratureEncoder::Counter();
  while ((Strategy::State() != Strategy::STOP) || (oldOdo != QuadratureEncoder::Counter())) {
    unsigned long s = myMicros();
    LineSensor::Read();
    sumLineSensorRead += myMicros() - s;
    linePosition = LineSensor::Position();
  
    s = myMicros();
    SensorStick::Process();
    sumSensorStickRead += myMicros() - s;
    
    oldOdo = QuadratureEncoder::Counter();
    //Strategy::Dump("-loop");
    if (!DEBUG) {
      Strategy::Process();
    } else {
      if ((myMicros() - microStart) > DEBUG_TIME_USEC) Strategy::Stop();
      Strategy::Dump("DEBUG");
    }
    //delay(20);
    if (Strategy::State() == Strategy::STOP) {
      if (!statsReported) {
        unsigned long microEnd = myMicros();
        unsigned long duration = microEnd - microStart;
        Serial.println("==== ==== STATS");
        
        Serial.print("loopCount: ");
        Serial.print(loopCount);
        Serial.print(", avg loop duration: ");
        Serial.println((duration * 1.0) / (loopCount * 1.0));
        
        Serial.print("usec at start: ");
        Serial.print(microStart);
        Serial.print(", at end: ");
        Serial.print(microEnd);
        Serial.print(", duration: ");
        Serial.println(duration);
        
        Serial.print("lineSensor sumLineSensorRead: ");
        Serial.print(sumLineSensorRead);
        Serial.print(", avg read duration: ");
        Serial.println((sumLineSensorRead * 1.0) / (loopCount * 1.0));
        
        Serial.print("sensorStick sumSensorStickRead: ");
        Serial.print(sumSensorStickRead);
        Serial.print(",  avg read dration: ");
        Serial.println((sumSensorStickRead * 1.0) / (loopCount * 1.0));
        
        Strategy::Dump("Final dump");
        statsReported = true;
      }
      
      delay(20);
    }

      loopCount++;
  }
}
