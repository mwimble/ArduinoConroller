#include <QueueList.h>
#include <QTRSensors.h>
#include <Timer4.h>
#include <Timer5.h>
#include <Wire.h>

#include "HMC5883L.h"
#include "I2Cdev.h"
#include "LineSensor.h"
#include "Log.h"
#include "Map.h"
#include "Motor.h"
#include "QuadratureEncoder.h"
#include "SensorStick.h"
#include "Strategy.h"

const bool DEBUG = false;
const unsigned long DEBUG_TIME_USEC = 60000000;

extern unsigned long myMicros();
unsigned long microStart = myMicros();

Log logger = Log();
LineSensor lineSensor = LineSensor();
Motor motor = Motor();
QuadratureEncoder quadratureEncoder = QuadratureEncoder();
SensorStick sensorStick = SensorStick();
Strategy strategy = Strategy();
	
void setup() {
  logger.waitSerial();  
  
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
  while ((Strategy::State() != Strategy::SOLVED) || (oldOdo != QuadratureEncoder::Counter())) {
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
    if (Strategy::State() == Strategy::SOLVED) {
      if (!statsReported) {
        unsigned long microEnd = myMicros();
        unsigned long duration = microEnd - microStart;
        logger.println("==== ==== STATS");
        
        logger.print("loopCount: ");
        logger.print(loopCount);
        logger.print(", avg loop duration: ");
        logger.println((duration * 1.0) / (loopCount * 1.0));
        
        logger.print("usec at start: ");
        logger.print(microStart);
        logger.print(", at end: ");
        logger.print(microEnd);
        logger.print(", duration: ");
        logger.println(duration);
        
        logger.print("lineSensor sumLineSensorRead: ");
        logger.print(sumLineSensorRead);
        logger.print(", avg read duration: ");
        logger.println((sumLineSensorRead * 1.0) / (loopCount * 1.0));
        
        logger.print("sensorStick sumSensorStickRead: ");
        logger.print(sumSensorStickRead);
        logger.print(",  avg read dration: ");
        logger.println((sumSensorStickRead * 1.0) / (loopCount * 1.0));
        
        Strategy::Dump("Final dump");
        Map::DumpTree(Strategy::StartMap());
        statsReported = true;
        return;
      }
      
      delay(20);
    }

      loopCount++;
  }
}
