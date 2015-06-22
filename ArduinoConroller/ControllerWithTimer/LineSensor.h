#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#include "QTRSensors.h"

class LineSensor {
  public:
  static const int NUM_SENSORS = 8;
  typedef unsigned int TSensorArray[NUM_SENSORS] ;

  private:
  static const int TIMEOUT_USEC = 800;
  static const int EMITTER_PIN = 39;
  
  static TSensorArray _sensorValues;
  static unsigned int _position;
  static unsigned long _readCount;
  static QTRSensorsRC qtrrc;
  // TODO Capture average time beween reads.

  public:
  LineSensor() {
  }

  static void Calibrate() {
    qtrrc.calibrate();
    for (int i = 0; i < NUM_SENSORS; i++) {
      _sensorValues[i] = 0;
      qtrrc.calibratedMinimumOn[i] = 40;
      qtrrc.calibratedMaximumOn[i] = TIMEOUT_USEC;
    }
  }

  static void Read() {
    _position = qtrrc.readLine(_sensorValues);
    _readCount++;
  }

  static unsigned int Position() {
    return _position;
  }

  static const TSensorArray& SensorValues() {
    return _sensorValues;
  }
};

LineSensor::TSensorArray  LineSensor::_sensorValues;
unsigned int  LineSensor::_position;
unsigned long  LineSensor::_readCount;
QTRSensorsRC LineSensor::qtrrc((unsigned char[]) {23, 25, 27, 29, 31, 33, 35, 37},
		                   NUM_SENSORS,
		                   TIMEOUT_USEC,
		                   EMITTER_PIN);

#endif
