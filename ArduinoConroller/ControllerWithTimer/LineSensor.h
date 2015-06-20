#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#include "QTRSensors.h"

class LineSensor {
public:
  static const int NUM_SENSORS = 8;
  typedef unsigned int TSensorArray[NUM_SENSORS] ;

private:
    static const int TIMEOUT_USEC = 2500;
    static const int EMITTER_PIN = 39;

    
    TSensorArray _sensorValues;
    unsigned int _position;
    unsigned long _readCount;
    static QTRSensorsRC qtrrc;
    // TODO Capture average time beween reads.

public:
	LineSensor() : _position(0), _readCount(0) {
/*
		for (int i = 0; i < NUM_SENSORS; i++) {
			_sensorValues[i] = 0;
			qtrrc.calibratedMinimumOn[i] = 100;
			qtrrc.calibratedMaximumOn[i] = 2500;

		}
*/
	}

 	void Calibrate() {
		for (int i = 0; i < 1; i++) {
   			qtrrc.calibrate();
	   	}
 
 		for (int i = 0; i < NUM_SENSORS; i++) {
			_sensorValues[i] = 0;
			qtrrc.calibratedMinimumOn[i] = 100;
			qtrrc.calibratedMaximumOn[i] = 2500;

		}
	}

	void Read() {
		_position = qtrrc.readLine(_sensorValues);
		_readCount++;
	}

	unsigned int Position() {
		return _position;
	}

	const TSensorArray& SensorValues() {
		return _sensorValues;
	}
};

QTRSensorsRC LineSensor::qtrrc((unsigned char[]) {23, 25, 27, 29, 31, 33, 35, 37},
		                   NUM_SENSORS,
		                   TIMEOUT_USEC,
		                   EMITTER_PIN);

#endif
