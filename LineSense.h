#ifndef __LINE_SENSE_H
#define __LINE_SENSE_H

#include <pthread.h>

class LineSense {
public:
    static const int kNumSensors = 8;

    typedef unsigned int TSensorArray[kNumSensors] ;

	LineSense();

 	void Calibrate();

    bool IsOnLine();

    /* Read sensors, return position */
	unsigned int Read();

	unsigned int Position();

	const TSensorArray& SensorValues();

private:
    static const int kEmitterPin = 12;
    static const int kLineThreshold = 200; // Sensor values over this are considered an indication of a line.
    static const int kNoiseThreshold = 50; // Sensor values below this are considered noise.
    static const int kTimeoutUsec = 2500;
    static const int kSamplesToAverage = 4;

    static const int kPins[kNumSensors];
    
    unsigned int calibrated_maximum_[kNumSensors];
    unsigned int calibrated_minimum_[kNumSensors];
    bool is_on_line_;
    unsigned int last_position_;
    unsigned int next_raw_sensor_index;
    unsigned int position_;
    TSensorArray raw_sensor_values_[kSamplesToAverage];
    unsigned long read_count_;
    pthread_t sensor_thread_;
    int sensor_thread_id_;
    TSensorArray sensor_values_;
    // TODO Capture average time beween reads.

    /* Compute microsecond difference between two times. */
    unsigned int DurationMicroseconds(struct timeval& newer, struct timeval& older);

    /* Turn infrared emitters all off. */
    void EmittersOff();

    /* Turn infrared emitters all on. */
    void EmittersOn();

    /* Internal read of sensors. */
    void ReadPrivate();

    static void *ReadRawSensors(void *anonPtr);
};

#endif