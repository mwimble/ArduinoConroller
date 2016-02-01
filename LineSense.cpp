#include <ros/ros.h>
#include <string>
#include <time.h>
#include <unistd.h>
#include <wiringPi.h>

#include "LineSense.h"

LineSense::LineSense() {
	sensor_thread_id_ = pthread_create(&sensor_thread_, NULL, Foo, this);
	if (sensor_thread_id_) {
		throw std::string("Unable to create LineSense sensor thread");
	}
	wiringPiSetupGpio();
	Calibrate();
}

void LineSense::Calibrate() {
	for (int i = 0; i < kNumSensors; i++) { //##### Until we get a real calibrate
		sensor_values_[i] = 0;
		calibrated_minimum_[i] = 100;
		calibrated_maximum_[i] = 2500;
	}
}

unsigned int LineSense::DurationMicroseconds(struct timeval& newer, struct timeval& older) {
	long t_newer = (newer.tv_sec * 1000000) + newer.tv_usec;
	long t_older = (older.tv_sec * 1000000) + older.tv_usec;
	return (unsigned int)(t_newer - t_older);
}

void LineSense::EmittersOff() {
    pinMode(kEmitterPin, OUTPUT);
    digitalWrite(kEmitterPin, LOW);
}

void LineSense::EmittersOn() {
    pinMode(kEmitterPin, OUTPUT);
    digitalWrite(kEmitterPin, HIGH);
	usleep(200); // Typically actually 400us
}

bool LineSense::IsOnLine() {
	return is_on_line_;
}

unsigned int LineSense::Position() {
	return position_;
}

unsigned int LineSense::Read() {
	is_on_line_ = false;

	EmittersOn();
	ReadPrivate();
	EmittersOff();

	// Now compute the "line position".
	unsigned int average = 0;
	unsigned int sum = 0;

	for (int i = 0; i < kNumSensors; i++) {
		if (sensor_values_[i] > kLineThreshold) {
			is_on_line_ = true;
		}

		if (sensor_values_[i] > kNoiseThreshold) { // Skip noise values.
			average += (long)(sensor_values_[i]) * (i * 1000);
			sum += (long)(sensor_values_[i]);
		}
	}

	if (!is_on_line_) {
		if (last_position_ < ((kNumSensors - 1) * 1000 / 2)) {
			position_ = 0;
		} else {
			position_ = (kNumSensors - 1) * 1000;
		}
	} else {
		position_ = sum == 0 ? 0 : (average / sum);
	}

	last_position_ = position_;
	read_count_++;
	return position_;
}

void LineSense::ReadPrivate() {
	// the assumed sensor value to the maximum microsecond
	// timeout value.
	for (int i = 0; i < kNumSensors; i++) {
		sensor_values_[i] = kTimeoutUsec; // Assume no reflectance.
		pinMode(kPins[i], OUTPUT);
		digitalWrite(kPins[i], HIGH);
	}
	// Give the transistors time to saturate.
	usleep(10);

	// Now flip the pins to input mode.
	for (int i = 0; i < kNumSensors; i++) {
		pinMode(kPins[i], INPUT);
		digitalWrite(kPins[i], LOW);
	}

	int continue_count = kNumSensors;
	int next_sensor = 0;
	struct timeval start_time;
	struct timeval now;
	gettimeofday(&start_time, NULL);
	
	// Now time each sensor as it flips from on to off. The microsecond time is the
	// sensor value. Once all sensors have flipped, or the maximum time out period
	// has passed, stop.
	do {
		gettimeofday(&now, NULL);
		if (continue_count <= 0) break; // If all sensors have reported (flipped), abort early.
		if ((sensor_values_[next_sensor] == kTimeoutUsec) && // Sensor did not previously go low.
			(digitalRead(kPins[next_sensor]) == LOW)) { // Sensor has newly gone low.

			sensor_values_[next_sensor] = DurationMicroseconds(now, start_time); // Value is usec time it took to go low.

			// Now normalize the value;
			int denominator = calibrated_maximum_[next_sensor] - calibrated_minimum_[next_sensor];
			signed int scaled_value =
				denominator == 0 ? 0 :
					(((signed long) sensor_values_[next_sensor]) - calibrated_minimum_[next_sensor]) * 1000 / denominator;

			if (scaled_value < 0) scaled_value = 0;
			else if (scaled_value > 1000) scaled_value = 1000;

			sensor_values_[next_sensor] = scaled_value;

			continue_count--; // Note that one less sensor is outstanding.
		}

		if (++next_sensor >= kNumSensors) next_sensor = 0; // Round robin.
	} while (DurationMicroseconds(now, start_time) < kTimeoutUsec); // Stop at max time.
}

void *LineSense::ReadRawSensors(void *anonPtr) {
	LineSense* instance = (LineSense*) anonPtr;
	ROS_INFO("Foo");
}

const LineSense::TSensorArray& LineSense::SensorValues() {
	return sensor_values_;
}

const int LineSense::kPins[kNumSensors] = {5, 6, 13, 19, 26, 21, 20, 16};
