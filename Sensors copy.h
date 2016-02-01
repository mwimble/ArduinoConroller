/*
 * Sensorts.h
 *
 *  Created on: Jan 31, 2015
 *      Author: michaelwimble
 */

#ifndef SENSORS_H_
#define SENSORS_H_

namespace PositionSensors {

class Sensors {
private:
	static const bool DEBUG = false;
	static const int ACCELEROMETER_ADDR = 0x53;
	static const int ACCELEROMETER_BLOCK_START = 0x1D;
	static const int ACCELEROMETER_BLOCK_END = 0x39;
	int	accelerometerFileHandle;
	unsigned char accelerometerData[ACCELEROMETER_BLOCK_END + 1];
	unsigned char deviceId[1];

	static const int COMPASS_ADDR = 0x1E;
	static const int COMPASS_BLOCK_START = 0;
	static const int COMPASS_BLOCK_END = 12;
	int	compassFileHandle;
	unsigned char compassData[COMPASS_BLOCK_END + 1];
	static const float compassScale = 0.92;
	struct CompassRawData {
		float x_raw;
		float y_raw;
		float z_raw;
	};

	CompassRawData compassAxisData;

	static const int GYRO_ADDR = 0x68;
	int	gyroFileHandle;

	/*
	Santa Clara California
	Latitude: 37 21' 14.8" N
	Longitude: 121 57' 18.9" W
	Magnetic declination: +13 37'
	Declination is POSITIVE (EAST)
	Inclination: 60  58'
	Magnetic field strength: 48359.9 nT
	 */
	static const float declinationRadians = 1.0640690672992;

	void initializeAccelerometer();
	void initializeCompass();
	void initializeGyro();
	static void sleepMs(long ms);
	static int openI2C(const char* deviceName, const int deviceAddress);
	static int sendToDevice(
			const int fileHandle,
			const unsigned char* data,
			const int dataLen,
			const int deviceAddress,
			const char* operation);
	static int sendToAndReceiveFromDevice(
			const int fileHandle,
			const unsigned char* sendData,
			const int sendDataLen,
			const unsigned char* receiveData,
			const int receiveDataLen,
			const int deviceAddress,
			const char* operation);

public:
	enum Axis {
		X,
		Y,
		Z
	};

	Sensors();
	virtual ~Sensors();

	void dumpAccelerometerSensorBlock();

	void getAccelerometerBlock();

	float gForce(Axis axis);

	void dumpCompassSensorBlock();

	void getCompassBlock();

	float getCompassHeading();
};

} /* namespace PositionSensors */

#endif /* SENSORS_H_ */
