/*
 * Sensors.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: michaelwimble
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "Sensors.h"

namespace PositionSensors {

Sensors::Sensors() {
	initializeAccelerometer();
	initializeCompass();
//	initializeGyro();
}

Sensors::~Sensors() {
	if (accelerometerFileHandle && ((close(accelerometerFileHandle)) != 0)) {
		fprintf(stderr, "accelerometerFileHandle close error: %s\n", strerror(errno));
	}

	if (compassFileHandle && ((close(compassFileHandle)) != 0)) {
		fprintf(stderr, "compassFileHandle close error: %s\n", strerror(errno));
	}

	if (gyroFileHandle && ((close(gyroFileHandle)) != 0)) {
		fprintf(stderr, "gyroFileHandle close error: %s\n", strerror(errno));
	}
}

void Sensors::sleepMs(long ms) {
	struct timespec nanoTime = {0, ms * 1000000};
	struct timespec remaining;
	nanosleep(&nanoTime, &remaining);
}

int Sensors::openI2C(const char* deviceName, const int deviceAddress) {
	int result;
	char filename[16];
	sprintf(filename, "/dev/i2c-2");
	if (Sensors::DEBUG) {
		fprintf(stderr, "Opening %s\n", filename);
	}

	if ((result = open(filename, O_RDWR)) < 0) {
		fprintf(stderr, "[Sensors::openI2C] %s open error: %s\n", deviceName, strerror(errno));
		return -1;
	}

	if (ioctl(result, I2C_SLAVE, deviceAddress) < 0) {
		fprintf(stderr, "[Sensors::openI2C] %s ioctl error: %s\n", deviceName, strerror(errno));
		return -1;
	}

	return result;
}

int Sensors::sendToDevice(
		const int fileHandle,
		const unsigned char* data,
		const int dataLen,
		const int deviceAddress,
		const char* operation) {
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2];

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::sendToDevice] fileHandle: %d\n", fileHandle);
		fprintf(stderr, "...data[0]: %02X\n", data[0]);
		fprintf(stderr, "...dataLen: %d\n", dataLen);
		fprintf(stderr, "...deviceAddress: %2X\n", deviceAddress);
		fprintf(stderr, "...operation: %s\n", operation);
	}

	msgs[0].addr = deviceAddress;
	msgs[0].flags = 0;
	msgs[0].len = dataLen;
	msgs[0].buf = (__u8*) data;
	msgset .nmsgs = 1;
	msgset.msgs = msgs;
	if (Sensors::DEBUG) {
		if (int xxx = ioctl(fileHandle, I2C_RDWR, &msgset) < 0) {
			fprintf(stderr, "[Sensors::sendToDevice] (%d) %s error: %s\n", xxx, operation, strerror(errno));
			return -1;
		}
	}

	Sensors::sleepMs(5);
	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::sendToDevice] success of %s\n", operation);
	}

	return 0;
}

int Sensors::sendToAndReceiveFromDevice(
		const int fileHandle,
		const unsigned char* sendData,
		const int sendDataLen,
		const unsigned char* receiveData,
		const int receiveDataLen,
		const int deviceAddress,
		const char* operation) {
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2];

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::sendToAndReceiveFromDevice] fileHandle: %d\n", fileHandle);
		fprintf(stderr, "...sendData[0]: %02X\n", sendData[0]);
		fprintf(stderr, "...sendDataLen: %d\n", sendDataLen);
		fprintf(stderr, "...deviceAddress: %2X\n", deviceAddress);
		fprintf(stderr, "...operation: %s\n", operation);
	}

	msgs[0].addr = deviceAddress;
	msgs[0].flags = 0;
	msgs[0].len = sendDataLen;
	msgs[0].buf = (__u8*) sendData;

	msgs[1].addr = deviceAddress;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = receiveDataLen;
	msgs[1].buf = (__u8*) receiveData;

	msgset .nmsgs = 2;
	msgset.msgs = msgs;
	if (ioctl(fileHandle, I2C_RDWR, &msgset) < 0) {
		fprintf(stderr, "[Sensors::sendToAndReceiveFromDevice] %s error: %s\n", operation, strerror(errno));
		return -1;
	}

	if (Sensors::DEBUG) {
		printf("...raw data block: ");
		int listOffset = 0;
		for (unsigned int i = 0; i < receiveDataLen; i++) {
			printf("%02X: %02X, ", listOffset++,
					receiveData[i]);
		}

		printf("\n");
	}

	Sensors::sleepMs(5);
	return 0;
}

void Sensors::initializeAccelerometer() {
	if ((accelerometerFileHandle = Sensors::openI2C("Accelerometer", ACCELEROMETER_ADDR)) <= 0) {
		return;
	}

	unsigned char data[2];
	data[0] = 0x2D; // Power register
	data[1] = 0x08; // Measure mode.
	int result;
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer Measurement Mode")) < 0) {
		fprintf(stderr, "#1 result: %d\n", result);
		return;
	}

	data[0] = 0x31; // Data format register
	data[1] = 0x08; // Full resolution.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer Full Resolution")) < 0) {
		fprintf(stderr, "#2 result: %d\n", result);
		return;
	}

	data[0] = 0x38; // FIFO_CTL
	data[1] = 0x84; // Stream watermark level.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer Stream Watermark")) < 0) {
		fprintf(stderr, "#3 result: %d\n", result);
		return;
	}

	data[0] = 0x2C; // Rate
	data[1] = 0x09; // 50Hz.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer 50Hz Rate")) < 0) {
		fprintf(stderr,"#4 result: %d\n", result);
		return;
	}

	fprintf(stderr, "Accelerometer initialized\n"); //###
}

void Sensors::getAccelerometerBlock() {
	unsigned char data[1] = { ACCELEROMETER_BLOCK_START };
	if ((Sensors::sendToAndReceiveFromDevice(
			accelerometerFileHandle,
			data,
			sizeof(data),
			&accelerometerData[ACCELEROMETER_BLOCK_START],
			ACCELEROMETER_BLOCK_END - ACCELEROMETER_BLOCK_START + 1,
			Sensors::ACCELEROMETER_ADDR, "Accelerometer Block Read")) < 0) {
		fprintf(stderr, "[Sensors::getAccelerometerBlock] fail\n");
		return;
	}

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::getAccelerometerBlock] Accelerometer raw data block: ");
		int listOffset = ACCELEROMETER_BLOCK_START;
		for (unsigned int i = 0; i < (ACCELEROMETER_BLOCK_END - ACCELEROMETER_BLOCK_START + 1); i++) {
			fprintf(stderr, "%02X: %02X, ", listOffset++,
					accelerometerData[ACCELEROMETER_BLOCK_START + i]);
		}

		fprintf(stderr, "\n");
	}
}

void Sensors::dumpAccelerometerSensorBlock() {
	printf("THRESH_TAP:     0x%02X (%3.2fmg)\n", accelerometerData[0x1D],
			accelerometerData[0x1D] * 62.5);
	printf("OFSX:           0x%02X (%3.2fmg)\n", accelerometerData[0x1E],
			((signed char) accelerometerData[0x1E]) * 15.6);
	printf("OFSY:           0x%02X (%3.2fmg)\n", accelerometerData[0x1F],
			((signed char) accelerometerData[0x1F]) * 15.6);
	printf("OFSZ:           0x%02X (%3.2fmg)\n", accelerometerData[0x20],
			((signed char) accelerometerData[0x20]) * 15.6);
	printf("DUR:            0x%02X (%3.2fms)\n", accelerometerData[0x21],
			accelerometerData[0x21] * 625.0 / 1000.0);
	printf("Latent:         0x%02X (%3.2fms)\n", accelerometerData[0x22],
			accelerometerData[0x22] * 1.25);
	printf("Window:         0x%02X (%3.2fms)\n", accelerometerData[0x23],
			accelerometerData[0x23] * 1.25);
	printf("THRESH_ACT:     0x%02X (%3.2fmg)\n", accelerometerData[0x24],
			accelerometerData[0x24] * 62.5);
	printf("THRESH_INACT:   0x%02X (%3.2fmg)\n", accelerometerData[0x25],
			accelerometerData[0x25] * 62.5);
	printf("TIME_INACT:     0x%02X (%ds)\n", accelerometerData[0x26],
			accelerometerData[0x26]);
	printf("ACT_INACT_CTL:  0x%02X (ACT:%s, ActX:%s, ActY:%s, ActZ:%s, INACT:%s, InactX:%s, InactY:%s, InactZZ:%s)\n",
			accelerometerData[0x27],
			(accelerometerData[0x27] & 0x80 ? "AC" : "DC"),
			(accelerometerData[0x27] & 0x40 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x20 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x10 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x08 ? "AC" : "DC"),
			(accelerometerData[0x27] & 0x04 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x02 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x01 ? "ENA" : "dis"));
	printf("THRESH_FF:      0x%02X (%3.2fmg)\n", accelerometerData[0x28],
			accelerometerData[0x28] * 62.5);
	printf("TIME_FF:        0x%02X (%dms)\n", accelerometerData[0x29],
			(accelerometerData[0x29] >> 4) * 5);
	printf("TAP_AXES:       0x%02X (Supp:%s, TAPXen:%s, TAPYen:%s, TAPZen:%s)\n",
			accelerometerData[0x2A],
			(accelerometerData[0x2A] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x2A] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x2A] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x2A] & 0x01 ? "Y" : "n"));
	printf("ACC_TAP_STATUS: 0x%02X (Sources ACTX:%s, ACTY:%s, ACTZ:%s, Asleep:%s, TAPX: %s, TAPY:%s, TAPZ:%s)\n",
			accelerometerData[0x2B],
			(accelerometerData[0x2B] & 0x40 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x01 ? "Y" : "n"));
	printf("BW_RATE:        0x%02X (LOWP:%s, RATE:%d)\n", accelerometerData[0x2C],
			(accelerometerData[0x2C] & 0x10 ? "Y" : "n"),
			accelerometerData[0x2C] & 0x0F);
	printf("POWER_CTL:      0x%02X (LINK:%s, AUTO_SLEEP:%s, Measure:%s, Sleep:%s, Wakeup:%d)\n",
			accelerometerData[0x2D],
			(accelerometerData[0x2D] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x2D] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x2D] & 0x08 ? "meas" : "stby"),
			(accelerometerData[0x2D] & 0x04 ? "SLEEP" : "norm"),
			accelerometerData[0x2D] & 0x03);
	printf("INT_ENABLE:     0x%02X (DataReady:%s, SingleTap:%s, DoubleTap:%s, Activity:%s, Inactivity:%s, FreeFall:%s, WaterMark:%s, Overrun:%s)\n",
			accelerometerData[0x2E],
			(accelerometerData[0x2E] & 0x80 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x40 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x01 ? "Y" : "n"));
	printf("INT_MAP:        0x%02X (DataReady:%s, SingleTap:%s, DoubleTap:%s, Activity:%s, Inactivity:%s, FreeFall:%s, WaterMark:%s, Overrun:%s)\n",
			accelerometerData[0x2F],
			(accelerometerData[0x2F] & 0x80 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x40 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x20 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x10 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x08 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x04 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x02 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x01 ? "INT2" : "INT1"));
	printf("INT_SOURCE:     0x%02X (DataReady:%s, SingleTap:%s, DoubleTap:%s, Activity:%s, Inactivity:%s, FreeFall:%s, WaterMark:%s, Overrun:%s)\n",
			accelerometerData[0x30],
			(accelerometerData[0x30] & 0x80 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x40 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x01 ? "Y" : "n"));
	const char* gRange[] = { "2g", "4g", "8g", "16g" };
	printf("DATA_FORMAT:    0x%02X (SelfTest:%s, SPI:%s, IntInvert:%s, FullRes:%s, Justify:%s, Range%s\n",
			accelerometerData[0x31],
			(accelerometerData[0x31] & 0x80 ? "Y" : "n"),
			(accelerometerData[0x31] & 0x40 ? "3wire" : "4wire"),
			(accelerometerData[0x31] & 0x20 ? "actHigh" : "actLow"),
			(accelerometerData[0x31] & 0x08 ? "4mg/bit" : "10-bit"),
			(accelerometerData[0x31] & 0x04 ? "left" : "right"),
			gRange[accelerometerData[0x31] & 0x03]);
	short value;
	value = accelerometerData[0x33] << 8 | accelerometerData[0x32];
	printf("X:             %d 0x%04X (%.4fg)\n", value, (unsigned short) value, gForce(X));
	value = accelerometerData[0x35] << 8 | accelerometerData[0x34];
	printf("Y:             %d 0x%04X (%.4fg)\n", value, (unsigned short) value, gForce(Y));
	value = accelerometerData[0x37] << 8 | accelerometerData[0x36];
	printf("Z:             %d 0x%04X (%.4fg)\n", value, (unsigned short) value, gForce(Z));
	const char* fifoMode[] = { "bypass", "FIFO", "stream", "trigger" };
	printf("FIFO_MODE:     0x%02X (Mode:%s, trigger:%s, samples:%d)\n",
			accelerometerData[0x38], fifoMode[accelerometerData[0x38] >> 6],
			(accelerometerData[0x38] & 0x20 ? "int2" : "int1"),
			accelerometerData[0x38] & 0x1F);
	printf("FIFO_STATUS:   0x%02X (triggered:%s, entries:%d)\n",
			accelerometerData[0x39],
			(accelerometerData[0x39] & 0x80 ? "Y" : "n"),
			accelerometerData[0x39] & 0x3F);
}

void Sensors::initializeCompass() {
	if ((compassFileHandle = Sensors::openI2C("Magnetometer", COMPASS_ADDR)) <= 0) {
		return;
	}

	unsigned char data[] = {
		0x00,	// Register address to start writing.
		0x70,	// 011 => 8 samples averaged, 100 => 15 Hz output rate, 00 => Normal config.
		0x20,   // 001 => +/- 1.3 Ga (default), 0.92mG/LSb, output range F800->007F (-2048->2047).
		0x00    // Continue-measurement mode.
	};

	int xxx;
	if ((xxx = Sensors::sendToDevice(compassFileHandle, data,  sizeof(data), Sensors::COMPASS_ADDR, "Compass Setup")) < 0) {
		fprintf(stderr, "[Sensors::initializeCompass] (%d) fail\n", xxx);
		return;
	}

	printf("Compass initialized\n");
}

void Sensors::getCompassBlock() {
	unsigned char data[2];

	data[0] = 0;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			compassData,
			3,
			Sensors::COMPASS_ADDR, "Compass Block Read pt1")) < 0) {
		return;
	}

	data[0] = 3;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			&compassData[3],
			6,
			Sensors::COMPASS_ADDR, "Compass Block Read pt2")) < 0) {
		return;
	}

	data[0] = 9;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			&compassData[9],
			4,
			Sensors::COMPASS_ADDR, "Compass Block Read pt3")) < 0) {
		return;
	}

	printf("Compass raw data block: ");
	int listOffset = COMPASS_BLOCK_START;
	for (unsigned int i = 0; i < (COMPASS_BLOCK_END - COMPASS_BLOCK_START + 1); i++) {
		printf("%02X: %02X, ", listOffset++,
				compassData[COMPASS_BLOCK_START + i]);
	}

	printf("\n");
}

void Sensors::dumpCompassSensorBlock() {
	const int samplesAveraged[] = {1, 2, 4, 8};
	const char* outputRage[] = {
		"0.75Hz", "1.5Hz", "3Hz", "7.5Hz",
		"15Hz", "30Hz", "75Hz", "???"
	};
	const char* measurementMode[] = {
		"Normal", "+bias", "-bias", "???"
	};
	const char* gain[] = {
		"0.88", "1.3", "1.9", "2.5"
		"4.0", "4.7", "5.6", "8.1"
	};
	const char* mode[] = {
		"continuous", "single", "idle(10)", "idle(11)"
	};

	printf("CRA:            0x%02X (avgd samp:%d, rate:%s, config:%s\n",
		   compassData[0x00],
		   samplesAveraged[(compassData[0x00] & 0x60) >> 5],
		   outputRage[(compassData[0x00] & 0x1C) >> 2],
		   measurementMode[compassData[0x00] & 0x3]);
	printf("CRB:            0x%02X (+/-%s Ga)\n",
		   compassData[0x01],
		   gain[compassData[0x01] >> 5]);
	printf("MODE:           0x%02X (%s)\n",
		   compassData[0x02],
		   mode[compassData[0x02]]);
	short value = (compassData[0x03] << 8) | compassData[0x04];
	compassAxisData.x_raw = value;
	printf("X:              0x%02X 0x%02X (%d)\n", compassData[0x03], compassData[0x04], value);
	value = (compassData[0x05] << 8) | compassData[0x06];
	compassAxisData.y_raw = value;
	printf("Y:              0x%02X 0x%02X (%d)\n", compassData[0x05], compassData[0x06], value);
	value = (compassData[0x07] << 8) | compassData[0x08];
	compassAxisData.z_raw = value;
	printf("Z:              0x%02X 0x%02X (%d)\n", compassData[0x07], compassData[0x08], value);
	printf("STATUS:         0x%02X (lock:%s, rdy:%s)\n",
		   compassData[0x09],
		   compassData[0x09] & 0x02 ? "Y" : "n",
		   compassData[0x09] & 0x01 ? "Y" : "n");
	printf("ID:             0x%02X%02X%02X\n",
		   compassData[0x0A],
		   compassData[0x0B],
		   compassData[0x0C]);
}

float Sensors::getCompassHeading() {
	float heading = atan2(compassAxisData.y_raw, compassAxisData.x_raw) + declinationRadians;
	printf("compassAxisData.y_raw: %f, x_raw: %f, atan: %f\n", compassAxisData.y_raw, compassAxisData.x_raw, heading);
	if (heading < 0) {
		heading += (2 * M_PI);
	}

	if (heading > (2 * M_PI)) {
		heading -= (2 * M_PI);
	}

	return heading * (180.0 / M_PI);
}

float Sensors::gForce(Axis axis) {
	short value = 0.0;
	if (axis == X) {
		value = (accelerometerData[0x33] << 8 | accelerometerData[0x32]);
	}

	if (axis == Y) {
		value = (accelerometerData[0x35] << 8 | accelerometerData[0x34]);
	}

	if (axis == Z) {
		value = (accelerometerData[0x37] << 8 | accelerometerData[0x36]);
	}

	return value / 256.0;
}

void Sensors::initializeGyro() {
	if ((gyroFileHandle = Sensors::openI2C("Gyro", GYRO_ADDR)) <= 0) {
		return;
	}

	/*
	char filename[16];
	sprintf(filename, "/dev/i2c-1");
	fprintf(stderr, "Opening %s\n", filename);
	if ((accelerometerFileHandle = open(filename, O_RDWR)) < 0) {
		fprintf(stderr, "i2c_open accelerometer open error: %s\n", strerror(errno));
		return;
	}

	if (ioctl(accelerometerFileHandle, I2C_SLAVE, ACCELEROMETER_ADDR) < 0) {
		fprintf(stderr, "i2c_open accelerometer ioctl error: %s\n", strerror(errno));
		return;
	}

	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2];
	unsigned char offset[2] = { ACCELEROMETER_BLOCK_START, 0 };

	offset[0] = 0; // Start register address.
	offset[1] = 0;
	msgs[0].addr = ACCELEROMETER_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 1; // Length of data in offset.
	msgs[0].buf = (__u8 *) offset;

	msgs[1].addr = ACCELEROMETER_ADDR; // Now read the device ID.
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(deviceId);
	msgs[1].buf = (__u8 *) deviceId;

	msgset.nmsgs = 2;
	msgset.msgs = msgs;

	if (ioctl(accelerometerFileHandle, I2C_RDWR, &msgset) < 0) {
		fprintf(stderr, "i2c_read_no_ack accelerometer read device ID error: %s\n", strerror(errno));
		return;
	}

	printf("Accelerometer Device ID: ");

	int listOffset = 0;
	for (unsigned int i = 0; i < sizeof(deviceId); i++) {
		printf("%02X: %02X, ", listOffset++, deviceId[i]);
	}

	printf("\n");

	offset[0] = 0x2D; // POWER_CTL
	offset[1] = 0x08; // Measure mode.
	msgs[0].addr = ACCELEROMETER_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (__u8 *) offset;

	msgset.nmsgs = 1;
	msgset.msgs = msgs;

	if (ioctl(accelerometerFileHandle, I2C_RDWR, &msgset) < 0) {
		fprintf(stderr, "i2c_read_no_ack accelerometer write register 2D error: %s\n", strerror(errno));
		return;
	}

	offset[0] = 0x38; // FIFO_CTL
	offset[1] = 0x84; // Stream watermark level.
	msgs[0].addr = ACCELEROMETER_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (__u8 *) offset;

	msgset.nmsgs = 1;
	msgset.msgs = msgs;

	if (ioctl(accelerometerFileHandle, I2C_RDWR, &msgset) < 0) {
		fprintf(stderr, "i2c_read_no_ack accelerometer FIFO config error: %s\n", strerror(errno));
		return;
	}

	fprintf(stderr, "Accelerometer initialized\n"); //###
	*/
}

} /* namespace PositionSensors */

