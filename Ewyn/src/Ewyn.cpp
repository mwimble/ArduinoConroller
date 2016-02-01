#include "string.h"

//#define USE_USBCON 1
#include <ros.h>
#include <QueueList.h>

//#include "LineSensor.h"
#include "Motor.h"
//#include "QTRSensors.h"
//#include "QuadratureEncoder.h"
#include "RosLogger.h"
#include <Timer5.h>

ros::NodeHandle  	nh;
//LineSensor*			lineSensor;
Motor*				motor;
//QuadratureEncoder*	quadratureEncoder;
RosLogger*			rlog;

int loopCounter = 1;

void setup() {
	nh.initNode();
	Serial.begin(38400);
	delay(1000);

	rlog = new RosLogger(nh);
//	quadratureEncoder = new QuadratureEncoder();
//	lineSensor = new LineSensor();
//	lineSensor->calibrate();
	motor = new Motor(nh);
	Motor::Command c;
	c.direction = Motor::FORWARD; motor->enqueue(c);
	c.direction = Motor::BACKWARD; motor->enqueue(c);
	c.direction = Motor::LEFT_TURN; motor->enqueue(c);
	c.direction = Motor::RIGHT_TURN; motor->enqueue(c);
	for (int i = 0; i < 5; i++) {
		nh.spinOnce();
		delay(1000);
	}
	
	for (int i = 0; i < 4; i++) {
		rlog->info("START UP %d", i);
		nh.spinOnce();
		delay(200);
	}
}

char buffer[128];

void loop() {
	// lineSensor->read();
	// const LineSensor::TSensorArray& values = lineSensor->sensorValues();

	// sprintf(buffer, "Position: %d, values: L> %d, %d, %d, %d, %d, %d, %d, %d <R",
	// 	lineSensor->position(),
	// 	values[0],
	// 	values[1],
	// 	values[2],
	// 	values[3],
	// 	values[4],
	// 	values[5],
	// 	values[6],
	// 	values[7]
	// 	);
	
	// rlog->info("Number: %d, quad: %d", loopCounter++, QuadratureEncoder::Counter());
	// rlog->info(buffer);
	rlog->info("Queue len: %d", motor->queueLength());
	//quadratureEncoder->info();
	motor->run();
	nh.spinOnce();
	delay(500);
}