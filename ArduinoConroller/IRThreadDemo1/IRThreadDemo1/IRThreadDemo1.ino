#include <Timer5.h>
#include <Thread.h>
#include <ThreadController.h>
#include <QTRSensors.h>
#include "LineSensorThread.h"
#include "QuadratureEncoderThread.h"

static unsigned long totalMillisecondCounter = 0;

LineSensorThread* lineSensorThread;
QuadratureEncoderThread* quadratureEncoderThread;
ThreadController controller = ThreadController();

ISR(timer5Event) {
  // 10ms timer
  resetTimer5();
  controller.run();
}

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(20000);
  Serial.println("setup start");
  lineSensorThread = new LineSensorThread();
  quadratureEncoderThread = new QuadratureEncoderThread();
  controller.add(lineSensorThread);
  controller.add(quadratureEncoderThread);
  Serial.println("setup end");
  delay(1000);
  startTimer5(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  Serial.print("[");Serial.print(lineSensorThread->readCount);Serial.print("] ");
  Serial.print("position: ");
  Serial.print(lineSensorThread->position);
  Serial.print(" -- ");
  for (int i = 0; i < 8; i++) {
    Serial.print(" ");
    Serial.print(lineSensorThread->sensorValues[i]);
  }

  Serial.println();
  
  Serial.print("Counter: ");
  Serial.println(quadratureEncoderThread->lastCounter);

  Serial.println();  
}

