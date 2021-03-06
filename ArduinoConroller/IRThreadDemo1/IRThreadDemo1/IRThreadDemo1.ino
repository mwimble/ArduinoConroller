#include <Timer5.h>
#include <Thread.h>
#include <ThreadController.h>
#include <QTRSensors.h>
#include <Wire.h>

#include "I2CThread.h"
#include "LineSensorThread.h"
#include "MotorDriver.h"
#include "QuadratureEncoderThread.h"
#include "Strategy.h"

static unsigned long totalMillisecondCounter = 0;

LineSensorThread* lineSensorThread;
QuadratureEncoderThread* quadratureEncoderThread;
ThreadController controller = ThreadController();
//I2CThread* i2cThread;

#define MAP_ROWS 160
#define CLICKS_PER_ROW 5
#define SENSOR_COLS 8

//int mapP[MAP_ROWS][8] = { 0 };

ISR(timer5Event) {
  // 10ms timer
  resetTimer5();
  controller.run();
//  int counterRow = quadratureEncoderThread->lastCounter / CLICKS_PER_ROW;
//  
//  if (counterRow < 0) { counterRow = 0; }
//  else if (counterRow >= MAP_ROWS) { counterRow = MAP_ROWS -1; }
//  
//  for (int col = 0; col < SENSOR_COLS; col++) {
//    int lrCol = (SENSOR_COLS - col) - 1;
//    if ((counterRow < 0) || (counterRow >= MAP_ROWS)) {
//     Serial.print("counterRow range error: ");
//     Serial.println(counterRow);
//    } else {
//      if ((lrCol < 0) || (lrCol >= 8)) {
//        Serial.print("lrCol range error: ");
//        Serial.println(lrCol);
//      } else {
//        mapP[counterRow][lrCol] = lineSensorThread->sensorValues[col];
//      }
//    }
//  }
}

Strategy* strategy;
void setup() {
  Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(20000);
  Serial.println("setup start");
  motorSetup();
  lineSensorThread = new LineSensorThread();
  quadratureEncoderThread = new QuadratureEncoderThread();
  strategy = new Strategy(lineSensorThread, quadratureEncoderThread);

//  unsigned int initSensorValues[8]  = {20, 37, 177, 886, 836, 2500, 0, 0};
//  for (int xx = 0; xx < 8; xx++) lineSensorThread->sensorValues[xx] = initSensorValues[xx];
//  Serial.print("Expect 3347 for findLineDirection: ");Serial.println(findLineDirection());
//  StopSystem();

  controller.add(lineSensorThread);
  controller.add(quadratureEncoderThread);
//  i2cThread = new I2CThread();
  Serial.println("setup end");
  delay(1000);
  startTimer5(50000);
}

void loop() {
//  Serial.println("-- Start mapping");
//  while ((quadratureEncoderThread->lastCounter / CLICKS_PER_ROW) < MAP_ROWS) {
//    Serial.print("Counter: ");
//    Serial.println(quadratureEncoderThread->lastCounter);
//    delay(500);
//  }
//  
//  Serial.println("-- End mapping");
//  
//  for (int mapx = MAP_ROWS - 1; mapx >= 0; mapx--) {
//    Serial.print("[");
//    Serial.print(mapx);
//    Serial.print("]\t");
//    for (int colx = 0; colx < SENSOR_COLS; colx++) {
//      Serial.print(mapP[mapx][colx]);
//      Serial.print("\t");
//    }
//    
//    Serial.println();
//  }
//  
//  Serial.println("-- END --");
  
  strategy->doStrategy();
  delay(10);
}

