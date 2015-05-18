#include <Timer5.h>
#include <Thread.h>
#include <ThreadController.h>
#include <QTRSensors.h>
#include "LineSensorThread.h"
//extern volatile unsigned long timer0_overflow_count;

static unsigned long totalMillisecondCounter = 0;

LineSensorThread* lineSensorThread;
ThreadController controller = ThreadController();

//unsigned long myMicros() {
//     unsigned long m, t;
//     uint8_t oldSREG = SREG;
//
//     cli();
//     t = TCNT0;
//
//// check if overflow is pending
//#ifdef TIFR0
//     if (TIFR0 & _BV(TOV0))
//           t+=256;
//#else
//     if (TIFR & _BV(TOV0))
//           t+=256;
//#endif
//
//     m = timer0_overflow_count;
//     SREG = oldSREG;
//
//     sei();
//     return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
//} 

unsigned long totalMillis = 0.0;
unsigned long loopCount = 0.0;
ISR(timer5Event) {
  // 10ms timer
  resetTimer5();
//  Serial.println("I5");
//  unsigned long lastClock = myMicros();
//  Serial.println("I5a");
//  unsigned long delay = 0;
  //totalMillisecondCounter++;
  controller.run();
//  unsigned long nextClock = myMicros();
//  delay = (nextClock - lastClock);
//  totalMillis += delay;
//  loopCount++;
//  Serial.print("I5: ");
//  Serial.println(totalMillis / loopCount);
}

//unsigned long foo(int k) {
//  unsigned long lastClock = micros();
//  unsigned long delay = 0;
//  do {
//      unsigned long nextClock = micros();
//      delay = (nextClock - lastClock);
//  } while (delay < k);
//  
//  return delay;
//}
void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  
//for (int i = 0; i < 1000; i += 10) {
//  unsigned long start = micros();
//  unsigned long delay = foo(i);
//  unsigned long now = micros() - start;
//  Serial.print("[");Serial.print(i);Serial.print("] d:");Serial.print(delay);Serial.print(" - n:");Serial.println(now);
//}  
  Serial.setTimeout(20000);
  Serial.println("setup start");
  lineSensorThread = new LineSensorThread();
  //lineSensorThread->setInterval(10);
  controller.add(lineSensorThread);
  //controller.run();
  //lineSensorThread->enabled = true;
  Serial.println("setup end");
  //lineSensorThread->run();
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
}
