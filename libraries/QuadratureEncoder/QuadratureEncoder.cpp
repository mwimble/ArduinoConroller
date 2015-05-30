#include "QuadratureEncoder.h"

QuadratureEncoder::QuadratureEncoder() {
    pinMode(QEA, INPUT_PULLUP);
    pinMode(QEB, INPUT_PULLUP);
    attachInterrupt(QEA, qeaChange, CHANGE);
    attachInterrupt(QEB, qebChange, CHANGE);
}


void QuadratureEncoder::qeaChange() {
  int a = digitalRead(SIGA);
  int b = digitalRead(SIGB);
  if (a == b) counter++;
  else counter--;
}

void QuadratureEncoder::qebChange() {
  int a = digitalRead(SIGA);
  int b = digitalRead(SIGB);
  if (a != b) counter++;
  else counter--;
}

long QuadratureEncoder::Counter() {
  return counter;
}

long QuadratureEncoder::counter = 0;
