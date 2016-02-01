#ifndef __QuadratureEncoder_h
#define __QuadratureEncoder_h

#include "Arduino.h"

class QuadratureEncoder {
  private:
  static const int QEA = 4;    // Interrupt 4 (pin 19) will be for quadrature signal A.
  static const int QEB = 5;    // Interrupt 4 (pin 19) will be for quadrature signal A.
  static const int SIGA = 19; // Quadrature Encoder Signal A (black wire) => INT-4, Pin 19.
  static const int SIGB = 18; // Quadrature Encoder Signal B (white wire) => INT-5, Pin 18.
  
  static volatile long counter;
  
  public:
  QuadratureEncoder();
  
  long Counter();
  
  private:
  static void qeaChange();
  static void qebChange();
};

#endif
