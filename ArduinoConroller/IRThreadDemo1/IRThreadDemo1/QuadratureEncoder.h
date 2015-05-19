
#ifndef QuadratureEncoder_h
#define QuadratureEncoder_h

#include "Arduino.h"

class QuadratureEncoder {
  private:
  static const int QEA = 4;  // Quadrature Encoder Signal A (black wire) => INT-4, Pin 19.
  static const int QEB = 5;  // Quadrature Encoder Signal B (white wire) => INT-5, Pin 18.
  static const int SIGA = 19;
  static const int SIGB = 18;
  
  static long counter;
  
  public:
  QuadratureEncoder();
  
  long Counter();
  
  private:
  static void qeaChange();
  static void qebChange();
};

#endif