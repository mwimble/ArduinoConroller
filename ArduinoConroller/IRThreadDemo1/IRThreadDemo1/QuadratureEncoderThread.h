#ifndef _QUADRATURE_ENCODER_THREAD
#define _QUADRATURE_ENCODER_THREAD

#include "QuadratureEncoder.h"

class QuadratureEncoderThread: public Thread {
  private:
  QuadratureEncoder qe = QuadratureEncoder();
  
  public:
  void run();
  
  long counter() {
    return qe.Counter();
  }
  
  static long lastCounter;
};

void QuadratureEncoderThread::run() {
  lastCounter = counter();
  runned();
}

long QuadratureEncoderThread::lastCounter = 0;
#endif
