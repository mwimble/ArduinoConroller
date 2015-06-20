#ifndef __QUADRATURE_ENCODER_H
#define __QUADRATURE_ENCODER_H

#include <Arduino.h>

class QuadratureEncoder {
private:
  static const int QEA = 4;  // Quadrature Encoder Signal A (black wire) => INT-4, Pin 19.
  static const int QEB = 5;  // Quadrature Encoder Signal B (white wire) => INT-5, Pin 18.
  static const int SIGA = 19;
  static const int SIGB = 18;

  static long counter;		// Running counter.

public:
	QuadratureEncoder() {
		pinMode(QEA, INPUT_PULLUP);
		pinMode(QEB, INPUT_PULLUP);
		attachInterrupt(QEA, QuadratureEncoder::qeaChange, CHANGE);
		attachInterrupt(QEB, QuadratureEncoder::qebChange, CHANGE);
	}

  	static long Counter() {
                static QuadratureEncoder singleton;
  		return singleton.counter;
	}

private:
	static void qeaChange() {
		int a = digitalRead(SIGA);
		int b = digitalRead(SIGB);
		if (a == b) counter++;
		else counter--;
	}

	static void qebChange() {
		int a = digitalRead(SIGA);
		int b = digitalRead(SIGB);
		if (a != b) counter++;
		else counter--;
	}

};

//long QuadratureEncoder::counter;		// Running counter.

#endif
