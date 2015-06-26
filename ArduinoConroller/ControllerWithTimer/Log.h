#ifndef __LOG_H
#define __LOG_H

class Log {
  public:
  Log() {
    Serial.begin(57600);
    Serial3.begin(57600);
  }
  
  void waitSerial() {
    Serial.println("Enter key to start");
    Serial3.println("Enter key to start");
    while (!Serial.available() && !Serial3.available());
    while ((Serial.available() && Serial.read()) || (Serial3.available() && Serial3.read()));
//    delay(10000);
  }

  void print(const char* s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void print(double s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void print(float s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void print(int s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void print(long s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void print(unsigned int s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void print(unsigned long s) {
    Serial.print(s);
    Serial3.print(s);
  }

  void println(const char* s) {
    Serial.println(s);
    Serial3.println(s);
  }

  void println(double s) {
    Serial.println(s);
    Serial3.println(s);
  }
};

#endif
