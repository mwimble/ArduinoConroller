#ifndef __IMH_H
#define __IMU_H

class IMU {
  public:
  typedef enum {
    PREFIXSTART,
    PREFIX,
    YAWINT
  } STATE;
  
  IMU() {
    Serial2.begin(57600);
  }
  
  static int Yaw() {
    return yaw;
  }
  
  static void processStream() {
    char buffer[64];
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      switch (state) {
        case PREFIXSTART: // Await # in #YPR
          if (c == '\n') return; // Give other threads a break.
          
          if (c == '#') state = PREFIX;
          
          break;
          
        case PREFIX: // Wait out #YPR=
          if ((c == 'Y') || (c == 'P') || (c == 'R') || (c == '=')) {
            if (c == '=') {
              yaw_sign = 1;
              yaw = 0;
              state = YAWINT;
            }
          } else {
            yaw = -999;
            state = PREFIXSTART; // Blew it, start over.
          }
                    
          break;
          
        case YAWINT:
          if (c == '-') {
            yaw_sign = -1;
          } else if ((c >= '0') && (c <= '9')) {
            yaw = 10 * yaw + (c - '0');
          } else if (c == '.') {
            yaw *= yaw_sign;
            state = PREFIXSTART;
          } else {
            // Something went wrong.
            yaw = -999;
            state = PREFIXSTART;
          }
          
          break;
      }
    }
  }
  
  String error;

  private:
  static STATE state;
  static int yaw;
  static int yaw_sign;
};

IMU::STATE IMU::state = PREFIXSTART;
int IMU::yaw = -999;
int IMU::yaw_sign = 1;

#endif

