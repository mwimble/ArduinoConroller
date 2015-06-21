#ifndef __STRAGEGY_H
#define __STRAGEGY_H

extern LineSensor lineSensor;
extern Motor motor;
extern QuadratureEncoder quadratureEncoder;
extern SensorStick sensorStick;

class Strategy {
  public:

  typedef enum TState {
    FOLLOW_LINE,
    FIND_LINE_END,
    FOUND_END,
    STOP,
  };
    
  Strategy() {
  }
  
  void Initialize() {
    Motor::Stop();
    state_ = FOLLOW_LINE;
    Motor::Forward(a_speed_, b_speed_);
  }
  
  static void Dump(char* message) {
    bool leftTurnFound = lineSensorValues_[0] > 120;
    bool rightTurnFound = lineSensorValues_[7] > 120;
    float position = (lineSensor.Position() * 1.0) / 1000.0;
    Serial.print(message);
    Serial.print(" [Odo:");
    Serial.print(quadratureEncoder.Counter());
    Serial.print("] State: ");
    Serial.print(state_);
    Serial.print(", Position: ");
    Serial.print(position);
    Serial.print(leftTurnFound ? "  *L" : "   L");
    for (int i = 0; i < 8; i++) {
      Serial.print(lineSensorValues_[i]);
      Serial.print(" ");
    }
    
    Serial.print(rightTurnFound ? "R*, A: " : "R , A: ");
    Serial.print(Motor::SpeedA());
    Serial.print(", B: ");
    Serial.print(Motor::SpeedB());
    Serial.print(", YAW: ");
    Serial.print(sensorStick.Heading());
    Serial.print(", s: ");
    Serial.print(lineStartOdo_);
    Serial.print(", e: ");
    Serial.print(lineEndOdo_);
    Serial.print(", len: ");
    Serial.println(lineEndOdo_ - lineStartOdo_);
  }
  
  static void Process() {
    const int kSLOW_SPEED = 64;
    bool leftTurnFound = lineSensorValues_[0] > 120;
    bool rightTurnFound = lineSensorValues_[7] > 120;
    float position = (lineSensor.Position() * 1.0) / 1000.0;
    
    switch (state_) {
      case FOLLOW_LINE:
        Serial.println("+++ FOLLOW_LINE");
        if (leftTurnFound || rightTurnFound) {
          lineStartOdo_ = quadratureEncoder.Counter();
          state_ = FIND_LINE_END;
          Dump("+++ FOLLOW_LINE FOUND LINE START");
        } else { // Do PID.
        /*
          if (position < 3.0) {
            a_speed_ = a_speed_ - 10;
            b_speed_ = b_speed_ + 10;
          } else if (position > 4.0) {
            a_speed_ = a_speed_ + 10;
            b_speed_ = b_speed_ - 10;
          }
          
          if (a_speed_ < 35) a_speed_ = 35;
          if (b_speed_ < 35) b_speed_ = 35;
          if (a_speed_ > 127) a_speed_ = 127;
          if (b_speed_ > 127) b_speed_ = 127;
                    
          Serial.print(">>> Did PID, position: ");
          Serial.print(position);
          Serial.print(", new a_speed_: ");
          Serial.print(a_speed_);
          Serial.print(", new b_speed_: ");
          Serial.println(b_speed_);
          
          Motor::Forward(a_speed_, b_speed_);
          */
          if (position < 3.0) {
            while (position < 3.4) {
              Motor::Stop();
              delay(200);
              Motor::Left(kSLOW_SPEED, kSLOW_SPEED);
              delay(200);
              Dump("--- correcting with left turn");
            }            
          } else if (position > 4.0) {
            while (position > 3.4) {
              Motor::Stop();
              delay(200);
              Motor::Right(kSLOW_SPEED, kSLOW_SPEED);
              delay(200);
              Dump("--- correcting with right turn");
            }            
          }
          
          Motor::Forward(64, 64);
        }

        break;
        
      case FIND_LINE_END:
        lineEndOdo_ = quadratureEncoder.Counter();
        if (quadratureEncoder.Counter() > (lineStartOdo_ + 700)) {
          state_ = FOUND_END;
          Motor::Stop();
        } else if (!leftTurnFound && !rightTurnFound) {
          state_ = FOUND_END; //#####
          Dump("+++ FOLLOW_LINE FOUND LINE END BY IR");
        }
        
        break;
        
      case FOUND_END:
        Dump("+++ FOUND_END");
        Motor::Stop();
        state_ = STOP;
        break;
        
      case STOP:
        Dump("+++ STOP");
        Motor::Stop();
        break;
    }
  }
  
  static TState State() { return state_; }
  
  private:
  static const LineSensor::TSensorArray& lineSensorValues_;
  static int a_speed_;
  static int b_speed_;
  static float heading_;
  static long lineEndOdo_;
  static long lineStartOdo_;
  static TState state_;
};

const LineSensor::TSensorArray& Strategy::lineSensorValues_ = lineSensor.SensorValues();
int Strategy::a_speed_ = 64;
int Strategy::b_speed_ = 64;
float Strategy::heading_;
long Strategy::lineEndOdo_;
long Strategy::lineStartOdo_;
Strategy::TState Strategy::state_ = FOLLOW_LINE;

#endif

