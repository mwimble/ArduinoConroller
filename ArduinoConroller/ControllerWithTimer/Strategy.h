#ifndef __STRAGEGY_H
#define __STRAGEGY_H

extern LineSensor lineSensor;
extern QuadratureEncoder quadratureEncoder;
extern Motor motor;

class Strategy {
  public:
  
  Strategy() :
    a_speed_(64),
    b_speed_(64),
    state(FOLLOW_LINE) {
  }
  
  void Initialize() {
    motor.Stop();
    state = FOLLOW_LINE;
    motor.Forward(a_speed_, b_speed_);
  }
  
  void Process() {
    bool leftTurnFound = lineSensorValues_[0] > 200;
    bool rightTurnFound = lineSensorValues_[7] > 200;
    float position = lineSensor.Position() / 1000.0;
    Serial.print("[Odo:");
    Serial.print(quadratureEncoder.Counter());
    Serial.print("] ");
    
    switch (state) {
      case FOLLOW_LINE:
        Serial.println("+++ FOLLOW_LINE");
        if (leftTurnFound || rightTurnFound) {
          lineStartOdo = quadratureEncoder.Counter();
          Serial.print("... ... FOUND start of crossing, left: ");
          Serial.print(leftTurnFound);
          Serial.print(", right: ");
          Serial.println(rightTurnFound);
          state = FIND_LINE_END;
        } else { // Do PID.
          if (position < 3.0) {
            a_speed_ -= 10;
            b_speed_ += 10;
          } else if (position > 4.0) {
            a_speed_ += 10;
            b_speed_ -= 10;
          }
          
          if (a_speed_ < 0) a_speed_ = 0;
          if (b_speed_ < 0) b_speed_ = 0;
          if (a_speed_ > 127) a_speed_ = 127;
          if (b_speed_ > 127) b_speed_ = 127;

          /*
          if ((a_speed_ > 250) || (b_speed_ < 0)) {
            motor.Stop();
            Serial.print("Correction out of range. Position: ");
            Serial.print(position);
            Serial.print(", a_speed: ");
            Serial.print(a_speed_);
            Serial.print(", b_speed: ");
            Serial.println(b_speed_);
            state = STOP;
          } else {
            Serial.print("... ... PID. Position: ");
            Serial.print(position);
            Serial.print(", a_speed: ");
            Serial.print(a_speed_);
            Serial.print(", b_speed: ");
            Serial.println(b_speed_);
            motor.Forward(a_speed_, b_speed_);
          }
          */
        }
                    
        break;
        
      case FIND_LINE_END:
        Serial.println("+++ FOLLOW_LINE_END");
        if (quadratureEncoder.Counter() > (lineStartOdo + 200)) {
          state = FOUND_END;
          motor.Stop();
        } else if (!leftTurnFound && !rightTurnFound) {
          lineEndOdo = quadratureEncoder.Counter();
          state = FOUND_END; //#####
        }
        
        break;
        
      case FOUND_END:
        Serial.println("+++ FOUND_END");
        motor.Stop();
        break;
        
      case STOP:
        Serial.println("+++ STOP");
        motor.Stop();
        break;
    }
  }
  
  private:
  typedef enum State {
    FOLLOW_LINE,
    FIND_LINE_END,
    FOUND_END,
    STOP,
  };
  
  const LineSensor::TSensorArray& lineSensorValues_ = lineSensor.SensorValues();
  int a_speed_;
  int b_speed_;
  float heading_;
  long lineEndOdo;
  long lineStartOdo;
  State state;
};

#endif

