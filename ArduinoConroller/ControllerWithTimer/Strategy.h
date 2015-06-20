#ifndef __STRAGEGY_H
#define __STRAGEGY_H

extern LineSensor lineSensor;
extern QuadratureEncoder quadratureEncoder;
extern Motor motor;

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
  
  void Process() {
    bool leftTurnFound = lineSensorValues_[0] > 120;
    bool rightTurnFound = lineSensorValues_[7] > 120;
    float position = lineSensor.Position() / 1000.0;
    Serial.print("[Odo:");
    Serial.print(quadratureEncoder.Counter());
    Serial.print("] ");
    
    switch (state_) {
      case FOLLOW_LINE:
        Serial.println("+++ FOLLOW_LINE");
        if (leftTurnFound || rightTurnFound) {
          lineStartOdo_ = quadratureEncoder.Counter();
          Serial.print(">>> FOUND start of crossing, odo: ");
          Serial.print(lineStartOdo_);
          Serial.print(", left: ");
          Serial.print(leftTurnFound);
          Serial.print(", right: ");
          Serial.println(rightTurnFound);
          state_ = FIND_LINE_END;
        } else { // Do PID.
          if (position < 3.0) {
            a_speed_ -= 20;
            b_speed_ += 20;
          } else if (position > 4.0) {
            a_speed_ += 20;
            b_speed_ -= 20;
          }
          
          if (a_speed_ < 0) a_speed_ = 0;
          if (b_speed_ < 0) b_speed_ = 0;
          if (a_speed_ > 127) a_speed_ = 127;
          if (b_speed_ > 127) b_speed_ = 127;

          /*
          if ((a_speed_ > 250) || (b_speed_ < 0)) {
            Motor::Stop();
            Serial.print("Correction out of range. Position: ");
            Serial.print(position);
            Serial.print(", a_speed: ");
            Serial.print(a_speed_);
            Serial.print(", b_speed: ");
            Serial.println(b_speed_);
            state_ = STOP;
          } else {
            Serial.print("... ... PID. Position: ");
            Serial.print(position);
            Serial.print(", a_speed: ");
            Serial.print(a_speed_);
            Serial.print(", b_speed: ");
            Serial.println(b_speed_);
            Motor::Forward(a_speed_, b_speed_);
          }
          */
        }
                    
        Motor::Forward(a_speed_, b_speed_);
        break;
        
      case FIND_LINE_END:
        Serial.println("+++ FOLLOW_LINE_END");
        if (quadratureEncoder.Counter() > (lineStartOdo_ + 700)) {
          state_ = FOUND_END;
          Motor::Stop();
        } else if (!leftTurnFound && !rightTurnFound) {
          lineEndOdo_ = quadratureEncoder.Counter();
          Serial.print(">>> Found line end Odo:");
          Serial.print(lineEndOdo_);
          Serial.print(", width: ");
          Serial.print(lineEndOdo_ - lineStartOdo_);
          Serial.print(" Odos or ");
          Serial.print((lineEndOdo_ - lineStartOdo_) / 0.0067);
          Serial.println(" IN");
          state_ = FOUND_END; //#####
        }
        
        break;
        
      case FOUND_END:
        Serial.println("+++ FOUND_END");
        Motor::Stop();
        state_ = STOP;
        break;
        
      case STOP:
        Serial.println("+++ STOP");
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

