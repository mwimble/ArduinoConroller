#ifndef __STRAGEGY_H
#define __STRAGEGY_H

extern unsigned long myMicros();
extern LineSensor lineSensor;
extern unsigned int linePosition;
extern Motor motor;
extern QuadratureEncoder quadratureEncoder;
extern SensorStick sensorStick;

class Strategy {
  public:
  static const int kSLOW_SPEED = 64;
  static const int kLINE_DETECTED_THRESHOLD = 300;

  typedef enum TState {
    FOLLOW_LINE,
    FIND_LINE_END,
    FOUND_END,
    STOP,
  };
  
  static const char* kSTATE_NAMES[4];
  
  Strategy() {
  }
  
  void Initialize() {
    Motor::Stop();
    state_ = FOLLOW_LINE;
  }
  
  static void Dump(char* message) {
    bool leftTurnFound = lineSensorValues_[0] > kLINE_DETECTED_THRESHOLD;
    bool rightTurnFound = lineSensorValues_[7] > kLINE_DETECTED_THRESHOLD;
    float position = (linePosition * 1.0) / 1000.0;
    float ms = ((myMicros() * 1.0) - start_time_) / 1000.0;
    Serial.print(ms);
    Serial.print(" :: ");
    Serial.print(message);
    Serial.print(" [Odo:");
    Serial.print(quadratureEncoder.Counter());
    Serial.print("] State: ");
    Serial.print(kSTATE_NAMES[state_]);
    Serial.print(", Position: ");
    Serial.print(position);
    Serial.print(leftTurnFound ? "  *L " : "   L ");
    for (int i = 0; i < 8; i++) {
      Serial.print(lineSensorValues_[i]);
      Serial.print(" ");
    }
    
    Serial.print(rightTurnFound ? "R* " : "R  ");
    Serial.print(", YAW: ");
    Serial.print(sensorStick.Heading());
    Serial.print(", s: ");
    Serial.print(lineStartOdo_);
    Serial.print(", e: ");
    Serial.print(lineEndOdo_);
    Serial.print(", len: ");
    Serial.println(lineEndOdo_ - lineStartOdo_);
  }
  
  static void Stop() {
    state_ = STOP;
    Motor::Stop();
  }
  
  static void Process() {
    bool leftTurnFound = lineSensorValues_[0] > kLINE_DETECTED_THRESHOLD;
    bool rightTurnFound = lineSensorValues_[7] > kLINE_DETECTED_THRESHOLD;
    float position = (linePosition * 1.0) / 1000.0;
    
    switch (state_) {
      case FOLLOW_LINE:
        if (leftTurnFound || rightTurnFound) {
          lineStartOdo_ = quadratureEncoder.Counter();
          lineEndOdo_ = lineStartOdo_;
          state_ = FIND_LINE_END;
          Dump("+++ FOLLOW_LINE FOUND LINE START");
        } else {
          if (position < 3.0) {
            Motor::Left(kSLOW_SPEED, kSLOW_SPEED);
            delay(50);
            Motor::Stop();
            delay(50);
            position = (linePosition * 1.0) / 1000.0;
            Dump("--- FOLLOW_LINE correcting with left turn");
          } else if (position > 4.0) {
            Motor::Right(kSLOW_SPEED, kSLOW_SPEED);
            delay(50);
            Motor::Stop();
            delay(50);
            position = (linePosition * 1.0) / 1000.0;
            Dump("--- FOLLOW_LINE correcting with right turn");
          } else {
            Dump("--- FOLLOW_LINE on course");
          }
          
          Motor::Forward(64, 64);
        }

        break;
        
      case FIND_LINE_END:
        lineEndOdo_ = quadratureEncoder.Counter();
        if (quadratureEncoder.Counter() > (lineStartOdo_ + 700)) {
          state_ = FOUND_END;
          Dump("+++ FIND_LINE_END FOUND LINE END BY MAX DISTANCE");
        } else if (!leftTurnFound && !rightTurnFound) {
          state_ = FOUND_END; //#####
          Dump("+++ FIND_LINE_END FOUND LINE END BY IR");
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
  static unsigned long start_time_;
};

const LineSensor::TSensorArray& Strategy::lineSensorValues_ = LineSensor::SensorValues();
int Strategy::a_speed_ = 64;
int Strategy::b_speed_ = 64;
float Strategy::heading_;
long Strategy::lineEndOdo_;
long Strategy::lineStartOdo_;
Strategy::TState Strategy::state_ = FOLLOW_LINE;
unsigned long Strategy::start_time_ = myMicros();

const char* Strategy::kSTATE_NAMES[4] = {
  "FOLLOW_LINE",
  "FINE_LINE_END",
  "FOUND_END",
  "STOP"
};
    
#endif

