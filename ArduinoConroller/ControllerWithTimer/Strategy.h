#ifndef __STRAGEGY_H
#define __STRAGEGY_H

// TODO
// * Average two LineSensor readings to reduce anomolies.

extern unsigned long myMicros();
extern LineSensor lineSensor;
extern unsigned int linePosition;

class Strategy {
  public:
  static const int kSLOW_SPEED = 64;
  static const int kTURN_SPEED = 80;
  static const int kLINE_DETECTED_THRESHOLD = 400;
  static const int kMAKE_TURN_DELTA_YAW = 55; // Min degrees to turn before resampling line.
  static const float kINCHES_PER_ODO = 0.0068; // Inches traveled in one odometer click.
  static const long kODOS_TO_OVERSHOOT_LINE = 147 * 2; // Odos to trave beyone far edge of line before starting turn.
  static const long kODOS_TO_SAMPLE_CONTINUATION = 147; // Odos to sample past line end to look for line continuation.
  
  private:
    static bool CenterLineFound() {
      bool partA = line_sensor_values_[LineSensor::NUM_SENSORS / 2] >= kLINE_DETECTED_THRESHOLD;
      bool partB = (LineSensor::NUM_SENSORS % 2) == 1 ? false 
                                                    : line_sensor_values_[(LineSensor::NUM_SENSORS / 2) - 1] >= kLINE_DETECTED_THRESHOLD;
      return partA || partB;
    }
    
    static bool LeftLineFound() {
      return line_sensor_values_[0] >= kLINE_DETECTED_THRESHOLD;
    }
    
    static bool RightLineFound() {
      return line_sensor_values_[LineSensor::NUM_SENSORS - 1] >= kLINE_DETECTED_THRESHOLD;
    }

  static const unsigned int kLEFT_LINE_SEGMENT = 0;
  static const unsigned int kCENTER_LINE_SEGMENT = 1;
  static const unsigned int kRIGHT_LINE_SEGMENT = 2;
  
  static unsigned int segment_detection_end_samples_; // Number of left/right line sample observations.
  static unsigned int segment_detection_center_samples_; // Number of center line sample observations.
  static unsigned int segment_detection_counts_[3];
  
  static void StartLineImaging() {
    segment_detection_end_samples_ = 0;
    segment_detection_center_samples_ = 0;
    for (int i = 0; i < 3; i++) segment_detection_counts_[i] = 0;
  }
  
  static void ImageLine() {
    if ((state_ != STOPPING)  && (state_ != PRE_ROLL)) {
      // If we are crossing a line, sample only the left and right line segments.
      if (LeftLineFound()) segment_detection_counts_[kLEFT_LINE_SEGMENT]++;
      if (RightLineFound()) segment_detection_counts_[kRIGHT_LINE_SEGMENT]++;
      segment_detection_end_samples_++;
    } else {
      // If we have crossed the line, sample only the center line segment.
      if ((last_stopping_odo_ + kODOS_TO_SAMPLE_CONTINUATION) <= QuadratureEncoder::Counter()) {
        if (CenterLineFound()) segment_detection_counts_[kCENTER_LINE_SEGMENT]++;
        segment_detection_center_samples_++;
      }
    }
  }
  
  static bool LeftLineSegmentFound() {
    return segment_detection_counts_[kLEFT_LINE_SEGMENT] >= (segment_detection_end_samples_ / 2);
  }

  static bool CenterLineSegmentFound() {
    return segment_detection_counts_[kCENTER_LINE_SEGMENT] >= (segment_detection_center_samples_ / 2);
  }

  static bool RightLineSegmentFound() {
    return segment_detection_counts_[kRIGHT_LINE_SEGMENT] >= (segment_detection_end_samples_ / 2);
  }
  
  public:

  enum TState {
    FOLLOW_LINE = 0,
    FIND_LINE_END,
    FOUND_END,
    STOPPING,
    PRE_ROLL,
    STOP,
    LEFT_TURN_MIN_YAW,
    LEFT_TURN,
    RIGHT_TURN_MIN_YAW,
    RIGHT_TURN,
    END_OF_ENUM_DONT_USE
  };
  
  static const char* kSTATE_NAMES[END_OF_ENUM_DONT_USE];
  
  Strategy() {
  }
  
  void Initialize() {
    Motor::Stop();
    state_ = FOLLOW_LINE;
  }
  
  static void Dump(char* message) {
    bool leftTurnFound = LeftLineFound();
    bool rightTurnFound = RightLineFound();
    float position = (linePosition * 1.0) / 1000.0;
    float ms = ((myMicros() * 1.0) - start_time_) / 1000.0;
    Serial.print(ms);
    Serial.print(" :: ");
    Serial.print(message);
    Serial.print(" [Odo:");
    Serial.print(QuadratureEncoder::Counter());
    Serial.print("] State: ");
    Serial.print(kSTATE_NAMES[state_]);
    Serial.print(", Position: ");
    Serial.print(position);
    Serial.print(leftTurnFound ? "  *L " : "   L ");
    for (int i = 0; i < 8; i++) {
      Serial.print(line_sensor_values_[i]);
      Serial.print(" ");
    }
    
    Serial.print(rightTurnFound ? "R* " : "R  ");
    Serial.print(", YAW: ");
    Serial.print(SensorStick::Heading());
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
    float goalHeading;
    bool leftTurnFound = LeftLineFound();
    bool rightTurnFound = RightLineFound();
    float position = (linePosition * 1.0) / 1000.0;
    
    switch (state_) {
      case FOLLOW_LINE:
        if (leftTurnFound || rightTurnFound) {
          lineStartOdo_ = quadratureEncoder.Counter();
          lineEndOdo_ = lineStartOdo_;
          state_ = FIND_LINE_END;
          StartLineImaging();
          Dump("+++ FOLLOW_LINE FOUND LINE START");
        } else {
          if (position < 3.0) {
            Motor::Left(kTURN_SPEED, kTURN_SPEED);
//            delay(50);
//            Motor::Stop();
//            delay(50);
            position = (linePosition * 1.0) / 1000.0;
            Dump("--- FOLLOW_LINE correcting with left turn");
          } else if (position > 4.0) {
//            Motor::Right(kTURN_SPEED, kTURN_SPEED);
//            delay(50);
//            Motor::Stop();
//            delay(50);
            position = (linePosition * 1.0) / 1000.0;
            Dump("--- FOLLOW_LINE correcting with right turn");
          } else {
            Dump("--- FOLLOW_LINE on course");
          }
          
          Motor::Forward(kSLOW_SPEED, kSLOW_SPEED);
        }

        break;
        
      case FIND_LINE_END:
        ImageLine();
        if ((!leftTurnFound && !rightTurnFound) && (lineEndOdo_ == lineStartOdo_)) {
          // Capture only once.
          lineEndOdo_ = quadratureEncoder.Counter();
        }
        
        if (quadratureEncoder.Counter() > (lineStartOdo_ + 700)) {
          state_ = FOUND_END;
          Dump("+++ FIND_LINE_END FOUND LINE END BY MAX DISTANCE");
        } else if (!leftTurnFound && !rightTurnFound) {
          state_ = FOUND_END; //#####
          Dump("+++ FIND_LINE_END FOUND LINE END BY IR");
        }
        
        break;
        
      case FOUND_END:
        last_stopping_odo_ = quadratureEncoder.Counter();
        state_ = PRE_ROLL;
        break;
        
      case PRE_ROLL:
        Dump("+++ PRE_ROLL");
        Serial.print("goal odo: ");
        Serial.println(lineEndOdo_ + kODOS_TO_OVERSHOOT_LINE);
        if (quadratureEncoder.Counter() > (lineEndOdo_ + kODOS_TO_OVERSHOOT_LINE)) {
          Motor::Stop();
          state_ = STOPPING;
        }
        
        break;
        
      case STOPPING:
        ImageLine();
        if (quadratureEncoder.Counter() != last_stopping_odo_) {
          // Keep sampling until full stop.
          Dump("+++ STOPPING WAITING FOR FULL STOP");
          last_stopping_odo_ = quadratureEncoder.Counter();
        } else {
          turn_start_heading_ = SensorStick::Heading();
          Dump("+++ STOPPING FOUND_END");
          Serial.print("Left/right segment samples: ");
          Serial.print(segment_detection_end_samples_);
          Serial.print(", center segment samples: ");
          Serial.print(segment_detection_center_samples_);
          Serial.print(", L");
          Serial.print(LeftLineSegmentFound() ? "*" : "");
          Serial.print(": ");
          Serial.print(segment_detection_counts_[kLEFT_LINE_SEGMENT]);
          Serial.print(", C");
          Serial.print(CenterLineSegmentFound() ? "*" : "");
          Serial.print(": ");
          Serial.print(segment_detection_counts_[kCENTER_LINE_SEGMENT]);
          Serial.print(", R");
          Serial.print(RightLineSegmentFound() ? "*" : "");
          Serial.print(": ");
          Serial.println(segment_detection_counts_[kRIGHT_LINE_SEGMENT]);
          if (LeftLineSegmentFound()) {
            Map* newMap = new Map(current_map_, 
                                  Map::L,
                                  Map::kLeftOf[current_map_->LogicalHeading()],
                                  SensorStick::Heading(),
                                  LeftLineSegmentFound(),
                                  RightLineSegmentFound(),
                                  CenterLineSegmentFound());
            state_ = LEFT_TURN_MIN_YAW;
          } else if (RightLineSegmentFound()) {
            Map* newMap = new Map(current_map_, 
                                  Map::R,
                                  Map::kRightOf[current_map_->LogicalHeading()],
                                  SensorStick::Heading(),
                                  LeftLineSegmentFound(),
                                  RightLineSegmentFound(),
                                  CenterLineSegmentFound());
            state_ = RIGHT_TURN_MIN_YAW;
          } else if (CenterLineSegmentFound()) {
            Map* newMap = new Map(current_map_, 
                                  Map::C,
                                  current_map_->LogicalHeading(),
                                  SensorStick::Heading(),
                                  LeftLineSegmentFound(),
                                  RightLineSegmentFound(),
                                  CenterLineSegmentFound());
            state_ = FOLLOW_LINE;
          } else {
            // Do something interesting.
            Serial.println("@@@ DEAD END @@@");
            state_ = STOP;
          }
        }
        
        break;
        
      case RIGHT_TURN_MIN_YAW:
        Dump("+++ RIGHT_TURN_MIN_YAW");
        goalHeading = ((long)(turn_start_heading_ + kMAKE_TURN_DELTA_YAW)) % 360;
        Serial.print("turn_start_heading_: ");
        Serial.print(turn_start_heading_);
        Serial.print(", goal heading: ");
        Serial.println(goalHeading);
        if (SensorStick::Heading() < goalHeading) {
          Motor::Right(kTURN_SPEED, kTURN_SPEED);
        } else {
          state_ = RIGHT_TURN;
        }
        
        break;
        
      case RIGHT_TURN:
        Dump("+++ RIGHT_TURN");
        if ((SensorStick::Heading() < (turn_start_heading_ - 10)) &&
            (SensorStick::Heading() > (turn_start_heading_ - kMAKE_TURN_DELTA_YAW))) {
          // Turned too far and not found line.
          Serial.println("@@@ @@@ FAILED TO FIND LINE AFTER RIGHT TURN");
          state_ = STOP;
          break;
        }
        
        if (position > 6.0) {
          Motor::Right(kTURN_SPEED, kTURN_SPEED);
        } else {
          state_ = STOP;
        }
        
        break;
      
      case LEFT_TURN_MIN_YAW:
        Dump("+++ LEFT_TURN_MIN_YAW");
        goalHeading = ((long)(turn_start_heading_ - kMAKE_TURN_DELTA_YAW)) % 360;
        Serial.print("turn_start_heading_: ");
        Serial.print(turn_start_heading_);
        Serial.print(", goal heading: ");
        Serial.println(goalHeading);
        if (SensorStick::Heading() > goalHeading) {
          Motor::Left(kTURN_SPEED, kTURN_SPEED);
        } else {
          state_ = LEFT_TURN;
        }
        
        break;
        
      case LEFT_TURN:
        Dump("+++ LEFT_TURN");
        if ((SensorStick::Heading() > (turn_start_heading_ + 10)) &&
            (SensorStick::Heading() < (turn_start_heading_ + kMAKE_TURN_DELTA_YAW))) {
          // Turned too far and not found line.
          Serial.println("@@@ @@@ FAILED TO FIND LINE AFTER LEFT TURN");
          state_ = STOP;
          break;
        }
        
        if (position < 1.0) {
          Motor::Left(kTURN_SPEED, kTURN_SPEED);
        } else {
          state_ = STOP;
        }
        
        break;
       
      case STOP:
        Dump("+++ STOP");
        Motor::Stop();
        break;
    }
  }
  
  static TState State() { return state_; }
  
  static Map* StartMap() { return start_map_; }
  
  private:
  static const LineSensor::TSensorArray& line_sensor_values_;
  static int a_speed_;
  static int b_speed_;
  static float heading_;
  static unsigned long last_stopping_odo_;
  static long lineEndOdo_;
  static long lineStartOdo_;
  static TState state_;
  static unsigned long start_time_;
  static Map* start_map_;
  static Map* current_map_;
  static float turn_start_heading_;
};

const LineSensor::TSensorArray& Strategy::line_sensor_values_ = LineSensor::SensorValues();
int Strategy::a_speed_ = 64;
int Strategy::b_speed_ = 64;
float Strategy::heading_;
long Strategy::lineEndOdo_;
long Strategy::lineStartOdo_;
Strategy::TState Strategy::state_ = FOLLOW_LINE;
unsigned long Strategy::start_time_ = myMicros();
unsigned int Strategy::segment_detection_end_samples_;
unsigned int Strategy::segment_detection_center_samples_;
unsigned int Strategy::segment_detection_counts_[3];
unsigned long Strategy::last_stopping_odo_;
Map* Strategy::start_map_ = new Map(NULL, Map::C, Map::E, SensorStick::Heading(), false, false, true);
Map* Strategy::current_map_ = Strategy::start_map_;
float Strategy::turn_start_heading_;

const char* Strategy::kSTATE_NAMES[END_OF_ENUM_DONT_USE] = {
  "FOLLOW_LINE",
  "FIND_LINE_END",
  "FOUND_END",
  "STOPPING",
  "PRE_ROLL",
  "STOP",
  "LEFT_TURN_MIN_YAW",
  "LEFT_TURN",
  "RIGHT_TURN_MIN_YAW",
  "RIGHT_TURN",
};
    
#endif

