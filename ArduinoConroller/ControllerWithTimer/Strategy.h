#ifndef __STRAGEGY_H
#define __STRAGEGY_H

// TODO
// * Average two LineSensor readings to reduce anomolies.

extern unsigned long myMicros();
extern LineSensor lineSensor;
extern unsigned int linePosition;

class Strategy {
  public:
  static const bool kDEBUG_FOLLOW_LINE = false;
  
  static const int kSLOW_SPEED = 64;
  static const int kTURN_SPEED = 80;
  static const int kLINE_DETECTED_THRESHOLD = 200;
  static const int kMAKE_TURN_DELTA_YAW = 20; // Min degrees to turn before resampling line.
  static const int kTURN_AROUND_DELTA_YAW = 125; // Min degrees to turn before resampling line.
  static const float kINCHES_PER_ODO = 0.0068; // Inches traveled in one odometer click.
  static const long kODOS_TO_OVERSHOOT_LINE = 147 * 2; // Odos to trave beyone far edge of line before starting turn.
  static const long kODOS_TO_SAMPLE_CONTINUATION = 147; // Odos to sample past line end to look for line continuation.
  static const long kMIN_NO_CENTER_IS_DEAD_END = 147; // Min Odos not finding a center sensor to cound as a dead end indication.
  static const long kODO_WIDTH_OF_STOP_SYMBOL = 300;
  
  private:
//    static void StartPreRoll();
//    static void DoPreRoll();
//    static void DoStopping();
    
    static bool AnyLineFound() {
      for (int i = 0; i < LineSensor::NUM_SENSORS; i++) {
        if (line_sensor_values_[i] >= (kLINE_DETECTED_THRESHOLD + 150)) return true;
      }
      
      return false;
    }
    
    static bool CenterLineFound() {
      bool partA = line_sensor_values_[LineSensor::NUM_SENSORS / 2] >= kLINE_DETECTED_THRESHOLD;
      bool partB = (LineSensor::NUM_SENSORS % 2) == 1 ? false 
                                                    : line_sensor_values_[(LineSensor::NUM_SENSORS / 2) - 1] >= kLINE_DETECTED_THRESHOLD;
      return partA || partB;
    }
    
    static bool LeftLineFound() {
      return line_sensor_values_[0] >= (kLINE_DETECTED_THRESHOLD + 150);
    }
    
    static bool RightLineFound() {
      return line_sensor_values_[LineSensor::NUM_SENSORS - 1] >= (kLINE_DETECTED_THRESHOLD + 150);
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
      if ((lineEndOdo_ + kODOS_TO_SAMPLE_CONTINUATION) <= QuadratureEncoder::Counter()) {
        if (CenterLineFound()) segment_detection_counts_[kCENTER_LINE_SEGMENT]++;
        segment_detection_center_samples_++;
      }
    }
  }
  
  static bool LeftLineSegmentFound() {
    return segment_detection_counts_[kLEFT_LINE_SEGMENT] > (segment_detection_end_samples_ / 2);
  }

  static bool CenterLineSegmentFound() {
    return segment_detection_counts_[kCENTER_LINE_SEGMENT] > (segment_detection_center_samples_ / 2);
  }

  static bool RightLineSegmentFound() {
    return segment_detection_counts_[kRIGHT_LINE_SEGMENT] > (segment_detection_end_samples_ / 2);
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
    DEAD_END_MIN_YAW,
    SOLVED,
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
  
  static int found_line_start_count_;
  
  static void Process() {
    static long lastCenterOdo = 0;
    bool leftTurnFound = LeftLineFound();
    bool rightTurnFound = RightLineFound();
    bool deadEndFound = CenterLineFound() ? false : (QuadratureEncoder::Counter() - lastCenterOdo) > kMIN_NO_CENTER_IS_DEAD_END;
    float position = (linePosition * 1.0) / 1000.0;
    
//    Serial.print("deadEndFound: ");
//    Serial.print(deadEndFound);
//    Serial.print(", lastCenterOdo: ");
//    Serial.println(lastCenterOdo);
    if (CenterLineFound()) {
      lastCenterOdo = QuadratureEncoder::Counter();
    }
    
    switch (state_) {
      case FOLLOW_LINE:
        if (deadEndFound) {
          found_line_start_count_ = 0;
          Dump("@@@ DEAD END FOUND");
          turn_start_heading_ = SensorStick::Heading();
          state_= DEAD_END_MIN_YAW;
          break;
        } else if (CenterLineFound() && (leftTurnFound || rightTurnFound)) {
          if (++found_line_start_count_ < 2) break; //#####
          
          lineStartOdo_ = QuadratureEncoder::Counter();
          lineEndOdo_ = lineStartOdo_;
          state_ = FIND_LINE_END;
          StartLineImaging();
          Motor::Forward(kSLOW_SPEED, kSLOW_SPEED);
          Dump("+++ FOLLOW_LINE FOUND LINE START");
        } else if (CenterLineFound()) {
          found_line_start_count_ = 0;
          if (position < 3.0) {
            Motor::Left(kTURN_SPEED, kTURN_SPEED);
//            delay(50);
//            Motor::Stop();
//            delay(50);
            if (kDEBUG_FOLLOW_LINE) {
              Dump("--- FOLLOW_LINE correcting with left turn");
            }
          } else if (position > 4.0) {
            Motor::Right(kTURN_SPEED, kTURN_SPEED);
//            delay(50);
//            Motor::Stop();
//            delay(50);
            if (kDEBUG_FOLLOW_LINE) {
              Dump("--- FOLLOW_LINE correcting with right turn");
            }
          } else {
            Motor::Forward(kSLOW_SPEED, kSLOW_SPEED);
            if (kDEBUG_FOLLOW_LINE) {
              Dump("--- FOLLOW_LINE on course");
            }
          }
        }

        break;
        
      case FIND_LINE_END:
        ImageLine();
        if ((!leftTurnFound && !rightTurnFound) && (lineEndOdo_ == lineStartOdo_)) {
          // Capture only once.
          lineEndOdo_ = QuadratureEncoder::Counter();
        }
        
        if ((lineEndOdo_ - lineStartOdo_) > kODO_WIDTH_OF_STOP_SYMBOL) {
          state_ = SOLVED;
        } else if (QuadratureEncoder::Counter() > (lineStartOdo_ + 700)) {
          state_ = FOUND_END;
          Dump("+++ FIND_LINE_END FOUND LINE END BY MAX DISTANCE");
        } else if (!leftTurnFound && !rightTurnFound) {
          state_ = FOUND_END; //#####
          Dump("+++ FIND_LINE_END FOUND LINE END BY IR");
        }
        
        break;
        
      case FOUND_END:
        StartPreRoll();        
        break;
        
      case PRE_ROLL:
        DoPreRoll();        
        break;
        
      case STOPPING:
        DoStopping();
        break;
        
      case RIGHT_TURN_MIN_YAW:
        DoMinRightYaw();
        break;
        
      case RIGHT_TURN:
        Dump("+++ RIGHT_TURN");
        if (false && IsCloseTo(SensorStick::Heading(), turn_start_heading_, 15)) {
          // Turned too far and not found line.
          Serial.println("@@@ @@@ FAILED TO FIND LINE AFTER RIGHT TURN");
          state_ = STOP;
          break;
        }
        
        if ((position > 4.0) || (position <= 0.0) || !AnyLineFound()) {
          Motor::Right(kTURN_SPEED, kTURN_SPEED);
        } else {
          state_ = FOLLOW_LINE;
        }
        
        break;
      
      case DEAD_END_MIN_YAW:
        DoMinLeftYaw();
        break;
        
      case LEFT_TURN_MIN_YAW:
        DoMinLeftYaw();
        break;
        
      case LEFT_TURN:
        Dump("+++ LEFT_TURN");
        if (false && IsCloseTo(SensorStick::Heading(), turn_start_heading_, 15)) {
          // Turned too far and not found line.
          Serial.println("@@@ @@@ FAILED TO FIND LINE AFTER LEFT TURN");
          state_ = STOP;
          break;
        }
        
        if ((position < 3.0) || (position >= 7.0) || !AnyLineFound()) {
          Serial.print("### LEFT TURN, looking for position < 3.0, position: ");
          Motor::Left(kTURN_SPEED, kTURN_SPEED);
        } else {
          Serial.print("### LEFT TURN position is > 3, follow line, position: ");
          state_ = FOLLOW_LINE;
          Serial.println(position);
        }
        
        break;
       
      case STOP:
        Dump("+++ STOP");
        Motor::Stop();
        break;
        
      case SOLVED:
        Dump("+++ SOLVED");
        Motor::Stop();
        break;
    }
  }
  
  static float FixYawRange(float yaw) {
    float result = yaw;
    while (result < 0.0) yaw += 360.0;
    while (result > 360.0) yaw -= 360.0;
    return result;
  }
  
  static bool IsCloseTo(float currentYaw, float targetYaw, float deltaDegrees) {
    float maxYaw = FixYawRange(targetYaw + deltaDegrees);
    float minYaw = FixYawRange(targetYaw - deltaDegrees);
    return (currentYaw > minYaw) && (currentYaw < maxYaw);
  }
  
  static TState State() { return state_; }
  
  static Map* StartMap() { return start_map_; }
  
  private:
  static long preroll_start_odo_;
  static long preroll_start_yaw_;
  static long preroll_goal_odo_;
  static const bool kDEBUG_PREROLL = false;
  
  static void StartPreRoll() {
    ImageLine();
    state_ = PRE_ROLL;
    preroll_start_odo_ = QuadratureEncoder::Counter();
    preroll_start_yaw_ = SensorStick::Heading();
    preroll_goal_odo_ = lineEndOdo_ + kODOS_TO_OVERSHOOT_LINE;
    Dump("+++ START PRE_ROLL");
    if (kDEBUG_PREROLL) {
      Serial.print("goal odo: ");
      Serial.println(preroll_goal_odo_);
    }
  }
  
  static long stopping_start_odo_;
  
  static void DoPreRoll() {
    ImageLine();
    if (kDEBUG_PREROLL) {
      Dump("++ PRE_ROLL");
    }
    
    if (QuadratureEncoder::Counter() >= preroll_goal_odo_) {
      Motor::Stop();
      last_stopping_odo_ = QuadratureEncoder::Counter();
      stopping_start_odo_ = QuadratureEncoder::Counter();
      state_ = STOPPING;

      if (kDEBUG_PREROLL) {
        Serial.print("goal odo: ");
        Serial.print(preroll_goal_odo_);
      }
      
      Dump("+++ START STOPPING");
    } else {
      Motor::Forward(kSLOW_SPEED, kSLOW_SPEED);
    }
  }

  static const bool kDEBUG_STOPPING = false;
  static long last_stopping_odo_;
  static float turn_start_heading_;

  static void DoStopping() {
    ImageLine();
    if (QuadratureEncoder::Counter() != last_stopping_odo_) {
      // Keep sampling until full stop.
      if (kDEBUG_STOPPING) {
          Dump("+++ STOPPING WAITING FOR FULL STOP");
      }
      
      last_stopping_odo_ = QuadratureEncoder::Counter();
    } else {
      // Finally came to a full stop.
      long fullStopOdo = QuadratureEncoder::Counter();
      turn_start_heading_ = SensorStick::Heading();
      Dump("+++ STOPPING FOUND_END");
      Serial.print("Start odo: ");
      Serial.print(stopping_start_odo_);
      Serial.print(", full stop odo: ");
      Serial.print(fullStopOdo);
      Serial.print(", distance traveled to stop: ");
      Serial.println(fullStopOdo - stopping_start_odo_);
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
        current_map_ = newMap;
        state_ = LEFT_TURN_MIN_YAW;
      } else if (CenterLineSegmentFound()) {
        Map* newMap = new Map(current_map_, 
                              Map::C,
                              current_map_->LogicalHeading(),
                              SensorStick::Heading(),
                              LeftLineSegmentFound(),
                              RightLineSegmentFound(),
                              CenterLineSegmentFound());
        current_map_ = newMap;
        state_ = FOLLOW_LINE;
      } else if (RightLineSegmentFound()) {
        Map* newMap = new Map(current_map_, 
                              Map::R,
                              Map::kRightOf[current_map_->LogicalHeading()],
                              SensorStick::Heading(),
                              LeftLineSegmentFound(),
                              RightLineSegmentFound(),
                              CenterLineSegmentFound());
        current_map_ = newMap;
        state_ = RIGHT_TURN_MIN_YAW;
      } else {
        // Do something interesting.
        Serial.println("@@@ DEAD END @@@");
        state_ = STOP;
      }
    }
  }
  
  enum TYawDirection {
    L,
    R
  };
  
  static const bool kDEBUG_MIN_YAW = true;
  
  static void DoMinLeftYaw() {
    if (kDEBUG_MIN_YAW) {
    }
    
    if (AnyLineFound()) {
      Motor::Left(kTURN_SPEED, kTURN_SPEED);
    } else {
      Dump("+++ LEFT_TURN_MIN_YAW complete");
      state_ = LEFT_TURN;
    }
  }
  
  static void DoMinRightYaw() {
    if (kDEBUG_MIN_YAW) {
    }
    
    if (AnyLineFound()) {
      Motor::Right(kTURN_SPEED, kTURN_SPEED);
    } else {
      Dump("+++ RIGHT_TURN_MIN_YAW complete");
      state_ = RIGHT_TURN;
    }
  }
  
  static const LineSensor::TSensorArray& line_sensor_values_;
  static int a_speed_;
  static int b_speed_;
  static float heading_;
  static long lineEndOdo_;
  static long lineStartOdo_;
  static TState state_;
  static unsigned long start_time_;
  static Map* start_map_;
  static Map* current_map_;
};

long Strategy::preroll_start_odo_;
long Strategy::preroll_start_yaw_;
long Strategy::preroll_goal_odo_;
long Strategy::stopping_start_odo_;

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
long Strategy::last_stopping_odo_;
Map* Strategy::start_map_ = new Map(NULL, Map::C, Map::E, SensorStick::Heading(), false, false, true);
Map* Strategy::current_map_ = Strategy::start_map_;
float Strategy::turn_start_heading_;
int Strategy::found_line_start_count_;

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
  "DEAD_END__MIN_YAW",
  "SOLVED",
};
    
#endif

