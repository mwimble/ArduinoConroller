#ifndef __SENSOR_STICK_H
#define __SENSOR_STICK_H

#include <Wire.h>

#include "I2Cdev.h"
#include "HMC5883L.h"

extern Log logger;

class SensorStick {
  public:
  SensorStick() {
    Wire.begin();
  }
  
  static void initialize() {
    if (!initialized_) {
      mag_.initialize();
      if (!mag_.testConnection()) {
        logger.println("Unable to communicate with sensor stick");
        return;
      }
      
      initialized_ = true;
    }
  }
  
  static void Process() {
    initialize();
    mag_.getHeading(&mx_, &my_, &mz_);
    heading_ = atan2(my_, mx_);
    if(heading_ < 0) {
      heading_ += 2 * M_PI;
    }
    
    heading_ = heading_ * 180.0 / M_PI;
  }
  
  static float Heading() {
    return heading_;
  }
  
  private:
  static float heading_;
  static bool initialized_;
  static HMC5883L mag_;
  static int16_t mx_;
  static int16_t my_;
  static int16_t mz_;
};

float SensorStick::heading_;
bool SensorStick::initialized_ = false;
HMC5883L SensorStick::mag_;
int16_t SensorStick::mx_;
int16_t SensorStick::my_;
int16_t SensorStick::mz_;

#endif

