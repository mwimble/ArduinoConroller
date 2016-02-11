#ifndef __KAIMI_BASE_H
#define __KAIMI_BASE_H

#include <ros.h>
#include <Adafruit_MCP4725.h>
#include <DueTimer.h>
#include <geometry_msgs/Twist.h>
#include <Thread.h>
#include <ThreadController.h>

extern ros::NodeHandle nh;
extern ThreadController masterThreadController;

class KaimiBase : Thread {
  private:

    static Adafruit_MCP4725 dacFB;
    static Adafruit_MCP4725 dacLR;

    // Queue for commands.
    static const unsigned int QUEUE_LENGTH = 16;
    static geometry_msgs::Twist _commandQueue[QUEUE_LENGTH];
    static unsigned long readX;
    static unsigned long writeX;

    static void push(geometry_msgs::Twist msg) {
      _commandQueue[writeX++ % QUEUE_LENGTH] = msg;
    }

    static geometry_msgs::Twist pop() {
      if (readX >= writeX) {
        Serial.print("!!! KaimiBase::bad pull attempt to pull, readX: ");
        Serial.print(readX);
        Serial.print(", writeX: ");
        Serial.println(writeX);
        return geometry_msgs::Twist();
      }

      return _commandQueue[readX++ % QUEUE_LENGTH];
    }

    static bool isEmpty() {
      return readX >= writeX;
    }

    static unsigned long count() {
      return writeX - readX;
    }

    // Singleton instance.
    static KaimiBase* _singleton;

    static float _WHEEL_SEPARATION;
    static float _WHILE_RADIUS;

    static float _WHEEL_SEPARATION_MULTIPLIER;
    static float _WHEEL_RADIUS_MULTIPLIER;
    static float _NEUTRAL_BIAS;

    static DueTimer commandTimer;
    static bool commandIsExwcuting;
    static unsigned long countCommandTimeouts;

    static unsigned long startMillis;

    static const unsigned int STOP_VALUE = 2034;

    KaimiBase() { // Only singleton constructor.
      Serial.println("KaimiBase constructor start");
      dacFB.begin(0x62);
      dacLR.begin(0x63);
      dacFB.setVoltage(STOP_VALUE, true);
      dacLR.setVoltage(STOP_VALUE, true);
    }

    KaimiBase(KaimiBase const&) {}; // Cannot be copied.
    KaimiBase& operator=(KaimiBase const&) {}; // Cannot be assigned.

    static void commandTimerHandler() {
      countCommandTimeouts++;

      stop();
    }

    static void doWork() {
      //commandTimer.stop();

      if (queueIsEmpty()) return;

      geometry_msgs::Twist command = pop();

      Serial.print("KaimiBase::doWork x: ");
      Serial.print(command.linear.x);
      Serial.print(", z: ");
      Serial.println(command.angular.z);
      

      float ws = _WHEEL_SEPARATION * _WHEEL_SEPARATION_MULTIPLIER;
      float wr = _WHILE_RADIUS * _WHEEL_RADIUS_MULTIPLIER;

      bool truncatedX = false;
      bool truncatedZ = false;
      if (command.linear.x > 1.0) {
        truncatedX = true;
        command.linear.x = 1.0;
      } else if (command.linear.x < -1.0) {
        truncatedX = true;
        command.linear.x = -1.0;
      }

      if (command.angular.z > 1.0) {
        truncatedZ = true;
        command.angular.z = 1.0;
      } else if (command.angular.z < -1.0) {
        truncatedZ = true;
        command.angular.z = -1.0;
      }

      // Note, DAC ranges should be in:
      // Full backwards/left = 0.96v => 806 value
      // Full forwards/right => 3.96v => 3192

      float lr =  ((-command.angular.z * 1228) + 2034);
      float fb = ((command.linear.x * 1228) + 2034);
      char status[256];
      sprintf(status,
              "[ % 7.3f] KaimiBase::doWork, command.linear.x: %f%s, command.angular.z: %f%s, lr: %f, fb: %f",
              (millis() - startMillis) / 1000.0,
              command.linear.x,
              truncatedX ? "*T*" : "",
              command.angular.z,
              truncatedZ ? "*T*" : "",
              lr,
              fb);
      Serial.println(status);
      nh.loginfo(status);
      dacFB.setVoltage(fb, false);
      dacLR.setVoltage(lr, false);

      // For 0 linear, and 1 angular, value = 1.73076923076923
      // For 1 linear and 0 angular, value = 6.05693519079346
      // To normalize, multiply z by 3.49956255468067
      // So velLeft/velRight will go span -6.05693519079346 .. 6.05693519079346
      //      float zm = 3.49956255468067;
      //      float velLeft = (command.linear.x - (command.angular.z * ws * zm / 2.0)) / wr;
      //      float velRight = (command.linear.x + (command.angular.z * ws * zm/ 2.0)) / wr;
      //      float velHalfRange = 1.0 / wr; // 6.05693519079346 swing from velocity from -1 to 0 or 0 to 1
      //
      //      float dacNeutral = 2034; // Value for speed = 0;
      //      float dacMin = 806; // Value for min. So max is dacNeutral + dacMin.
      //      float dacHalfRange = dacNeutral - dacMin; // 1228, swing from min to neutral or neutral to max
      //      float velToDacMultiplier = dacHalfRange / velHalfRange; // 202.74279999999995
      //      // velLeft or velRight * velToDacMultiplier will g from -1228 to +1228.
      //
      //      // Velocities should scale in -1.36 .. 1.36
      //      velLeft = (velLeft * velToDacMultiplier) + dacNeutral;
      //      velright = (velRight * velToDacMultiplier) + dacNeutral;
      //
      //      sprintf(status,
      //              "[ % 7.3f] KaimiBase::doWork, command.linear.x: %f%s, command.angular.z: %f%s, velLeft: %f, velright: %f",
      //              (millis() - startMillis) / 1000.0,
      //              command.linear.x,
      //              truncatedX ? "*T*" : "",
      //              command.angular.z,
      //              truncatedZ ? "*T*" : "",
      //              velLeft,
      //              velright);
      //      Serial.println(status);
      //      nh.loginfo(status);

      commandTimer.start(500000);
    }

    static bool queueIsEmpty() {
      bool result = true;
      result = isEmpty();
      return result;
    }

    static Thread baseThread;

  public:
    // Singleton instance.
    static KaimiBase* Singleton() {
      Serial.println("KaimiBase Singleton start");
      if (!_singleton) {
        _singleton = new KaimiBase();
        masterThreadController.add(&baseThread);
      }

      return _singleton;
    }

    static void enqueue(geometry_msgs::Twist command) {
      push(command);
    }

    bool shouldRun(long time = -1) {
      // Note we are ignoring any interval here.
      return !queueIsEmpty();
    }

    static int queueLength() {
      return count();
    }

    static void stop() {
      Serial.print("[");
      Serial.print((millis() - startMillis) / 1000.0);
      Serial.print("] KaimiBase::stop value: ");
      Serial.println(STOP_VALUE);
      dacFB.setVoltage(STOP_VALUE, false);
      dacLR.setVoltage(STOP_VALUE, false);
      commandTimer.stop();
      commandIsExwcuting = false;
    }

};

#endif

