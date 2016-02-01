#ifndef __KAIMI_BASE_H
#define __KAIMI_BASE_H

#include <ros.h>
#include <DueTimer.h>
#include <geometry_msgs/Twist.h>
#include <Thread.h>
#include <ThreadController.h>

extern ros::NodeHandle nh;
extern ThreadController masterThreadController;

class KaimiBase : Thread {
  private:

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

    static DueTimer commandTimer;
    static bool commandIsExwcuting;
    static unsigned long countCommandTimeouts;

    static unsigned long startMillis;

    static unsigned long xxx;
    static float lastY;
    static float lastZ;

    KaimiBase() { // Only singleton constructor.
    }

    KaimiBase(KaimiBase const&) {}; // Cannot be copied.
    KaimiBase& operator=(KaimiBase const&) {}; // Cannot be assigned.

    static void commandTimerHandler() {
      countCommandTimeouts++;
      float yyy = (millis() - xxx) / 1000.0;

//      Serial.print("Timeout, lastY: ");
//      Serial.print(lastY);
//      Serial.print(", lastZ: ");
//      Serial.println(lastZ);

      stop();
    }

    static void doWork() {
      //commandTimer.stop();

      if (queueIsEmpty()) return;

      geometry_msgs::Twist command = pop();

      lastY = command.linear.y;
      lastZ = command.angular.z;

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

      //      float velLeft = (command.linear.x - (command.angular.z * ws / 2.0)) / wr;
      //      float velRight = (command.linear.x + (command.angular.z * ws / 2.0)) / wr;

      float dacFwdRev = (command.linear.x + 1) * 2048.0;
      float dacLeftRight = (command.angular.z + 1) * 2048.0;

      //      sprintf(status,
      //              "[ % 7.3f] KaimiBase::doWork, fwd / rev: % f % s, l / r: % f % s",
      //              (millis() - startMillis) / 1000.0,
      //              dacFwdRev,
      //              truncatedX ? "*T*" : "",
      //              dacLeftRight,
      //              truncatedZ ? "*T*" : "");
      //      nh.loginfo(status);

      xxx = millis();//#####
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
      commandTimer.stop();
      commandIsExwcuting = false;
    }

};

#endif

