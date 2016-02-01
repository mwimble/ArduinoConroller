#include "KaimiBase.h"

geometry_msgs::Twist KaimiBase::_commandQueue[QUEUE_LENGTH];
KaimiBase* KaimiBase::_singleton;
Thread KaimiBase::baseThread = Thread(&KaimiBase::doWork, 10);

float KaimiBase::_WHEEL_SEPARATION = 0.5715; // 22.5"
float KaimiBase::_WHILE_RADIUS = 0.1651; // 6.5"
float KaimiBase::_WHEEL_SEPARATION_MULTIPLIER = 1.0;
float KaimiBase::_WHEEL_RADIUS_MULTIPLIER = 1.0;

DueTimer KaimiBase::commandTimer = DueTimer::getAvailable().attachInterrupt(commandTimerHandler).start(500000); // Every 500ms
bool KaimiBase::commandIsExwcuting = false;
unsigned long KaimiBase::countCommandTimeouts = 0;

unsigned long  KaimiBase::startMillis = millis();
unsigned long KaimiBase::xxx = 0;
float KaimiBase::lastY = 0.0;
float KaimiBase::lastZ = 0.0;

unsigned long  KaimiBase::readX = 0;
unsigned long  KaimiBase::writeX = 0;

