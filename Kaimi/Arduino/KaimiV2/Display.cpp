#include "Display.h"
#include <UTFT.h>

const char* Display::ON_FOR = (const prog_char*)        F("On for: 0d hh:mm:ss.mmm | Main bat xx.x V, xx.x A");
const char* Display::PI1_STATE = (const prog_char*)     F("PI1  state: ???? hb: aaaaaa, secsDue: bb");
const char* Display::TDK1_STATE = (const prog_char*)    F("TDK1 state: ???? hb: aaaaaa, secsDue: bb");
const char* Display::TIME = (const prog_char*)          F("Current time: xx:xx:xx  || Queue length: qq");
const char* Display::LISTHDR = (const prog_char*)       F("    Seq   Time        X     Z");
//xxxxxxx | xx:xx:xx.xxx | x.xxx | x.xxx
const char* Display::PI1MSG = (const prog_char*)        F(" PI1 Message: xxxxxx");
const char* Display::TDK1MSG = (const prog_char*)       F("TDK1 Message: xxxxxx");

Display* Display::_singleton;
Thread Display::baseThread = Thread(&Display::doWork, 10);
UTFT Display::myGLCD(CTE70, 25, 26, 27, 28);
unsigned long Display::startMillis = millis();
unsigned int Display::POS_LIST_Y = 0;
unsigned int Display::POS_PI1_MSG_Y = 0;
unsigned int Display::POS_TDK1_MSG_Y = 0;
Display::TDisplayCommand Display::commands[4];
unsigned long Display::sequenceNumber = 0;

KaimiBase* Display::robotBase;

