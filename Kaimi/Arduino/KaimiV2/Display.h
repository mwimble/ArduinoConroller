#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <Thread.h>
#include <ThreadController.h>
#include <UTFT.h>

#include "KaimiBase.h"

extern ThreadController masterThreadController;

// Declare which fonts we will be using
extern uint8_t SmallFont[];       // 8 x 12 pixels.
extern uint8_t BigFont[];         // 16 x 16 pixels.
extern uint8_t SevenSegNumFont[]; // 32 x 50 pixels.


class Display : Thread {
  private:

    // Singleton instance.
    static Display* _singleton;

    Display() { // Only singleton constructor.
    }

    Display(Display const&) {}; // Cannot be copied.
    Display& operator=(Display const&) {}; // Cannot be assigned.

    static Thread baseThread;
    static UTFT myGLCD; // The display.
    static unsigned long startMillis;  // Milliseconds at program start.

    static const char* ON_FOR;
    static const char* PI1_STATE;
    static const char* TDK1_STATE;
    static const char* TIME;
    static const char* LISTHDR;
    static const char* PI1MSG;
    static const char* TDK1MSG;

    static const unsigned long MILLIS_PER_DAY = 1000 * 60 * 60 * 24;
    static const unsigned long MILLIS_PER_HOUR = 1000 * 60 * 60;
    static const unsigned long MILLIS_PER_MINUTE = 1000 * 60;
    static const unsigned long MILLIS_PER_SECOND = 1000;

    static const unsigned int WHITE_COLOR = 255;
    static const unsigned int BLACK_COLOR = 0;

    static unsigned int POS_LIST_Y;
    static unsigned int POS_PI1_MSG_Y;
    static unsigned int POS_TDK1_MSG_Y;

    typedef struct TDisplayCommand {
      unsigned int sequence;
      unsigned long time; // Milliseconds since start.
      float x;
      float z;
    };

    static KaimiBase* robotBase;

    static TDisplayCommand commands[4];
    static unsigned long sequenceNumber;

    static void showOnTime(unsigned long onDuration) {
      char s[128];
      unsigned int days = onDuration / MILLIS_PER_DAY;
      onDuration = onDuration % MILLIS_PER_DAY;
      unsigned int hours = onDuration / MILLIS_PER_HOUR;
      onDuration = onDuration % MILLIS_PER_HOUR;
      unsigned int minutes = onDuration / MILLIS_PER_MINUTE;
      onDuration = onDuration % MILLIS_PER_MINUTE;
      unsigned int seconds = onDuration / MILLIS_PER_SECOND;
      unsigned int millis = onDuration % MILLIS_PER_SECOND;

      sprintf(s, (const prog_char*) F("%1d %02d:%02d:0%2d:%03d"), days, hours, minutes, seconds, millis);
      myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
      myGLCD.setBackColor(WHITE_COLOR, WHITE_COLOR, WHITE_COLOR);
      myGLCD.print(s, (strstr(ON_FOR, "0d") - ON_FOR) * 16, 0, 0);
    }

    static void showCommands() {
      char s[128];
      int queueLength = robotBase->queueLength();

      sprintf(s, (const prog_char*) F("%2d"), queueLength);
      myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
      myGLCD.setBackColor(WHITE_COLOR, WHITE_COLOR, WHITE_COLOR);
      myGLCD.print(s, (strstr(TIME, "qq") - TIME) * 16, POS_LIST_Y - 34, 0);

      for (int i = 0; i < 4; i++) {
        //seq=27  24,25,26,23
        int relIndex = (sequenceNumber + i) % 4;
        TDisplayCommand c = commands[relIndex];
        unsigned long time = c.time;
        unsigned int days = time / MILLIS_PER_DAY;
        time = time % MILLIS_PER_DAY;
        unsigned int hours = time / MILLIS_PER_HOUR;
        time = time % MILLIS_PER_HOUR;
        unsigned int minutes = time / MILLIS_PER_MINUTE;
        time = time % MILLIS_PER_MINUTE;
        unsigned int seconds = time / MILLIS_PER_SECOND;
        unsigned int millis = time % MILLIS_PER_SECOND;

        sprintf(s,
                (const prog_char*) F("%7d %02d:%02d:%02d:%03d %5.3f %5.3f"),
                c.sequence,
                hours,
                minutes,
                seconds,
                millis,
                c.x,
                c.z),
                myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
        myGLCD.setBackColor(255, 228, 225); // MistyRose
        myGLCD.print(s, 0, POS_LIST_Y + (i * 17), 0);
      }
    }

    static void doWork() {
      unsigned long currentMillis = millis();
      unsigned long onDuration = currentMillis - startMillis;
      showOnTime(onDuration);
      showCommands();
    }

  public:

    // Singleton instance.
    static Display* Singleton() {
      if (!_singleton) {
        _singleton = new Display();

        robotBase = KaimiBase::Singleton();

        //#####
        commands[0] = {1, 112, 1.2, 5.6};
        commands[1] = {2, 1003, 2.3, 6.7};
        commands[2] = {3, 11227, 3.4, 7.8};
        commands[3] = {4, 15219, 4.5, 8.9};

        // Setup the LCD
        myGLCD.InitLCD();
        myGLCD.clrScr();

        // Build frame for on time display.
        myGLCD.setColor(WHITE_COLOR, WHITE_COLOR, WHITE_COLOR);
        myGLCD.fillRect(0, 0, 799, 17 * 4);

        unsigned int currY = 0;
        myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
        myGLCD.setBackColor(WHITE_COLOR, WHITE_COLOR, WHITE_COLOR);
        myGLCD.setFont(BigFont);
        myGLCD.print(ON_FOR, 0, currY, 0);
        currY += 17;
        myGLCD.print(PI1_STATE, 0, currY, 0);
        currY += 17;
        myGLCD.print(TDK1_STATE, 0, currY, 0);
        currY += 17;
        myGLCD.print(TIME, 0, currY, 0);
        currY += 17;

        myGLCD.setColor(255, 228, 225); // MistyRose
        myGLCD.fillRect(0, currY, 799, currY + (17 * 5));
        POS_LIST_Y = currY + 17;
        myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
        myGLCD.setBackColor(255, 228, 225); // MistyRose
        myGLCD.print(LISTHDR, 0, currY, 0);
        currY += 17 * 5;

        myGLCD.setColor(224, 255, 255); // Lavender
        myGLCD.fillRect(0, currY, 799, currY + (17 * 2));

        POS_PI1_MSG_Y = currY;
        myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
        myGLCD.setBackColor(224, 255, 255); // Lavender
        myGLCD.print(PI1MSG, 0, currY, 0);
        currY += 17;

        POS_TDK1_MSG_Y = currY;
        myGLCD.setColor(BLACK_COLOR, BLACK_COLOR, BLACK_COLOR);
        myGLCD.setBackColor(224, 255, 255); // Lavender
        myGLCD.print(TDK1MSG, 0, currY, 0);
        currY += 17;

        baseThread.setInterval(500); // 500 ms.
        masterThreadController.add(&baseThread);
      }

      return _singleton;
    }

    bool shouldRun(long time = -1) {
      // Note we are ignoring any interval here.
      return true;
    }

    void enqueue(double x, double z) {
      TDisplayCommand* d = &commands[sequenceNumber % 4];
      d->sequence = sequenceNumber++;
      d->time = millis();
      d->x = x;
      d->z = z;
    }

};

#endif
