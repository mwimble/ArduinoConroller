#include "LineSensorThread.h"
#include <QTRSensors.h>
#include "QuadratureEncoderThread.h"

class Strategy {
  private:
    static const int SENSOR_COLS = 8;
    
    LineSensorThread* lineSensorThread;
    QuadratureEncoderThread* quadratureEncoderThread;

    void displayState();

    int lastValue = 0;
    unsigned long average = 0;
    unsigned int sum = 0;
    int findLineDirection();

    int maxConsecutiveSensors = 0;
    int firstConsecutiveSensor = 0;
    bool atCrossing();

    int colSensor = 0;
    bool onLine();

    enum StrategyState {
      SeekLine,
      ContinueForward,
      VeerRight,
      VeerLeft,
      StoppedAtCrossing,
      StoppedAtLineEnd
    };
    
    const char* stateNames[6] = {
      "SeekLine",
      "ContinueForward",
      "VeerRight",
      "VeerLeft",
      "StoppedAtCrossing",
      "StoppedAtLineEnd"
    };

    StrategyState strategyState = SeekLine;

    void StopSystem();

  public:
    Strategy(LineSensorThread* lineSensorThread, QuadratureEncoderThread* quadratureEncoderThread);

    void doStrategy();

};

Strategy::Strategy(LineSensorThread* inLineSensorThread, QuadratureEncoderThread* inQuadratureEncoderThread) {
  lineSensorThread = inLineSensorThread;
  quadratureEncoderThread = inQuadratureEncoderThread;
}

void Strategy::displayState() {
  Serial.print("STRAGETY state: ");
  Serial.print(stateNames[strategyState]);
  Serial.print("linePosition: ");
  Serial.print(lastValue);
  Serial.print(", COLS left>>");
  for (int col = 0; col < SENSOR_COLS; col++) {
    int lrCol = (SENSOR_COLS - col) - 1;
    Serial.print(" ");
    Serial.print(lineSensorThread->sensorValues[col]);
  }

  Serial.print(" <<right, odo: ");
  Serial.println(quadratureEncoderThread->counter());
}

int Strategy::findLineDirection() {
  bool  foundLine = false;
  average = 0;
  sum = 0;
  for (int col = 0; col < SENSOR_COLS; col++) {
    unsigned int value = lineSensorThread->sensorValues[(SENSOR_COLS - col) - 1];
    if (value > 200) {
      foundLine = true;
    }

    if (value > 50) {
      average += (long)(value) * (col * 1000);
      sum += value;
    }
  }

  if (!foundLine) {
    if (lastValue < ((SENSOR_COLS * 1000) / 2)) {
      return 0;
    } else {
      return (SENSOR_COLS - 1) * 1000;
    }
  }

  lastValue = average / sum;
  if ((lastValue < 0) || (lastValue > 7000)) {
    Serial.print("lastValue out of range: ");
    Serial.println(lastValue);
    displayState();
  }
  
  return lastValue;
}

bool Strategy::atCrossing() {
  maxConsecutiveSensors = 0;
  firstConsecutiveSensor = 0;
  for (int col = 0; col < (SENSOR_COLS - 1); col++) {
    if (lineSensorThread->sensorValues[col] > 200) {
      int consecutiveSensors = 1;
      int nextCol = col + 1;
      while ((nextCol < SENSOR_COLS) && (lineSensorThread->sensorValues[nextCol] > 200)) {
        consecutiveSensors++;
        nextCol++;
      }

      if (consecutiveSensors > maxConsecutiveSensors) {
        maxConsecutiveSensors = consecutiveSensors;
        firstConsecutiveSensor = col;
      }
    }
  }

  return maxConsecutiveSensors > 3;
}

bool Strategy::onLine() {
  for (colSensor = 0; colSensor < SENSOR_COLS; colSensor++) {
    unsigned int value = lineSensorThread->sensorValues[colSensor];
    if (value > 200) {
      return true;
    }
  }

  return false;
}

void Strategy::StopSystem() {
  stop();

  int lineDirection = findLineDirection();
  Serial.print("findLineDirection: ");
  Serial.print(lineDirection);
  Serial.print(", lastValue: ");
  Serial.print(lastValue);
  Serial.print(", average:: ");
  Serial.print(average);
  Serial.print(", sum: ");
  Serial.println(sum);

  Serial.print("atCrossing: ");
  Serial.print(atCrossing());
  Serial.print(", maxConsecutiveSensors: ");
  Serial.print(maxConsecutiveSensors);
  Serial.print(", firstConsecutiveSensor: ");
  Serial.println(firstConsecutiveSensor);

  Serial.print("onLine: ");
  Serial.print(onLine());
  Serial.print(", colSensor: ");
  Serial.println(colSensor);

  displayState();

  Serial.print("strategyState: ");
  Serial.print(strategyState);

  for (;;) {
    delay(500);
  }
}

void Strategy::doStrategy() {
  int lineDirection = findLineDirection();
  bool isOnLine = onLine();
  bool isAtCrossing = atCrossing();

  if (isAtCrossing) {
    stop();
    Serial.println("STOPPED AT CROSSING");
    strategyState = StoppedAtCrossing;
    StopSystem();
  }

  switch (strategyState) {
    case SeekLine:
      if (!isOnLine) {
        Serial.println("! on line, forward");
        forward();
      } else {
        stop();
        if (lineDirection < 3000) {
          Serial.print("newly online, left, lineDirection: "); Serial.println(lineDirection);
          left();
          strategyState = VeerLeft;
        } else if (lineDirection > 4000) {
          Serial.print("newly online, right, lineDirection: "); Serial.println(lineDirection);
          right();
          strategyState = VeerRight;
        } else {
          Serial.print("newly online, forward, lineDirection: "); Serial.println(lineDirection);
          forward();
          strategyState = ContinueForward;
        }
      }

      break;

    case ContinueForward:
    case VeerLeft:
    case VeerRight:
      if (!isOnLine) {
        Serial.println("was forward, lost line, stop");
        stop();
        strategyState = StoppedAtLineEnd;
        Serial.println("AT LINE END");
        StopSystem();
      } else if (lineDirection < 3000) {
        Serial.print("was forward, left, lineDirection: "); Serial.println(lineDirection);
        left();
        strategyState = VeerLeft;
      } else if (lineDirection > 4000) {
        Serial.print("was forward, right, lineDirection: "); Serial.println(lineDirection);
        right();
        strategyState = VeerRight;
      } else {
        Serial.print("was forward, continue forward, lineDirection: "); Serial.println(lineDirection);
        forward();
        strategyState = ContinueForward;
      }

      break;

    case StoppedAtCrossing:
    case StoppedAtLineEnd:
      stop();
      StopSystem();
      break;

    default:
      Serial.print("STRATEGY STATE ERROR: ");
      StopSystem();
  }
}
