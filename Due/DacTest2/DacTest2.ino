#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dacFB;
Adafruit_MCP4725 dacLR;

int analogInputPin = A0;

void setup() {
  Serial.begin(115200);
  dacFB.begin(0x62);
  dacLR.begin(0x63);
  dacFB.setVoltage(2034, true); // 2.517
  dacLR.setVoltage(2018, true); // 2.492
}

int analogInputValue = 0;
int analogWriteValue = 0;
int direction = 0;

void doValue(int fbValue, int lrValue) {
  dacFB.setVoltage(fbValue, false);
  dacLR.setVoltage(lrValue, false);

  Serial.print("Write FB value: ");
  Serial.print(fbValue);
  Serial.print(", LR value");
  Serial.println(lrValue);

  char buffer[10];
  Serial.println("Entery any key to continue");
  while(Serial.available() == 0) {};
  Serial.readBytesUntil('\n', buffer, 10);
}

void loop() {
  doValue(2034, 2018);
  doValue(806, 3192);
  doValue(774, 2018);
  doValue(2034, 774);
  doValue(3192, 806);
}
