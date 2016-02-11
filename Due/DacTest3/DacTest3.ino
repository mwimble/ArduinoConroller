#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dacFB;
Adafruit_MCP4725 dacLR;

int analogInputPin = A0;

void setup() {
  Serial.begin(115200);
  delay(3000);
  while (!Serial) {
    delay(1);
  }

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
  delay(1000);
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println("Enter F/B, comma, L/R values:");
  Serial.flush();
  while (!Serial.available()) {
  }

  while ((Serial.peek() <= 32) && (Serial.peek() >= 0)) {
    int foo = Serial.read();
    Serial.print("Ignoring: ");
    Serial.println(foo, HEX);
  }

  Serial.print("Next char: ");
  Serial.println(Serial.peek(), HEX);

  if (Serial.available()) {
    int fb = Serial.parseInt();
    int lr = Serial.parseInt();
    doValue(fb, lr);
  }
}
