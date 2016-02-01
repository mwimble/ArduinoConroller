int analogInputPin = A0;

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit 
}

int analogInputValue = 0;
int analogWriteValue = 0;
int direction = 0;

void doValue(int outValue) {
  analogWrite(DAC0, outValue);
  int analogInputValue = analogRead(analogInputPin);
  Serial.print("Write value: ");
  Serial.print(outValue);
  Serial.print(", read value: " );
  Serial.print(analogInputValue);
  Serial.print(" -- ");
  Serial.println(analogInputValue * (5.0 / 4096.0)); 

  char buffer[10];
  Serial.println("Entery any key to continue");
  while(Serial.available() == 0) {};
  Serial.readBytesUntil('\n', buffer, 10);
}

void loop() {
  // put your main code here, to run repeatedly:
//  analogWrite(DAC0, analogWriteValue);
//  analogInputValue = analogRead(analogInputPin);
//  Serial.print("Write value: ");
//  Serial.print(analogWriteValue);
//  Serial.print(", read value: " );
//  Serial.print(analogInputValue);
//  Serial.print(" -- ");
//  Serial.println(analogInputValue * (3.5 / 4096.0));
//
//  if (direction == 0) {
//    analogWriteValue += 2;
//  } else {
//    analogWriteValue -= 2;
//  }
//
//  if (analogWriteValue > 4095) {
//    direction = 1;
//    analogWriteValue = 4095;
//  } else if (analogWriteValue < 0) {
//    direction = 0;
//    analogWriteValue = 0;
//  }
//
//  delay(1);
  doValue(0);
  doValue(1023);
  doValue(2047);
  doValue(4095);
}
