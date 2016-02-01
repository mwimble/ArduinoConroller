/*
  Mega multple serial test

 Receives from the main serial port, sends to the others.
 Receives from serial port 1, sends to the main serial (Serial 0).

 This example works only on the Arduino Mega

 The circuit:
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:

 created 30 Dec. 2008
 modified 20 May 2012
 by Tom Igoe & Jed Roach

 This example code is in the public domain.

 */


void setup() {
  // initialize both serial ports:
  Serial3.begin(57600);
  Serial.begin(9600);
}

int i;
void loop() {
  if (Serial3.available()) {
    int inByte = Serial3.read();
    Serial3.write(inByte);
    Serial.write(inByte);
    Serial3.println("<<");
  }

//Serial3.print("Hello world");
//Serial3.println(i++);
//Serial.print("Hello world");
//Serial.println(i++);
}
