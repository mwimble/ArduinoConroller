#include <Wire.h>

#define SLAVE_ADDRESS 0x0C
int number = 0;
int state = 0;

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(9600); // start serial for output
    
    Serial.println("setUp");
    
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);
    
    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
    
    Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount){
    //Serial.print("reaceiveData byteCount: ");
    //Serial.println(byteCount);
    //int regNum = Wire.read(); // Ignore register.
    while(Wire.available()) {
        number = Wire.read();
        Serial.print("data received: ");
        Serial.println(number);
        if (number == 1){
            if (state == 0){
              digitalWrite(13, HIGH); // set the LED on
              Serial.println("Setting LED on");
              state = 1;
            } else {
              digitalWrite(13, LOW); // set the LED off
              Serial.println("Setting LED off");
              state = 0;
            }
        }
    }
}

// callback for sending data
char s[] = "response=x\n";
int nextChar = 0;
void sendData(){
  //Serial.print("Sending: ");
  
  s[sizeof(s) - 3] = '0' + number + 1;
  //Serial.println(s[nextChar]);
  Wire.write(s[nextChar++]);
  
  if (nextChar >= sizeof(s)) {
    nextChar = 0;
  }
}


