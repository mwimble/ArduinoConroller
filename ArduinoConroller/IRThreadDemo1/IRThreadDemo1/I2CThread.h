#ifndef _I2CThread_h
#define _I2CThread_h

class I2CThread: public Thread {
  private:
  static const int SLAVE_ADDRESS = 0x0C;

  static int nextChar; //#####
  static int number; //#####
  static char s[12]; //#####
  static int state; //#####
 
  static void receiveData(int byteCount);
  static void sendData();
  
  public:
  I2CThread();
  void run();
};

I2CThread::I2CThread() {
    strcpy("response=x\n", I2CThread::s);
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);
    
    // define callbacks for i2c communication
    Wire.onReceive(I2CThread::receiveData);
    Wire.onRequest(I2CThread::sendData);  
}

void I2CThread::run() {
  
}

// Callback for received data
void I2CThread::receiveData(int byteCount) {
    //Serial.print("reaceiveData byteCount: ");
    //Serial.println(byteCount);
    //int regNum = Wire.read(); // Ignore register.
    while(Wire.available()) {
        number = Wire.read();
        Serial.print("data received: ");
        Serial.println(number);
        nextChar = 0;
        if (number == 'x'){
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

// Callback for sending data
void I2CThread::sendData() {
  //Serial.print("Sending: ");
  
  s[sizeof(s) - 3] = number + 1;
  //Serial.println(s[nextChar]);
  Wire.write(s[nextChar++]);
  
  if (nextChar >= sizeof(s)) {
    nextChar = 0;
  }
}

#endif
