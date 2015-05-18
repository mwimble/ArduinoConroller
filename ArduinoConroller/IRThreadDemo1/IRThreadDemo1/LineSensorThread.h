class LineSensorThread: public Thread {
  private:
    int blink;
    static const int NUM_SENSORS = 8;
    static const int TIMEOUT_USEC = 2500;
    static const int EMITTER_PIN = 39;
    
    QTRSensorsRC qtrrc = 
      QTRSensorsRC((unsigned char[]) {37, 35, 33, 31, 29, 27, 25, 23},
                   NUM_SENSORS,
                   TIMEOUT_USEC,
                   EMITTER_PIN);
                        
  public:
    unsigned int sensorValues[NUM_SENSORS];
    unsigned int position;
    unsigned long readCount;
    
    LineSensorThread();
    void run();
    
};

void LineSensorThread::run() {
  readCount++;
//  Serial.println("RUN");
  position = qtrrc.readLine(sensorValues);
//  Serial.print("LineSensorThread::run position: ");
//  Serial.print(position);
//  Serial.print(" -- ");
//  for (int i = 0; i < 8; i++) {
//    Serial.print(" ");
//    Serial.print(sensorValues[i]);
//  }
//  
//  Serial.println();
  runned(); 
}

LineSensorThread::LineSensorThread() {
  readCount = 0;
  Serial.println("Enter any character to start calibration");
  do {
    delay(500);
  } while (Serial.available() == 0);
  
  int anyByte = Serial.read();
  Serial.println("Start calibration");
  for (int i = 0; i < 200; i++) {
    qtrrc.calibrate();
  }
  
  Serial.println("End calibration");
  Serial.println("Calibration min values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("] ");
    Serial.println(qtrrc.calibratedMinimumOn[i]);
  }
  
  Serial.println("Calibration max values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("] ");
    Serial.println(qtrrc.calibratedMaximumOn[i]);
  }
  
  Serial.println("LineSensorThread end");
}
