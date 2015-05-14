#include <Wire.h>

void setup()
{
  Wire.begin(13);                // join i2c bus with address #13
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()

int x = 0;

void requestEvent()
{
  Wire.write(x++); // respond with message of 6 bytes
                       // as expected by master
}
