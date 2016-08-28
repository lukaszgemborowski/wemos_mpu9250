#include <Wire.h>

void setup()
{
  Serial.begin(115200);
  Serial.println("Startup sequence...");
  delay(100);
  Serial.println("Starting I2C port...");
  Wire.begin();
}

void loop()
{
}


