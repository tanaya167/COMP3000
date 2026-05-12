#include <Wire.h>
#include "MAX30105.h"

MAX30105 sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);

  Serial.println("Initializing MAX3010x...");

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while (1);
  }

  sensor.setup();  
  Serial.println("MAX30102 Ready!");
}

void loop() {
  long irValue = sensor.getIR();
  Serial.print("IR: ");
  Serial.println(irValue);
  delay(50);
}

