#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu(0x68, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(21, 22);
  delay(100);

  Serial.println("Initializing MPU6050 (explicit constructor)...");
  mpu.initialize();
  delay(50);

  bool ok = mpu.testConnection();
  Serial.print("MPU6050 testConnection(): ");
  Serial.println(ok ? "OK" : "FAIL");

  if (!ok) {
    Serial.println("If FAIL, try tying AD0 to GND (address 0x68) or to VCC (0x69),");
    Serial.println("or run the low-level WHO_AM_I test I gave earlier.");
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel X="); Serial.print(ax);
  Serial.print(" Y="); Serial.print(ay);
  Serial.print(" Z="); Serial.println(az);

  Serial.print("Gyro  X="); Serial.print(gx);
  Serial.print(" Y="); Serial.print(gy);
  Serial.print(" Z="); Serial.println(gz);

  Serial.println("--------------------");
  delay(300);
}



