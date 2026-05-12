#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); 

  Serial.println("Initializing MPU-6050...");
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find MPU-6050. Check wiring!");
    while (1);
  }

  Serial.println("MPU-6050 ready!");
  delay(1000);
}

void loop() {
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();

  Serial.println("Raw Accel: X=" + String(rawAccel.XAxis) +
                 " Y=" + String(rawAccel.YAxis) +
                 " Z=" + String(rawAccel.ZAxis));
  Serial.println("Normalized Accel: X=" + String(normAccel.XAxis) +
                 " Y=" + String(normAccel.YAxis) +
                 " Z=" + String(normAccel.ZAxis));

  Serial.println("Raw Gyro: X=" + String(rawGyro.XAxis) +
                 " Y=" + String(rawGyro.YAxis) +
                 " Z=" + String(rawGyro.ZAxis));
  Serial.println("Normalized Gyro: X=" + String(normGyro.XAxis) +
                 " Y=" + String(normGyro.YAxis) +
                 " Z=" + String(normGyro.ZAxis));

  Serial.println("------------------------");
  delay(500);
}

