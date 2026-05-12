#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); 

  Serial.println("Initializing MLX90614...");

  if (!mlx.begin()) {
    Serial.println("MLX90614 not found!");
    while (1);
  }

  Serial.println("MLX90614 initialized successfully!");
}

void loop() {
  Serial.print("Ambient = ");
  Serial.print(mlx.readAmbientTempC());
  Serial.print(" *C\tObject = ");
  Serial.print(mlx.readObjectTempC());
  Serial.println(" *C");
  delay(500);
}






