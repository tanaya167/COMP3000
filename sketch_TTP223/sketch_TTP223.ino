int touchPin = 4;

void setup() {
  Serial.begin(115200);
  pinMode(touchPin, INPUT);
}

void loop() {
  int val = digitalRead(touchPin);
  Serial.println(val ? "TOUCHED" : "NOT TOUCHED");
  delay(100);
}


