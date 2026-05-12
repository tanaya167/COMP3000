#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "MPU6050.h"

/* ================= CONFIG ================= */

#define SAMPLE_PERIOD_MS 40
#define ENROLL_MS 8000
#define VERIFY_MS 5000

#define TEMP_MIN_C 18.0
#define TEMP_MAX_C 45.0

#define MIN_PPG_PEAKS 6

/* ================= SENSORS ================= */

MAX30105 maxSensor;
MPU6050 mpu;

/* ================= STRUCTS ================= */

struct PpgFeatures {
  float ac;
  float meanIR;
  float normAC;
  int peaks;
};

static PpgFeatures measurePPG(uint32_t ms);


struct Sample {
  uint32_t t;
  bool contact;
  bool pulsePresent;
  long ir;
  float tempC;
  bool tempHuman;
  float ax_g, ay_g, az_g;
  float aMag;
  bool moving;
  bool worn;
};

struct Profile {
  bool valid = false;
  float ppgAC;
};

/* ================= GLOBALS ================= */

Profile userProfile;

/* ================= HELPERS ================= */

static float readTempC() {
  return 22.0 + random(-20, 20) / 10.0;
}

static Sample readSample() {
  Sample s{};
  s.t = millis();

  s.ir = maxSensor.getIR();
  s.contact = (s.ir > 5000);

  static long lastIR = 0;
  s.pulsePresent = abs(s.ir - lastIR) > 100;
  lastIR = s.ir;

  s.tempC = readTempC();
  s.tempHuman = (s.tempC >= TEMP_MIN_C && s.tempC <= TEMP_MAX_C);

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  s.ax_g = ax / 16384.0f;
  s.ay_g = ay / 16384.0f;
  s.az_g = az / 16384.0f;
  s.aMag = sqrtf(s.ax_g*s.ax_g + s.ay_g*s.ay_g + s.az_g*s.az_g);
  s.moving = fabsf(s.aMag - 1.0f) > 0.15f;

  s.worn = (s.contact && s.pulsePresent);
  return s;
}

/* ================= WORN CHECK ================= */

static bool waitProperlyWorn(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    Sample s = readSample();

    Serial.print("#WORNCHK contact=");
    Serial.print(s.contact);
    Serial.print(" pulse=");
    Serial.print(s.pulsePresent);
    Serial.print(" ir=");
    Serial.print(s.ir);
    Serial.print(" temp=");
    Serial.print(s.tempC, 2);
    Serial.print(" tempHuman=");
    Serial.print(s.tempHuman);
    Serial.print(" worn=");
    Serial.println(s.worn);

    if (s.worn) return true;
    delay(300);
  }
  return false;
}

/* ================= PPG FEATURE ================= */



static PpgFeatures measurePPG(uint32_t ms) {
  const int BUF = 400;
  long irBuf[BUF];
  int n = 0;

  uint32_t start = millis();
  while (millis() - start < ms && n < BUF) {
    long ir = maxSensor.getIR();

    if (ir > 10000 && ir < 120000) {  
      irBuf[n++] = ir;
    }

    delay(20);
  }

  PpgFeatures f{};
  if (n < 100) return f;

  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += irBuf[i];
  }

  f.meanIR = (float)sum / n;

  
  float acSum = 0;
  int acCount = 0;

  for (int i = 1; i < n - 1; i++) {
    float prev = irBuf[i-1];
    float curr = irBuf[i];
    float next = irBuf[i+1];

    
    if (curr > prev && curr > next && curr > f.meanIR) {
      float trough = prev;
      acSum += (curr - trough);
      acCount++;
      f.peaks++;
    }
  }

  if (acCount > 0)
    f.ac = acSum / acCount;
  else
    f.ac = 0;

  f.normAC = f.ac / f.meanIR;

  Serial.print("#PPG mean=");
  Serial.print(f.meanIR);
  Serial.print(" avgAC=");
  Serial.print(f.ac);
  Serial.print(" normAC=");
  Serial.print(f.normAC, 5);
  Serial.print(" peaks=");
  Serial.println(f.peaks);

  return f;
}

/* ================= ENROLL ================= */

static bool enrolUser(Profile &outP) {
  Serial.println("#ENROLL Put it on properly (touch + pulse). Starting in ~5s...");
  delay(5000);

  if (!waitProperlyWorn(5000)) {
    Serial.println("#ENROLL Failed: not properly worn.");
    return false;
  }

  Serial.println("#ENROLL PPG: measuring...");
  PpgFeatures ppg = measurePPG(8000);

  if (ppg.peaks < MIN_PPG_PEAKS || ppg.ac < 200) {
    Serial.println("#ENROLL Failed: PPG quality too low.");
    return false;
  }

  outP.valid = true;
  outP.ppgAC = ppg.ac;

  Serial.println("#ENROLL OK");
  return true;
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!maxSensor.begin(Wire)) {
    Serial.println("MAX3010x not found!");
    while (1);
  }

  maxSensor.setup(0x1F, 4, 2, 100, 411, 4096);
  maxSensor.setPulseAmplitudeRed(0x24);
  maxSensor.setPulseAmplitudeIR(0x24);

  mpu.initialize();

  Serial.println("Ready. Press E to enrol.");
}

/* ================= LOOP ================= */

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'E' || c == 'e') {
      enrolUser(userProfile);
    }
  }
}