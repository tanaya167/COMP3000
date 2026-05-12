#include <Wire.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include <Adafruit_MLX90614.h>
#include <math.h>
#include <Preferences.h>
#include "mbedtls/gcm.h"
#include "mbedtls/base64.h"
#include "esp_system.h"

// FORWARD DECLARATIONS 
struct Sample;
struct PpgFeatures;
struct Profile;

static Sample readSample();
static bool enrolUser(Profile &outP);
static bool verifyUser(const Profile& p);
static PpgFeatures computePpgFeatures(uint32_t durationMs);
static bool isWornNow();

static void saveProfile(const Profile& p);
static Profile loadProfile();
static void eraseProfile();

// PINS 
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const int TTP223_PIN = 4;

// SENSORS 
MPU6050 mpu(0x68, &Wire);
MAX30105 maxSensor;
Adafruit_MLX90614 mlx;

bool mlxOK = false;
bool maxOK = false;
bool mpuOK = false;

// TIMING 
static const uint32_t SAMPLE_PERIOD_MS = 200;
static const uint32_t ENROLL_MS = 15000;
static const uint32_t VERIFY_MS = 12000;

// MOTION / TEMP (UNCHANGED) 
static const float ACC_LSB_PER_G = 16384.0f;
static const long IR_PRESENT_TH = 2000;
static const float TEMP_MIN_C = 25.0;
static const float TEMP_MAX_C = 40.0;
static const float MOVE_MAG_TH = 0.25;
static const uint32_t UNWORN_GRACE_MS = 1500;

// PPG (FIXED ONLY HERE)
static const uint32_t PPG_SAMPLE_MS = 20;
static const uint32_t PPG_REFRACTORY_MS = 250;

static const float PPG_DC_ALPHA = 0.96f;
static const float PPG_NOISE_ALPHA = 0.94f;
static const float PPG_THR_MULT = 1.1f;
static const float PPG_MIN_THR = 20.0f;
static const int PPG_MIN_PEAKS = 2;

// GESTURE
static float lastAx = 0;
static float lastAy = 0;
static float lastAz = 0;

static uint32_t lastGestureMs = 0;

static const float SHAKE_THRESHOLD = 1.2f;
static const uint32_t GESTURE_COOLDOWN = 800;

// AES 
static const size_t AES_KEY_BYTES = 16;
static const size_t GCM_NONCE_BYTES = 12;
static const size_t GCM_TAG_BYTES = 16;

uint8_t AES_KEY[AES_KEY_BYTES] = {
  0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,
  0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81
};

mbedtls_gcm_context gcm;
Preferences prefs;

// STATE 
enum State { LOCKED, ENROLLING, AUTHENTICATED };
State state = LOCKED;

uint32_t seq = 0;
uint32_t lastWornMs = 0;


// GESTURE 
static const int GESTURE_LEN = 20;
static const float GESTURE_DEADBAND = 0.10f;
static const float TEMPLATE_REQUIRED = 0.30f;
static const float ENROL_REPEAT_REQUIRED = 0.50f;
static const float VERIFY_GESTURE_REQUIRED = 0.60f;

enum GestureType {
  GESTURE_SHAKE = 1,
  GESTURE_TILT = 2
};

struct Gesture {
  int8_t pattern[GESTURE_LEN];
};

static const int8_t SHAKE_TEMPLATE[GESTURE_LEN] = {
  1, 1, -1, -1, 1, 1, -1, 0, 1, 1,
  0, 0, 1, 1, 0, 0, 1, 1, 0, 0
};

static const int8_t TILT_TEMPLATE[GESTURE_LEN] = {
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 0, 0, 0, 0, 0, 0
};

struct Profile {
  bool valid;

  float irMean;
  float irStd;

  float aMagMean;
  float aMagStd;

  float tempMean;
  float tempStd;

  float bpmMean;
  float ibiMeanMs;
  float ibiStdMs;
  float acMean;

  Gesture gesture;
  int gestureType;   
};

Profile profile {};

// SAMPLE 
struct Sample {
  uint32_t t;
  bool contact;
  long ir;
  float tempC;
  float ax_g, ay_g, az_g;
  float aMag;
  bool pulsePresent;
  bool tempHuman;
  bool moving;
  bool worn;
};

// PPG FEATURES 
struct PpgFeatures {
  bool ok;
  int peaks;
  float bpm;
  float ibiMeanMs;
  float ibiStdMs;
  float acMean;
};

// SENSOR READ (TEMP UNCHANGED)
static Sample readSample() {
  Sample s{};
  s.t = millis();
  s.contact = (digitalRead(TTP223_PIN) == HIGH);

  int16_t ax, ay, az, gx, gy, gz;
  if (mpuOK) mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  s.ax_g = ax / ACC_LSB_PER_G;
  s.ay_g = ay / ACC_LSB_PER_G;
  s.az_g = az / ACC_LSB_PER_G;

  s.aMag = sqrtf(s.ax_g*s.ax_g + s.ay_g*s.ay_g + s.az_g*s.az_g);
  s.moving = fabsf(s.aMag - 1.0f) > MOVE_MAG_TH;

  s.ir = maxOK ? maxSensor.getIR() : 0;
  s.pulsePresent = (s.ir > IR_PRESENT_TH);

  //  TEMP (UNCHANGED EXACTLY) 
  s.tempC = NAN;
  if (mlxOK) s.tempC = mlx.readObjectTempC();

  s.tempHuman = (!isnan(s.tempC)) &&
                (s.tempC > TEMP_MIN_C) &&
                (s.tempC < TEMP_MAX_C);

  s.worn = s.contact || (s.pulsePresent && s.tempHuman);
  return s;
}

// PPG (FIXED STABILITY ONLY) 
static PpgFeatures computePpgFeatures(uint32_t durationMs) {
  PpgFeatures f{};
  uint32_t start = millis();

  float dc = 0, noise = 0;
  float prevAc = 0, peakAc = 0;
  bool rising = false;

  uint32_t lastPeakMs = 0;
  uint32_t ibi[32];
  int ibiCount = 0;

  double acSum = 0;
  int acN = 0;

  while (millis() - start < durationMs) {

    long ir = 0;
    if (maxOK) {
      ir = maxSensor.getIR();
    }
    if (ir < 0) ir = 0;

    if (dc == 0) dc = ir;
    dc = PPG_DC_ALPHA * dc + (1 - PPG_DC_ALPHA) * ir;

    float ac = (float)ir - dc;
    if (ac < 0) ac = 0;

    noise = PPG_NOISE_ALPHA * noise + (1 - PPG_NOISE_ALPHA) * fabsf(ac);
    float thr = fmaxf(PPG_MIN_THR, noise * PPG_THR_MULT);

    if (ac > prevAc) {
      rising = true;
      if (ac > peakAc) peakAc = ac;
    } else {
      if (rising) {
        uint32_t now = millis();

        if (peakAc > thr && peakAc > 30 && (now - lastPeakMs) > PPG_REFRACTORY_MS) {
          if (lastPeakMs != 0 && ibiCount < 32)
            ibi[ibiCount++] = now - lastPeakMs;

          lastPeakMs = now;
          acSum += peakAc;
          acN++;
          f.peaks++;
        }

        rising = false;
        peakAc = ac;
      }
    }

    prevAc = ac;
    delay(PPG_SAMPLE_MS);
  }

  if (f.peaks < PPG_MIN_PEAKS || ibiCount < 3)
    return f;

  double mean = 0;
  for (int i = 0; i < ibiCount; i++) mean += ibi[i];
  mean /= ibiCount;

  double var = 0;
  for (int i = 0; i < ibiCount; i++) {
    double d = ibi[i] - mean;
    var += d * d;


  }
  var /= (ibiCount - 1);

  f.ibiMeanMs = mean;
  f.ibiStdMs = sqrt(var);
  f.bpm = 60000.0f / f.ibiMeanMs;
  f.acMean = acN ? acSum / acN : 0;
  f.ok = true;

  return f;
}

static bool isWornNow() {
  Sample s = readSample();

  bool worn = s.contact || (s.pulsePresent && s.tempHuman);

  Serial.print("#WORN c="); Serial.print(s.contact);
  Serial.print(" ir="); Serial.print(s.ir);
  Serial.print(" t="); Serial.print(s.tempC);
  Serial.print(" human="); Serial.print(s.tempHuman);
  Serial.print(" worn="); Serial.println(worn);

  return worn;
}

static void encryptData(uint8_t* input, size_t len) {

  uint8_t output[128];
  uint8_t tag[GCM_TAG_BYTES];
  uint8_t nonce[GCM_NONCE_BYTES] = {0};

  
  mbedtls_gcm_setkey(
    &gcm,
    MBEDTLS_CIPHER_ID_AES,
    AES_KEY,
    128
  );

  
  mbedtls_gcm_crypt_and_tag(
    &gcm,
    MBEDTLS_GCM_ENCRYPT,
    len,
    nonce,
    GCM_NONCE_BYTES,
    NULL,
    0,
    input,
    output,
    GCM_TAG_BYTES,
    tag
  );

  Serial.println("#AES-GCM Encryption Complete");
}
//  PROFILE STORAGE 
static void saveProfile(const Profile& p) {
  prefs.begin("auth", false);
  prefs.putBool("valid", p.valid);
  prefs.putFloat("irMean", p.irMean);
  prefs.putFloat("irStd", p.irStd);
  prefs.putFloat("aMagMean", p.aMagMean);
  prefs.putFloat("tempMean", p.tempMean);
  prefs.putFloat("bpmMean", p.bpmMean);
  prefs.putFloat("ibiMean", p.ibiMeanMs);
  prefs.putFloat("ibiStd", p.ibiStdMs);
  prefs.putFloat("acMean", p.acMean);
  prefs.putBytes("gesture", p.gesture.pattern, GESTURE_LEN);
  prefs.putInt("gType", p.gestureType);
  prefs.end();
}

static Profile loadProfile() {
  Profile p{};
  prefs.begin("auth", true);
  p.valid = prefs.getBool("valid", false);
  p.irMean = prefs.getFloat("irMean", 0);
  p.irStd = prefs.getFloat("irStd", 0);
  p.aMagMean = prefs.getFloat("aMagMean", 0);
  p.tempMean = prefs.getFloat("tempMean", 0);
  p.bpmMean = prefs.getFloat("bpmMean", 0);
  p.ibiMeanMs = prefs.getFloat("ibiMean", 0);
  p.ibiStdMs = prefs.getFloat("ibiStd", 0);
  p.acMean = prefs.getFloat("acMean", 0);
  p.gestureType = prefs.getInt("gType", 1);
  prefs.getBytes("gesture", p.gesture.pattern, GESTURE_LEN);

  if (!p.valid) {
    memset(p.gesture.pattern, 0, GESTURE_LEN);
  }
  prefs.end();
  return p;
}

static void eraseProfile() {
  prefs.begin("auth", false);
  prefs.clear();
  prefs.end();
}

static void saveProfile(const Profile& p);
static Profile loadProfile();
static void eraseProfile();

static bool enrolUser(Profile &outP);
static bool verifyUser(const Profile& p);
static Sample readSample();
static PpgFeatures computePpgFeatures(uint32_t durationMs);

static Gesture captureGesture(uint32_t durationMs, int gestureType) {
  Gesture g{};
  uint32_t start = millis();

  Sample first = readSample();
  float baselineX = first.ax_g;
  float baselineZ = first.az_g;

  int idx = 0;

  Serial.println("#GESTURE: Perform now");

  while (millis() - start < durationMs && idx < GESTURE_LEN) {
    
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float value = 0.0f;

    if (gestureType == GESTURE_SHAKE) {
      value = (float)ax / ACC_LSB_PER_G;
    }
    else if (gestureType == GESTURE_TILT) {
      value = (float)az / ACC_LSB_PER_G;
    }

    Serial.print("#GESTURE value=");
    Serial.println(value, 3);

    if (fabs(value) < GESTURE_DEADBAND) {
      g.pattern[idx++] = 0;
    } 
    else if (value > 0) {
      g.pattern[idx++] = 1;
    } 
    else {
      g.pattern[idx++] = -1;
    }

    delay(200);
  }

  Serial.print("#GESTURE PATTERN: ");
  for (int i = 0; i < GESTURE_LEN; i++) {
    Serial.print(g.pattern[i]);
    Serial.print(" ");
  }
  Serial.println();

  return g;
}

static float gestureSimilarity(const Gesture& a, const Gesture& b) {
  int score = 0;

  for (int i = 0; i < GESTURE_LEN; i++) {
    if (a.pattern[i] == b.pattern[i]) {
      score++;
    }
  }

  return (float)score / (float)GESTURE_LEN;
}

static bool hasBothDirections (const Gesture& g) {
  bool hasPositive = false;
  bool hasNegative = false;

  for (int i = 0; i < GESTURE_LEN; i++) {
    if (g.pattern[i] == 1) hasPositive = true;
    if (g.pattern[i] == -1) hasNegative = true;
  }
  return hasPositive && hasNegative;
}

static bool hasMovement(const Gesture& g) {
  int active = 0;

  for (int i = 0; i < GESTURE_LEN; i++) {
    if (g.pattern[i] != 0) {
      active++;
    }
  }

  return active >= 5;
}

static bool matchesBroadTemplate(const Gesture& g, int gestureType) {
  if (!hasMovement(g)) {
    Serial.println("#GESTURE FAIL: no movement detected");
    return false;
  }
  if (!hasBothDirections(g)) {
    Serial.println("#GESTURE FAIL: no direction change detected");
    return false;
  }

  Gesture templateGesture{};

  if (gestureType == GESTURE_SHAKE) {
    memcpy(templateGesture.pattern, SHAKE_TEMPLATE, GESTURE_LEN);
  } 
  else if (gestureType == GESTURE_TILT) {
    memcpy(templateGesture.pattern, TILT_TEMPLATE, GESTURE_LEN);
  } 
  else {
    return false;
  }

  float similarity = gestureSimilarity(g, templateGesture);

  Serial.print("#BROAD TEMPLATE similarity=");
  Serial.println(similarity, 2);

  return similarity >= TEMPLATE_REQUIRED;
}


static bool matchGesture(const Gesture& live, const Gesture& stored, int gestureType) {
  if (!hasMovement(live)) {
    Serial.println("#GESTURE FAIL: no movement detected");
    return false;
  }

  float similarity = gestureSimilarity(live, stored);

  Serial.print("#GESTURE profile similarity=");
  Serial.println(similarity, 2);

  return similarity >= VERIFY_GESTURE_REQUIRED;
}

static void countdown5(const char* message) {
  Serial.println(message);

  for (int i = 10; i > 0; i--) {
    Serial.print("#Starting in ");
    Serial.println(i);
    delay(2000);
  }
}



  
// ENROL 
static bool enrolUser(Profile &outP) {
  Serial.println("#ENROL START - keep finger steady");

  PpgFeatures ppg = computePpgFeatures(ENROLL_MS);

  if (!ppg.ok) {
    Serial.println("#ENROL FAIL: bad PPG");
    return false;
  }

  outP.valid = true;
  outP.bpmMean = ppg.bpm;
  outP.ibiMeanMs = ppg.ibiMeanMs;
  outP.ibiStdMs = ppg.ibiStdMs;
  outP.acMean = ppg.acMean;

  Serial.println("Select gesture:");
  Serial.println("1 = Shake");
  Serial.println("2 = Tilt");

  int type = 1;

  while (Serial.available()== 0);
  type = Serial.parseInt();

  outP.gestureType = type;

  Serial.println("#ENROLL Gesture Setup");

  if (type == GESTURE_SHAKE) {
    Serial.println("#Selected gesture: SHAKE");
    Serial.println("#Instruction: Start palm down, rotate left/right to side position, then return. Repeat twice.");
  } 
  else if (type == GESTURE_TILT) {
    Serial.println("#Selected gesture: TILT");
    Serial.println("#Instruction: Start palm down, tilt up/down, then return. Repeat twice.");
  }

  
  countdown5("#Gesture attempt 1/3: perform the gesture after countdown.");
  Gesture g1 = captureGesture(4000, type);

  if (!matchesBroadTemplate(g1, type)) {
    Serial.println("#ENROL FAIL: first gesture did not match required gesture type");
    return false;
  }

 
  countdown5("#Break. Gesture attempt 2/3: repeat the same gesture.");
  Gesture g2 = captureGesture(4000, type);

  float sim12 = gestureSimilarity(g1, g2);
  Serial.print("#ENROL repeat similarity 1-2=");
  Serial.println(sim12, 2);

  if (!hasMovement(g2) || sim12 < ENROL_REPEAT_REQUIRED) {
    Serial.println("#ENROL FAIL: second gesture did not match first gesture");
    return false;
  }

  
  countdown5("#Break. Gesture attempt 3/3: repeat the same gesture again.");
  Gesture g3 = captureGesture(4000, type);

  float sim13 = gestureSimilarity(g1, g3);
  Serial.print("#ENROL repeat similarity 1-3=");
  Serial.println(sim13, 2);

  if (!hasMovement(g3) || sim13 < ENROL_REPEAT_REQUIRED) {
    Serial.println("#ENROL FAIL: third gesture did not match first gesture");
    return false;
  }

  outP.gesture = g1;

  Serial.println("#ENROL SUCCESS");
  return true;
}


// VERIFY 
static bool verifyUser(const Profile& p) {
  if (!p.valid) {
    Serial.println("#VERIFY FAIL: no profile");
    return false;
  }

  Serial.println("#VERIFY - wear properly");

  uint32_t start = millis();
  while (millis() - start < 5000) {
    if (isWornNow()) break;
    delay(200);
  }

  if (!isWornNow()) {
    Serial.println("#VERIFY FAIL: not worn");
    return false;
  }

  //  PPG 
  PpgFeatures now = computePpgFeatures(VERIFY_MS);

  if (!now.ok) {
    Serial.println("#VERIFY FAIL: bad PPG");
    return false;
  }

  float bpmDiff = fabs(now.bpm - p.bpmMean);
  float ibiDiff = fabs(now.ibiMeanMs - p.ibiMeanMs);

  bool bioOK = (bpmDiff < 20 && ibiDiff < 200);

  //  GESTURE 
  Gesture nowGesture = captureGesture(4000, p.gestureType);

  bool gestureOK = matchGesture(nowGesture, p.gesture, p.gestureType);

  // FINAL DECISION 
  bool bioWeakMatch = (bpmDiff < 40 && ibiDiff < 400);

  if (gestureOK && (bioOK || bioWeakMatch)) {
  Serial.println("#VERIFY SUCCESS");
  return true;
  }

 Serial.println("#VERIFY FAIL");
 return false;

  if (!gestureOK) {
    Serial.println("#VERIFY FAIL: gesture mismatch");
  } else {
    Serial.println("#VERIFY FAIL: biometrics mismatch");
  }

  return false;
}

// SETUP 
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("BOOT OK");

  pinMode(TTP223_PIN, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Init: MPU6050...");
  mpu.initialize();

  Serial.println("Init: MAX3010x...");
  maxOK = maxSensor.begin(Wire, I2C_SPEED_STANDARD);

if (maxOK) {
  maxSensor.setup(0x1f, 4, 2, 100, 411, 4096);
  maxSensor.setPulseAmplitudeRed(0x1F);
  maxSensor.setPulseAmplitudeIR(0x24);
} else {
  Serial.println("MAX30105 NOT FOUND");
}
  Serial.println("Init: MLX90614...");
  mlxOK = mlx.begin();

  mbedtls_gcm_init(&gcm);

  profile = loadProfile();

  state = LOCKED;
}
static void detectGesture(const Sample& s) {

  if (!s.worn) return;

  float dx = fabs(s.ax_g - lastAx);
  float dy = fabs(s.ay_g - lastAy);
  float dz = fabs(s.az_g - lastAz);

  uint32_t now = millis();

  //  SHAKE DETECTION 
  float totalChange = dx + dy + dz;

  if (totalChange > SHAKE_THRESHOLD && (now - lastGestureMs) > GESTURE_COOLDOWN) {
    lastGestureMs = now;

    Serial.println("#GESTURE SHAKE DETECTED");

    if (state == AUTHENTICATED) {
      Serial.println("#ACTION: SHAKE TRIGGER");
    }
  }

  //  TILT DETECTION 
  float tiltChange = fabs(s.az_g - lastAz);

  if (tiltChange > 0.8 && (now - lastGestureMs) > GESTURE_COOLDOWN) {
    lastGestureMs = now;

    Serial.println("#GESTURE TILT DETECTED");

    if (state == AUTHENTICATED) {
      Serial.println("#ACTION: TILT TRIGGER");
    }
  }

  lastAx = s.ax_g;
  lastAy = s.ay_g;
  lastAz = s.az_g;
}

// LOOP 
void loop() {

  // AUTO VERIFY 
    if (state == LOCKED && profile.valid) {
     if (isWornNow()) {

      Serial.println(">> Device worn. Verifying user...");

      if (verifyUser(profile)) {
        Serial.println(">> ACCESS GRANTED");
        state = AUTHENTICATED;
        lastWornMs = millis();
      } 
      else {
      Serial.println(">> ACCESS DENIED");
      delay(1000); 
      }
    }
  }

  // SERIAL COMMANDS 
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'E' || c == 'e') {
      Profile p{};
      if (enrolUser(p)) {
        profile = p;
        saveProfile(profile);
        Serial.println("#ENROL OK");
      } 
      else {
        Serial.println("#ENROL FAIL");
      }
    }

    if (c == 'R' || c == 'r') {
      eraseProfile();
      Serial.println("#PROFILE RESET");
    }
  }
  
  // LOCKED STATE 
  if (state == LOCKED) {
    delay(100);
    return;
  }

  // AUTHENTICATED STATE 
  Sample s = readSample();
  detectGesture(s);

  if (isWornNow()) {
    lastWornMs = millis();
  } 
  else if (millis() - lastWornMs > UNWORN_GRACE_MS) {
    Serial.println("#AUTO LOCK");
    state = LOCKED;
    return;
  }
  // ENCRYPT SENSOR DATA 
  char payload[64];

  snprintf(
    payload,
    sizeof(payload),
    "IR:%ld,TEMP:%.2f,MOVE:%.2f",
    s.ir,
    s.tempC,
    s.aMag
  );

  encryptData((uint8_t*)payload, strlen(payload));

  // STREAM DATA 
  Serial.println("---- LIVE DATA ----");
  Serial.print("Heart Signal (IR): "); Serial.println(s.ir);
  Serial.print("Temperature (C): "); Serial.println(s.tempC);
  Serial.print("Movement (g): "); Serial.println(s.aMag);
  Serial.print("Motion State: "); Serial.println(s.moving ? "Moving" : "Still");
  Serial.println("-------------------");

  delay(SAMPLE_PERIOD_MS);
}
