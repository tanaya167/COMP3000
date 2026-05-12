#include <Wire.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include <Adafruit_MLX90614.h>
#include <math.h>

#include <Preferences.h>

#include "mbedtls/gcm.h"
#include "mbedtls/base64.h"
#include "esp_system.h"

struct Sample;
struct PpgFeatures;

static void emitEncryptedPacket(const char* plain, uint32_t ts, uint32_t seqVal);

static const int SDA_PIN    = 21;
static const int SCL_PIN    = 22;
static const int TTP223_PIN = 4;

MPU6050 mpu(0x68, &Wire);
MAX30105 maxSensor;
Adafruit_MLX90614 mlx;

bool mlxOK = false;
bool maxOK = false;
bool mpuOK = false;


static const uint32_t SAMPLE_PERIOD_MS = 200;     
static const uint32_t ENROLL_MS        = 15000;   
static const uint32_t VERIFY_MS        = 12000;    
static const float    ACC_LSB_PER_G    = 16384.0f;

static const long     IR_PRESENT_TH    = 2000;    
static const float    TEMP_MIN_C       = 25.0;    
static const float    TEMP_MAX_C       = 40.0;

static const float    MOVE_MAG_TH      = 0.25;    
static const uint32_t UNWORN_GRACE_MS  = 1500;    


static const float    HR_MEAN_TOL      = 5.0;    
static const float    MOTION_TOL       = 0.20;    


static const size_t AES_KEY_BYTES      = 16;   
static const size_t GCM_NONCE_BYTES    = 12;
static const size_t GCM_TAG_BYTES      = 16;

static const uint32_t PPG_SAMPLE_MS      = 20;   
static const uint32_t PPG_REFRACTORY_MS  = 300;  
static const float    PPG_DC_ALPHA       = 0.97f; 
static const float    PPG_NOISE_ALPHA    = 0.95f; 
static const float    PPG_THR_MULT       = 1.2f;  
static const float    PPG_MIN_THR        = 50.0f; 
static const int      PPG_MIN_PEAKS      = 2;    



uint8_t AES_KEY[AES_KEY_BYTES] = {
  0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81
};

mbedtls_gcm_context gcm;

Preferences prefs;

enum State { LOCKED, ENROLLING, AUTHENTICATED };
State state = LOCKED;

uint32_t seq = 0;
uint32_t lastWornMs = 0;


struct Profile {
  bool   valid;

  float  irMean;
  float  irStd;

  float  aMagMean;
  float  aMagStd;  

  float  tempMean;
  float  tempStd;   

  float  bpmMean;
  float  ibiMeanMs;
  float  ibiStdMs;
  float  acMean;
};

Profile profile {};

static void wakeMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}

static bool b64Encode(const uint8_t* in, size_t inLen, char* out, size_t outMax) {
  size_t outLen = 0;
  int rc = mbedtls_base64_encode((unsigned char*)out, outMax, &outLen, in, inLen);
  if (rc != 0) return false;
  if (outLen >= outMax) return false;
  out[outLen] = '\0';
  return true;
}

static void makeNonce(uint8_t nonce[GCM_NONCE_BYTES], uint32_t seqVal) {
  nonce[0] = (seqVal >> 24) & 0xFF;
  nonce[1] = (seqVal >> 16) & 0xFF;
  nonce[2] = (seqVal >> 8)  & 0xFF;
  nonce[3] = (seqVal)       & 0xFF;

  uint32_t r1 = esp_random();
  uint32_t r2 = esp_random();
  nonce[4]  = (r1 >> 24) & 0xFF;
  nonce[5]  = (r1 >> 16) & 0xFF;
  nonce[6]  = (r1 >> 8)  & 0xFF;
  nonce[7]  = (r1)       & 0xFF;
  nonce[8]  = (r2 >> 24) & 0xFF;
  nonce[9]  = (r2 >> 16) & 0xFF;
  nonce[10] = (r2 >> 8)  & 0xFF;
  nonce[11] = (r2)       & 0xFF;
}

static bool aesGcmEncrypt(const uint8_t* pt, size_t ptLen,
                          const uint8_t* aad, size_t aadLen,
                          uint32_t seqVal,
                          uint8_t* ct, uint8_t tag[GCM_TAG_BYTES],
                          uint8_t nonce[GCM_NONCE_BYTES]) {
  makeNonce(nonce, seqVal);

  if (mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, AES_KEY, AES_KEY_BYTES * 8) != 0) {
    return false;
  }

  int rc = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT,
                                    ptLen,
                                    nonce, GCM_NONCE_BYTES,
                                    aad, aadLen,
                                    pt, ct,
                                    GCM_TAG_BYTES, tag);
  return (rc == 0);
}

static void emitEncryptedPacket(const char* plain, uint32_t ts, uint32_t seqVal) {
  const uint8_t* pt = (const uint8_t*)plain;
  size_t ptLen = strlen(plain);

  
  static uint8_t ct[512];
  if (ptLen > sizeof(ct)) {
    Serial.println("#ERR plaintext too long for ct buffer");
    return;
  }

  uint8_t tag[GCM_TAG_BYTES];
  uint8_t nonce[GCM_NONCE_BYTES];

  
  uint8_t aad[4] = {
    (uint8_t)((seqVal >> 24) & 0xFF),
    (uint8_t)((seqVal >> 16) & 0xFF),
    (uint8_t)((seqVal >> 8)  & 0xFF),
    (uint8_t)( seqVal        & 0xFF)
  };

  bool ok = aesGcmEncrypt(pt, ptLen, aad, sizeof(aad), seqVal, ct, tag, nonce);
  if (!ok) {
    Serial.println("#ERR AES-GCM encrypt failed");
    return;
  }

  
  char n64[64];
  char ct64[800];
  char tag64[64];

  if (!b64Encode(nonce, GCM_NONCE_BYTES, n64, sizeof(n64)) ||
      !b64Encode(ct, ptLen, ct64, sizeof(ct64)) ||
      !b64Encode(tag, GCM_TAG_BYTES, tag64, sizeof(tag64))) {
    Serial.println("#ERR base64 encode failed");
    return;
  }


  Serial.print("{\"t\":"); Serial.print(ts);
  Serial.print(",\"s\":"); Serial.print(seqVal);
  Serial.print(",\"enc\":1");
  Serial.print(",\"n\":\""); Serial.print(n64); Serial.print("\"");
  Serial.print(",\"ct\":\""); Serial.print(ct64); Serial.print("\"");
  Serial.print(",\"tag\":\""); Serial.print(tag64); Serial.print("\"}");
  Serial.println();
}


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

struct PpgFeatures {
  bool ok;
  int peaks;
  float bpm;
  float ibiMeanMs;
  float ibiStdMs;
  float acMean;
};

static Sample readSample() {
  Sample s{};
  s.t = millis();
  s.contact = (digitalRead(TTP223_PIN) == HIGH);

  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (mpuOK) mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  s.ax_g = ax / ACC_LSB_PER_G;
  s.ay_g = ay / ACC_LSB_PER_G;
  s.az_g = az / ACC_LSB_PER_G;
  s.aMag = sqrtf(s.ax_g*s.ax_g + s.ay_g*s.ay_g + s.az_g*s.az_g);
  s.moving = fabsf(s.aMag - 1.0f) > MOVE_MAG_TH;

  s.ir = -1;
  if (maxOK) s.ir = maxSensor.getIR();
  s.pulsePresent = (s.ir >= 0) && (s.ir > IR_PRESENT_TH);

  s.tempC = NAN;
  if (mlxOK) s.tempC = mlx.readObjectTempC();
  s.tempHuman = (!isnan(s.tempC)) && (s.tempC > TEMP_MIN_C) && (s.tempC < TEMP_MAX_C);

  s.worn = s.contact || (s.pulsePresent && s.tempHuman);
  return s;
}

static void saveProfile(const Profile& p) {
  prefs.begin("auth", false);
  prefs.putBool("valid", p.valid);
  prefs.putFloat("irMean", p.irMean);
  prefs.putFloat("aMag", p.aMagMean);
  prefs.putFloat("temp", p.tempMean);
  prefs.putFloat("bpm",   p.bpmMean);
  prefs.putFloat("ibim",  p.ibiMeanMs);
  prefs.putFloat("ibis",  p.ibiStdMs);
  prefs.putFloat("acm",   p.acMean);
  prefs.putFloat("irStd",  p.irStd);
prefs.putFloat("aStd",   p.aMagStd);
prefs.putFloat("tStd",   p.tempStd);
  prefs.end();
}

static Profile loadProfile() {
  Profile p{};
  prefs.begin("auth", true);
  p.valid   = prefs.getBool("valid", false);
  p.irMean  = prefs.getFloat("irMean", 0);
  p.aMagMean = prefs.getFloat("aMag", 0);
  p.tempMean = prefs.getFloat("temp", 0);
  p.bpmMean   = prefs.getFloat("bpm",  0);
  p.ibiMeanMs = prefs.getFloat("ibim", 0);
  p.ibiStdMs  = prefs.getFloat("ibis", 0);
  p.acMean    = prefs.getFloat("acm",  0);
  p.irStd    = prefs.getFloat("irStd", 0);
  p.aMagStd  = prefs.getFloat("aStd",  0);
  p.tempStd  = prefs.getFloat("tStd",  0);
  prefs.end();
  return p;
}

static void eraseProfile() {
  prefs.begin("auth", false);
  prefs.clear();
  prefs.end();
}




static PpgFeatures computePpgFeatures(uint32_t durationMs) {
  PpgFeatures f{};
  f.ok = false;

  uint32_t start = millis();
  uint32_t lastPeakMs = 0;

 
  float dc = 0.0f;
  float noise = 0.0f;

  
  float prevAc = 0.0f;
  float peakAc = 0.0f;
  bool rising = false;

  
  static const int MAX_IBI = 32;
  uint32_t ibi[MAX_IBI];
  int ibiCount = 0;


  double acSum = 0.0;
  int acN = 0;

  while (millis() - start < durationMs) {
    long ir = maxSensor.getIR(); 
    if (ir < 0) ir = 0;

   
    if (dc == 0.0f) dc = (float)ir;
    dc = PPG_DC_ALPHA * dc + (1.0f - PPG_DC_ALPHA) * (float)ir;

    float ac = (float)ir - dc;

      
  if (millis() - start < 3000) {
    prevAc = ac;
    delay(PPG_SAMPLE_MS);
    continue;
  }

    
    noise = PPG_NOISE_ALPHA * noise + (1.0f - PPG_NOISE_ALPHA) * fabsf(ac);

    float thr = fmaxf(PPG_MIN_THR, noise * PPG_THR_MULT);

    
    if (ac > prevAc) {
      rising = true;
      if (ac > peakAc) peakAc = ac;
    } else {
      if (rising) {
        
        uint32_t now = millis();
        if (peakAc > thr && (now - lastPeakMs) > PPG_REFRACTORY_MS) {
          
          if (lastPeakMs != 0 && ibiCount < MAX_IBI) {
            ibi[ibiCount++] = now - lastPeakMs;
          }
          lastPeakMs = now;

          acSum += peakAc;
          acN++;

          f.peaks++;
        }
      }
      
      rising = false;
      peakAc = ac;
    }
    static uint32_t lastDbg = 0;
      if (millis() - lastDbg > 1000) {
      lastDbg = millis();
      Serial.print("#PPG ac="); Serial.print(ac, 1);
      Serial.print(" noise="); Serial.print(noise, 1);
      Serial.print(" thr="); Serial.print(thr, 1);
      Serial.print(" peaks="); Serial.println(f.peaks);
    }

    prevAc = ac;
    delay(PPG_SAMPLE_MS);
  }

  
  if (f.peaks < PPG_MIN_PEAKS || ibiCount < 4) {
    return f; 
  }

  
  double mean = 0.0;
  for (int i = 0; i < ibiCount; i++) mean += ibi[i];
  mean /= ibiCount;

  double var = 0.0;
  for (int i = 0; i < ibiCount; i++) {
    double d = (double)ibi[i] - mean;
    var += d * d;
  }
  var /= (ibiCount - 1);

  f.ibiMeanMs = (float)mean;
  f.ibiStdMs  = (float)sqrt(var);

  
  f.bpm = 60000.0f / f.ibiMeanMs;

  
  f.acMean = (acN ? (float)(acSum / acN) : 0.0f);

  f.ok = true;
  return f;
}


static bool isProperlyWornOnce() {
  Sample s = readSample();
  return (s.contact && s.pulsePresent && s.tempHuman);
}

static bool waitProperlyWorn(uint32_t timeoutMs) {
  uint32_t start = millis();
  uint32_t lastPrint = 0;

  while (millis() - start < timeoutMs) {
    Sample s = readSample();

    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.print("#WORNCHK contact="); Serial.print(s.contact ? "1" : "0");
      Serial.print(" pulse="); Serial.print(s.pulsePresent ? "1" : "0");
      Serial.print(" ir="); Serial.print(s.ir);
      Serial.print(" temp="); Serial.print(isnan(s.tempC) ? -999.0 : s.tempC);
      Serial.print(" tempHuman="); Serial.print(s.tempHuman ? "1" : "0");
      Serial.print(" worn="); Serial.println(s.worn ? "1" : "0");
    }

    if (s.contact && s.pulsePresent && s.tempHuman) return true;

    delay(100);
  }
  return false;
}




static bool enrolUser(Profile &outP) {
  Serial.println("#ENROLL Put it on properly (touch + pulse + warm). Starting in ~5s...");
  if (!waitProperlyWorn(5000)) {
    Serial.println("#ENROLL Failed: not properly worn.");
    return false;
  }


  uint32_t start = millis();
  double aSum = 0; uint32_t aN = 0;
  double tSum = 0; uint32_t tN = 0;

  while (millis() - start < ENROLL_MS) {
    Sample s = readSample();
    if (s.contact && s.pulsePresent && s.tempHuman) {
      aSum += s.aMag; aN++;
      if (!isnan(s.tempC)) { tSum += s.tempC; tN++; }
    }
    delay(SAMPLE_PERIOD_MS);
  }

 
 Serial.println("#ENROLL PPG: measuring (multi-sample)...");

PpgFeatures ppg;
int validCount = 0;

float bpmSum = 0, ibiMeanSum = 0, ibiStdSum = 0, acSum = 0;

for (int i = 0; i < 3; i++) {
  Serial.print("#ENROLL sample "); Serial.println(i + 1);

  PpgFeatures temp = computePpgFeatures(5000);

  if (temp.ok) {
    bpmSum += temp.bpm;
    ibiMeanSum += temp.ibiMeanMs;
    ibiStdSum += temp.ibiStdMs;
    acSum += temp.acMean;
    validCount++;
  } else {
    Serial.println("#ENROLL sample failed (low quality)");
  }
}

if (validCount == 0) {
  Serial.println("#ENROLL Failed: no valid PPG samples.");
  return false;
}

ppg.bpm       = bpmSum / validCount;
ppg.ibiMeanMs = ibiMeanSum / validCount;
ppg.ibiStdMs  = ibiStdSum / validCount;
ppg.acMean    = acSum / validCount;
ppg.ok = true;

  outP.valid   = true;
  outP.aMagMean= (aN ? (float)(aSum / aN) : 1.0f);
  outP.tempMean= (tN ? (float)(tSum / tN) : NAN);

  outP.bpmMean   = ppg.bpm;
  outP.ibiMeanMs = ppg.ibiMeanMs;
  outP.ibiStdMs  = ppg.ibiStdMs;
  outP.acMean    = ppg.acMean;

  Serial.print("#ENROLL OK ");
  Serial.print("aMagMean="); Serial.print(outP.aMagMean, 3);
  Serial.print(" tempMean="); Serial.print(outP.tempMean, 2);

  Serial.print(" | PPG bpm="); Serial.print(outP.bpmMean, 1);
  Serial.print(" ibiMeanMs="); Serial.print(outP.ibiMeanMs, 1);
  Serial.print(" ibiStdMs="); Serial.print(outP.ibiStdMs, 1);
  Serial.print(" acMean="); Serial.println(outP.acMean, 0);

  return true;
}

static bool verifyUser(const Profile& p) {
  if (!p.valid) return false;

  Serial.println("#VERIFY Wear properly (touch + pulse + warm).");
  if (!waitProperlyWorn(5000)) {
    Serial.println("#VERIFY Failed: not properly worn.");
    return false;
  }

  uint32_t start = millis();
  double aSum = 0; uint32_t aN = 0;

  while (millis() - start < VERIFY_MS) {
    Sample s = readSample();
    if (s.contact && s.pulsePresent && s.tempHuman) {
      aSum += s.aMag; aN++;
    }
    delay(SAMPLE_PERIOD_MS);
  }
  float aNow = (aN ? (float)(aSum / aN) : 1.0f);

  Serial.println("#VERIFY PPG: measuring...");
  PpgFeatures now = computePpgFeatures(VERIFY_MS);

  if (!now.ok) {
    Serial.println("#VERIFY Failed: PPG quality too low.");
    return false;
  }

  
  const float BPM_TOL      = 20.0f;     
  const float IBI_MEAN_TOL = 200.0f;    
  const float IBI_STD_TOL  = 200.0f;     
  const float AC_TOL_PCT   = 0.50f;     
  const float AMAG_TOL     = 0.15f;

  bool bpmOK  = fabsf(now.bpm - p.bpmMean) <= BPM_TOL;
  bool ibiMOK = fabsf(now.ibiMeanMs - p.ibiMeanMs) <= IBI_MEAN_TOL;
  bool ibiSOK = fabsf(now.ibiStdMs  - p.ibiStdMs)  <= IBI_STD_TOL;

  float acTol = fmaxf(200.0f, AC_TOL_PCT * p.acMean);
  bool acOK   = fabsf(now.acMean - p.acMean) <= acTol;

  bool aOK    = fabsf(aNow - p.aMagMean) <= AMAG_TOL;

  if (now.ibiStdMs > 300) {
  Serial.println("#VERIFY Reject: unstable signal");
  return false;
}

  
  int score = 0;
  if (bpmOK)  score += 2;
  if (ibiMOK) score += 2;
 
  if (acOK)   score += 1;
  if (aOK)    score += 1;

  Serial.print("#VERIFY PPG now bpm="); Serial.print(now.bpm, 1);
  Serial.print(" ibiMeanMs="); Serial.print(now.ibiMeanMs, 1);
  Serial.print(" ibiStdMs="); Serial.print(now.ibiStdMs, 1);
  Serial.print(" acMean="); Serial.print(now.acMean, 0);
  Serial.print(" | aMagNow="); Serial.println(aNow, 3);

  Serial.print("#VERIFY match bpm="); Serial.print(bpmOK ? "OK" : "FAIL");
  Serial.print(" ibiMean="); Serial.print(ibiMOK ? "OK" : "FAIL");
  Serial.print(" ibiStd="); Serial.print(ibiSOK ? "OK" : "FAIL");
  Serial.print(" ac="); Serial.print(acOK ? "OK" : "FAIL");
  Serial.print(" aMag="); Serial.print(aOK ? "OK" : "FAIL");
  Serial.print(" | score="); Serial.println(score);


  return (score >= 3);
}


void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TTP223_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(200);

  Serial.println("Init: MPU6050...");
  mpu.initialize();
  delay(100);
  wakeMPU6050();
  mpuOK = true;

  Serial.println("Init: MAX3010x...");
  maxOK = maxSensor.begin(Wire, I2C_SPEED_STANDARD);
  if (maxOK) {
    maxSensor.setup(0x1f, 4, 2, 100, 411, 4096);
    maxSensor.setPulseAmplitudeRed(0x1F);
    maxSensor.setPulseAmplitudeIR(0x24);
  }

  Serial.println("Init: MLX90614...");
  mlxOK = mlx.begin();

  mbedtls_gcm_init(&gcm);

  profile = loadProfile();

  Serial.print("Status: MAX="); Serial.print(maxOK ? "OK" : "FAIL");
  Serial.print(" MLX="); Serial.print(mlxOK ? "OK" : "FAIL");
  Serial.print(" MPU="); Serial.println(mpuOK ? "OK" : "FAIL");

  Serial.print("#PROFILE ");
  Serial.println(profile.valid ? "LOADED" : "NONE (press 'E' to enrol)");

  Serial.println("#Commands: E=enrol  U=unlock/verify  L=lock  S=show profile  R=reset profile");
  state = LOCKED;
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'E' || c == 'e') {
      state = ENROLLING;
      Profile p{};
      if (enrolUser(p)) {
        profile = p;
        saveProfile(profile);
        Serial.println("#ENROLL Saved profile. Now press 'U' to verify/unlock.");
      }
      state = LOCKED;
    } else if (c == 'U' || c == 'u') {
      if (!profile.valid) {
        Serial.println("#No profile. Press 'E' to enrol first.");
        state = LOCKED;
      } else {
        bool ok = verifyUser(profile);
        if (ok) {
          Serial.println("#UNLOCK OK -> AUTHENTICATED");
          state = AUTHENTICATED;
          lastWornMs = millis();
        } else {
          Serial.println("#UNLOCK FAIL -> LOCKED");
          state = LOCKED;
        }
      }
    } else if (c == 'L' || c == 'l') {
      Serial.println("#LOCKED");
      state = LOCKED;
    } else if (c == 'S' || c == 's') {
      Serial.print("#PROFILE valid="); Serial.print(profile.valid ? 1 : 0);
      Serial.print(" irMean="); Serial.print(profile.irMean, 0);
      Serial.print(" aMagMean="); Serial.print(profile.aMagMean, 3);
      Serial.print(" tempMean="); Serial.println(profile.tempMean, 2);
    } else if (c == 'R' || c == 'r') {
      eraseProfile();
      profile = Profile{};
      Serial.println("#PROFILE erased. Press 'E' to enrol.");
      state = LOCKED;
    }
  }

  Sample s = readSample();

  if (state == AUTHENTICATED) {
    if (s.worn) {
      lastWornMs = millis();
    } else if (millis() - lastWornMs > UNWORN_GRACE_MS) {
      Serial.println("#AUTOLOCK removed/unworn -> LOCKED");
      state = LOCKED;
    }
  }

  if (state != AUTHENTICATED) {
    delay(SAMPLE_PERIOD_MS);
    return;
  }

  char plain[256];
  snprintf(
    plain, sizeof(plain),
    "{\"t\":%lu,\"s\":%lu,\"c\":%d,\"pp\":%d,\"ir\":%ld,\"tc\":%.2f,\"th\":%d,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"am\":%.3f,\"mv\":%d}",
    (unsigned long)s.t,
    (unsigned long)seq,
    s.contact ? 1 : 0,
    s.pulsePresent ? 1 : 0,
    s.ir,
    (isnan(s.tempC) ? -999.0f : s.tempC),
    s.tempHuman ? 1 : 0,
    s.ax_g, s.ay_g, s.az_g, s.aMag,
    s.moving ? 1 : 0
  );

  emitEncryptedPacket(plain, s.t, seq);

  seq++;
  delay(SAMPLE_PERIOD_MS);
}







