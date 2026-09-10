#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>
#include <vector>
#include <algorithm>
// ==================================================
// PIN CONNECTIONS & CONSTANTS
// ==================================================
const int MOTOR_PWM_PIN = 5;
const int PWM_FREQUENCY = 500;
const int PWM_RESOLUTION = 8;
const int MOTOR_OFF_PWM = 255;
const int MOTOR_FULL_PWM = 0;
const unsigned long SENSOR_TIMEOUT_MS = 2000;
const size_t MAX_TEST_SAMPLES = 900;
const size_t MIN_TEST_SAMPLES = 20;
// Wi-Fi Access Point Configuration
const char* WIFI_NAME = "Mighty-Airflow-Control";
const char* WIFI_PASSWORD = "Mighty2026";
const char* SENSOR_TOKEN = "MightySensor2026";
// ==================================================
// AIR VELOCITY CALIBRATION TABLE (RUNTIME / WEBSITE EDITABLE)
// --------------------------------------------------
// The relationship between motor PWM percentage and measured air velocity
// is NOT hardcoded any more. It is calibrated live from the "Air Velocity
// Calibration" section of the dashboard: for each of 11 fixed PWM steps
// (0%, 10%, 20%, ..., 100%) the user runs the motors, measures the real
// air velocity with an external anemometer, and enters it into the
// website. Those 11 measured points are the SINGLE SOURCE OF TRUTH used
// by velocityToMotorPercent() / motorPercentToVelocity() for Mode 1 and
// Mode 2 -- there is nothing to edit in this firmware to recalibrate.
//
// The table is persisted in the ESP32's non-volatile Preferences (NVS)
// storage so it survives page refreshes, ESP32 restarts and Wi-Fi
// reconnects. The dashboard also keeps a browser-side copy in
// localStorage purely as a fast/offline UI cache; the ESP32's copy is
// always the authoritative one used to actually run the motors.
// ==================================================
const int CALIBRATION_POINT_COUNT = 11;      // 0%,10%,20%,...,100%
const int CALIBRATION_STEP_PERCENT = 10;
const char* CALIBRATION_NVS_NAMESPACE = "aircalib";

struct AirVelocityCalibrationPoint {
  int motorPercent;     // 0,10,20,...,100 -- fixed, never changes at runtime
  float velocityKmh;    // measured value entered by the user (NAN until set)
  bool hasValue;        // false until the user has entered/saved this point
};

AirVelocityCalibrationPoint calibrationTable[CALIBRATION_POINT_COUNT];

Preferences calibrationPrefs;

// true only when all 11 points are present, numeric, non-negative, and
// strictly increasing from 0% to 100%. While false, velocity-based Mode 1
// and Mode 2 tests are refused and the dashboard shows a calibration error
// banner. Manual direct-PWM control and the calibration tab itself are
// unaffected, since neither depends on the table being complete.
bool calibrationValid = false;
String calibrationErrorMessage = "Calibration has not been performed yet.";

// True while the Air Velocity Calibration tab is actively driving the
// motors at a fixed SET-X% step (see handleCalibSetPwm() / handleCalibStop()).
// Mode 1 / Mode 2 refuse to start while this is true, and calibration
// refuses to start while a Mode 1/2 test is active.
bool calibrationModeActive = false;
int calibrationRunningPercent = -1;
// ==================================================
// CARRIER-BODY AERODYNAMIC DRAG OFFSET (OPTIONAL, RUNTIME EDITABLE)
// --------------------------------------------------
// Whatever physically holds/carries the bike in the airflow (a stand, a
// fixture, a mounting arm) generates its own aerodynamic drag on the load
// cell, on top of the bike+rider's drag. This table lets the user measure
// that carrier-only drag once (bike removed) at each of the same 11 PWM
// steps used for air-velocity calibration, so it can be subtracted out of
// every Mode 1 / Mode 2 force reading before CdA/power are computed.
//
// Unlike the air-velocity table, this one is OPTIONAL: an uncalibrated (or
// partially calibrated) rig behaves exactly as before this feature existed
// -- interpolatedCarrierOffsetN() simply returns 0.0f wherever no data has
// been entered, so Mode 1/Mode 2 are never gated on this table.
// ==================================================
const char* CARRIER_OFFSET_NVS_NAMESPACE = "carrieroff";

struct CarrierOffsetPoint {
  int motorPercent;      // 0,10,20,...,100 -- fixed, never changes at runtime
  float offsetForceN;    // measured carrier-only drag force (NAN until set)
  bool hasValue;         // false until the user has entered/saved this point
};

CarrierOffsetPoint carrierOffsetTable[CALIBRATION_POINT_COUNT];

Preferences carrierOffsetPrefs;

// A single constant CdA offset for the carrier body (mount/stand/fixture),
// entered directly by the user rather than measured across the PWM range.
// Unlike air velocity or the carrier force offset above, a drag
// coefficient*area (CdA) is a property of the shape and is treated as
// roughly speed-independent, so one value is enough -- it is simply
// subtracted from the final computed CdA in every Mode 1/Mode 2 result to
// produce a bike+rider-only "Net CdA". Defaults to 0.0f (no change to
// existing behavior) until the user sets it.
const char* CARRIER_CDA_NVS_NAMESPACE = "carriercda";
Preferences carrierCdaPrefs;
float carrierBodyCdA = 0.0f;
bool carrierBodyCdASet = false;
// ==================================================
// MODE 2 AUTOMATIC SPEED SWEEP TARGETS
// --------------------------------------------------
// The six fixed target air velocities for the automatic sweep. The PWM
// required for each is calculated automatically from the calibration
// table via velocityToMotorPercent() -- there is no manual PWM mapping to
// configure on the dashboard any more.
// ==================================================
const size_t MODE2_SPEED_COUNT = 6;
const float MODE2_TARGET_VELOCITIES_KMH[MODE2_SPEED_COUNT] = {
  10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 35.0f
};
const unsigned long RAMP_DURATION_MS = 3000;
const unsigned long PREP_DURATION_MS = 5000;
const unsigned long MEASURE_DURATION_MS = 10000;
// ==================================================
// OBJECTS & RUNTIME VARIABLES
// ==================================================
WebServer server(80);
int motorSpeedPercent = 0;
bool pwmConfigured = false;
unsigned long lastBrowserContact = 0;
float displayedForceN = 0.0;
// millis() timestamp of the last time displayedForceN was actually
// reassigned from a fresh /sensor_update packet. Used purely as a
// diagnostic ("how many ms ago did the live reading last change") so a
// frozen live display can be told apart from a genuinely idle sensor --
// see forceAgeMs in handleData().
unsigned long lastForceUpdateMs = 0;
float pendingSampleForceN = 0.0;
uint32_t lastSensorSequence = 0;
bool newSensorSample = false;
bool calibrated = false;
bool sensorConnected = false;
unsigned long lastGoodSensorReading = 0;
unsigned long lastSerialPrint = 0;
IPAddress sensorNodeIP;
bool sensorNodeKnown = false;
// ==================================================
// SHARED NON-BLOCKING TEST STATE MACHINE
// ==================================================
enum TestState {
  TEST_IDLE = 0,
  TEST_ACCELERATING,
  TEST_PREPARATION,
  TEST_MEASURING,
  TEST_COMPLETED,
  TEST_ERROR
};
enum TestMode {
  MODE_NONE = 0,
  MODE_ONE = 1,
  MODE_TWO = 2
};
TestState currentTestState = TEST_IDLE;
TestMode currentTestMode = MODE_NONE;
unsigned long stateStartTime = 0;
unsigned long testStartTime = 0;
int rampStartPercent = 0;
int rampTargetPercent = 0;
// Mode 1 requested target (velocity-driven; PWM is derived, never entered directly).
float testTargetVelocityKmh = 0.0f;
float testVelocityMps = 0.0f;
float testRequiredPwmPercent = 0.0f;  // unrounded diagnostic value from the table
float testAirDensity = 1.225;
std::vector<float> recordedRawForces;
std::vector<uint32_t> recordedSampleTimesMs;
String testErrorMessage = "";
bool testWasCancelled = false;
struct TestResultData {
  float targetVelocityKmh;
  float velocityMps;
  int motorPercent;            // rounded PWM percentage actually applied
  float requestedPwmPercent;   // unrounded PWM percentage calculated from the table
  float airDensity;
  int totalReadings;
  int acceptedReadings;
  int excludedReadings;
  float avgForce;
  float medianForce;
  float minForce;
  float maxForce;
  float stdDev;
  float aeroPower;
  float cdA;
  float cdaExcludingCarrier;    // cdA minus the constant carrier-body CdA offset (0 offset if unset)
  float carrierOffsetAppliedN;  // carrier-body drag subtracted before stats (0 if uncalibrated)
  bool valid;
  std::vector<float> rawForces;
  std::vector<uint32_t> sampleTimesMs;
  std::vector<uint8_t> acceptedMask;
} latestResult;
struct Mode2StageResult {
  float velocityKmh;
  float velocityMps;
  int motorPercent;
  float requestedPwmPercent;
  int totalReadings;
  int acceptedReadings;
  int excludedReadings;
  float avgForce;
  float medianForce;
  float minForce;
  float maxForce;
  float stdDev;
  float aeroPower;
  float cdA;
  float cdaExcludingCarrier;    // cdA minus the constant carrier-body CdA offset (0 offset if unset)
  float carrierOffsetAppliedN;  // carrier-body drag subtracted before stats (0 if uncalibrated)
  bool valid;
  std::vector<float> rawForces;
};
// Populated at the start of each Mode 2 sweep by calling
// velocityToMotorPercent() for every fixed target velocity.
struct Mode2StageTarget {
  float velocityKmh;
  float velocityMps;
  int motorPercent;
  float requestedPwmPercent;
};
Mode2StageTarget mode2Targets[MODE2_SPEED_COUNT];
Mode2StageResult mode2Results[MODE2_SPEED_COUNT];
size_t mode2StageIndex = 0;
float mode2AverageCdA = 0.0f;
float mode2AverageNetCdA = 0.0f;
float mode2AverageForce = 0.0f;
float mode2AveragePower = 0.0f;
int mode2TotalAccepted = 0;
int mode2TotalExcluded = 0;
unsigned long mode2TotalDurationMs = 0;
bool mode2ResultValid = false;
// ==================================================
// MOTOR CONTROL
// ==================================================
void setMotorSpeed(int requestedPercent) {
  requestedPercent = constrain(requestedPercent, 0, 100);
  motorSpeedPercent = requestedPercent;
  int invertedDuty = map(requestedPercent, 0, 100, MOTOR_OFF_PWM, MOTOR_FULL_PWM);
  if (pwmConfigured) {
    ledcWrite(MOTOR_PWM_PIN, invertedDuty);
  }
}
void stopAllMotors() {
  setMotorSpeed(0);
}
bool testIsActive() {
  return currentTestState >= TEST_ACCELERATING &&
         currentTestState <= TEST_MEASURING;
}
void abortActiveTest(const String& reason, bool cancelled = false) {
  stopAllMotors();
  latestResult.valid = false;
  mode2ResultValid = false;
  testErrorMessage = reason;
  testWasCancelled = cancelled;
  currentTestState = TEST_ERROR;
  Serial.println("TEST ABORTED: " + reason);
}
// ==================================================
// AIR VELOCITY CALIBRATION: STORAGE, VALIDATION & LOOKUP
// --------------------------------------------------
// Storage: 11 floats persisted in ESP32 NVS (Preferences) under the
// "aircalib" namespace, one key per PWM step ("p0".."p100").
// Validation: all 11 points must exist, be numeric, be non-negative and be
// strictly increasing from 0% to 100% -- otherwise Mode 1 / Mode 2 are
// disabled until the table is fixed from the Air Velocity Calibration tab.
// Lookup: both directions search calibrationTable for the two rows that
// surround the requested value and linearly interpolate between them
// (piecewise linear -- never a single equation for the full 0-100% range).
// ==================================================
void initCalibrationDefaults() {
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    calibrationTable[i].motorPercent = i * CALIBRATION_STEP_PERCENT;
    calibrationTable[i].velocityKmh = NAN;
    calibrationTable[i].hasValue = false;
  }
  // 0% (motors off) defaults to 0 km/h, but the user can still override it
  // if the rig has residual airflow at idle.
  calibrationTable[0].velocityKmh = 0.0f;
  calibrationTable[0].hasValue = true;
}

void loadCalibrationFromStorage() {
  initCalibrationDefaults();
  calibrationPrefs.begin(CALIBRATION_NVS_NAMESPACE, true);
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    String key = "p" + String(calibrationTable[i].motorPercent);
    if (calibrationPrefs.isKey(key.c_str())) {
      float storedValue = calibrationPrefs.getFloat(key.c_str(), NAN);
      if (isfinite(storedValue)) {
        calibrationTable[i].velocityKmh = storedValue;
        calibrationTable[i].hasValue = true;
      }
    }
  }
  calibrationPrefs.end();
}

void saveCalibrationToStorage() {
  calibrationPrefs.begin(CALIBRATION_NVS_NAMESPACE, false);
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    String key = "p" + String(calibrationTable[i].motorPercent);
    if (calibrationTable[i].hasValue && isfinite(calibrationTable[i].velocityKmh)) {
      calibrationPrefs.putFloat(key.c_str(), calibrationTable[i].velocityKmh);
    } else {
      calibrationPrefs.remove(key.c_str());
    }
  }
  calibrationPrefs.end();
}

void resetCalibrationToDefaults() {
  calibrationPrefs.begin(CALIBRATION_NVS_NAMESPACE, false);
  calibrationPrefs.clear();
  calibrationPrefs.end();
  initCalibrationDefaults();
}

bool computeCalibrationValidity(String& errorMessage) {
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    if (!calibrationTable[i].hasValue || !isfinite(calibrationTable[i].velocityKmh)) {
      errorMessage = String(calibrationTable[i].motorPercent) +
                     "% measured air velocity has not been entered yet.";
      return false;
    }
    if (calibrationTable[i].velocityKmh < 0.0f) {
      errorMessage = String(calibrationTable[i].motorPercent) +
                     "% velocity must not be negative.";
      return false;
    }
  }
  for (int i = 1; i < CALIBRATION_POINT_COUNT; ++i) {
    if (calibrationTable[i].velocityKmh <= calibrationTable[i - 1].velocityKmh) {
      errorMessage = String(calibrationTable[i].motorPercent) +
                     "% velocity must be greater than " +
                     String(calibrationTable[i - 1].motorPercent) + "% velocity.";
      return false;
    }
  }
  errorMessage = "";
  return true;
}

void refreshCalibrationValidity() {
  calibrationValid = computeCalibrationValidity(calibrationErrorMessage);
}

float calibrationMinVelocityKmh() {
  return calibrationTable[0].velocityKmh;
}
float calibrationMaxVelocityKmh() {
  return calibrationTable[CALIBRATION_POINT_COUNT - 1].velocityKmh;
}
// Requested velocity (km/h) -> unrounded motor PWM percentage.
// Round the RESULT only when it is actually sent to setMotorSpeed().
float velocityToMotorPercent(float requestedVelocityKmh) {
  if (!calibrationValid || !isfinite(requestedVelocityKmh)) return NAN;
  if (requestedVelocityKmh < calibrationMinVelocityKmh() ||
      requestedVelocityKmh > calibrationMaxVelocityKmh()) {
    return NAN;  // reject / never extrapolate outside the calibrated range
  }
  for (int i = 0; i + 1 < CALIBRATION_POINT_COUNT; ++i) {
    float vLow = calibrationTable[i].velocityKmh;
    float vHigh = calibrationTable[i + 1].velocityKmh;
    if (requestedVelocityKmh >= vLow && requestedVelocityKmh <= vHigh) {
      if (requestedVelocityKmh == vLow) return (float)calibrationTable[i].motorPercent;
      if (requestedVelocityKmh == vHigh) return (float)calibrationTable[i + 1].motorPercent;
      float pwmLow = (float)calibrationTable[i].motorPercent;
      float pwmHigh = (float)calibrationTable[i + 1].motorPercent;
      if (vHigh <= vLow) return pwmLow;  // defensive: avoid divide-by-zero on a flat segment
      return pwmLow + ((requestedVelocityKmh - vLow) / (vHigh - vLow)) * (pwmHigh - pwmLow);
    }
  }
  return NAN;
}
// Motor PWM percentage (may be fractional) -> estimated air velocity (km/h).
// This is an ESTIMATE derived from the calibration table, not a live
// anemometer measurement.
float motorPercentToVelocity(float motorPercent) {
  if (!calibrationValid || !isfinite(motorPercent)) return NAN;
  float minPwm = (float)calibrationTable[0].motorPercent;
  float maxPwm = (float)calibrationTable[CALIBRATION_POINT_COUNT - 1].motorPercent;
  if (motorPercent < minPwm || motorPercent > maxPwm) return NAN;
  for (int i = 0; i + 1 < CALIBRATION_POINT_COUNT; ++i) {
    float pLow = (float)calibrationTable[i].motorPercent;
    float pHigh = (float)calibrationTable[i + 1].motorPercent;
    if (motorPercent >= pLow && motorPercent <= pHigh) {
      if (motorPercent == pLow) return calibrationTable[i].velocityKmh;
      if (motorPercent == pHigh) return calibrationTable[i + 1].velocityKmh;
      float vLow = calibrationTable[i].velocityKmh;
      float vHigh = calibrationTable[i + 1].velocityKmh;
      return vLow + ((motorPercent - pLow) / (pHigh - pLow)) * (vHigh - vLow);
    }
  }
  return NAN;
}
// ==================================================
// CARRIER-BODY OFFSET: STORAGE & LOOKUP
// --------------------------------------------------
// Same persistence pattern as the air-velocity table (11 floats in ESP32
// NVS, one key per PWM step), but optional: any point without a stored
// value is simply skipped, and interpolatedCarrierOffsetN() returns 0.0f
// wherever there isn't enough data to interpolate.
// ==================================================
void initCarrierOffsetDefaults() {
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    carrierOffsetTable[i].motorPercent = i * CALIBRATION_STEP_PERCENT;
    carrierOffsetTable[i].offsetForceN = NAN;
    carrierOffsetTable[i].hasValue = false;
  }
}

void loadCarrierOffsetFromStorage() {
  initCarrierOffsetDefaults();
  carrierOffsetPrefs.begin(CARRIER_OFFSET_NVS_NAMESPACE, true);
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    String key = "p" + String(carrierOffsetTable[i].motorPercent);
    if (carrierOffsetPrefs.isKey(key.c_str())) {
      float storedValue = carrierOffsetPrefs.getFloat(key.c_str(), NAN);
      if (isfinite(storedValue)) {
        carrierOffsetTable[i].offsetForceN = storedValue;
        carrierOffsetTable[i].hasValue = true;
      }
    }
  }
  carrierOffsetPrefs.end();
}

void saveCarrierOffsetToStorage() {
  carrierOffsetPrefs.begin(CARRIER_OFFSET_NVS_NAMESPACE, false);
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    String key = "p" + String(carrierOffsetTable[i].motorPercent);
    if (carrierOffsetTable[i].hasValue && isfinite(carrierOffsetTable[i].offsetForceN)) {
      carrierOffsetPrefs.putFloat(key.c_str(), carrierOffsetTable[i].offsetForceN);
    } else {
      carrierOffsetPrefs.remove(key.c_str());
    }
  }
  carrierOffsetPrefs.end();
}

void resetCarrierOffsetToDefaults() {
  carrierOffsetPrefs.begin(CARRIER_OFFSET_NVS_NAMESPACE, false);
  carrierOffsetPrefs.clear();
  carrierOffsetPrefs.end();
  initCarrierOffsetDefaults();
}

int carrierOffsetCalibratedCount() {
  int count = 0;
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    if (carrierOffsetTable[i].hasValue) ++count;
  }
  return count;
}

// Piecewise-linear interpolation across whatever carrier-offset points have
// been calibrated so far. Finds the nearest calibrated point at or below
// motorPercent and the nearest calibrated point at or above it (which need
// not be immediately adjacent PWM steps if some points in between are still
// empty) and interpolates between those two. Falls back to the single
// nearest calibrated point if only one side has data, and to 0.0f (no
// offset) if nothing has been calibrated at all -- so an uncalibrated rig
// behaves exactly as it did before this feature existed.
float interpolatedCarrierOffsetN(float motorPercent) {
  if (!isfinite(motorPercent)) return 0.0f;
  motorPercent = constrain(motorPercent, 0.0f, 100.0f);
  int lowerIndex = -1;
  int upperIndex = -1;
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    if (!carrierOffsetTable[i].hasValue) continue;
    float p = (float)carrierOffsetTable[i].motorPercent;
    if (p <= motorPercent && (lowerIndex == -1 || p > (float)carrierOffsetTable[lowerIndex].motorPercent)) {
      lowerIndex = i;
    }
    if (p >= motorPercent && (upperIndex == -1 || p < (float)carrierOffsetTable[upperIndex].motorPercent)) {
      upperIndex = i;
    }
  }
  if (lowerIndex == -1 && upperIndex == -1) return 0.0f;  // nothing calibrated yet
  if (lowerIndex == -1) return carrierOffsetTable[upperIndex].offsetForceN;   // only points above
  if (upperIndex == -1) return carrierOffsetTable[lowerIndex].offsetForceN;   // only points below
  if (lowerIndex == upperIndex) return carrierOffsetTable[lowerIndex].offsetForceN;  // exact match
  float pLow = (float)carrierOffsetTable[lowerIndex].motorPercent;
  float pHigh = (float)carrierOffsetTable[upperIndex].motorPercent;
  float vLow = carrierOffsetTable[lowerIndex].offsetForceN;
  float vHigh = carrierOffsetTable[upperIndex].offsetForceN;
  if (pHigh <= pLow) return vLow;  // defensive: avoid divide-by-zero
  return vLow + ((motorPercent - pLow) / (pHigh - pLow)) * (vHigh - vLow);
}
// ==================================================
// CARRIER-BODY CdA OFFSET: STORAGE (single constant value)
// --------------------------------------------------
// Simpler than the tables above: one float, persisted under its own tiny
// NVS namespace. Defaults to 0.0f (no effect) until the user sets it.
// ==================================================
void loadCarrierCdAFromStorage() {
  carrierCdaPrefs.begin(CARRIER_CDA_NVS_NAMESPACE, true);
  if (carrierCdaPrefs.isKey("cda")) {
    float storedValue = carrierCdaPrefs.getFloat("cda", NAN);
    if (isfinite(storedValue)) {
      carrierBodyCdA = storedValue;
      carrierBodyCdASet = true;
    }
  }
  carrierCdaPrefs.end();
}

void saveCarrierCdAToStorage(float value) {
  carrierCdaPrefs.begin(CARRIER_CDA_NVS_NAMESPACE, false);
  carrierCdaPrefs.putFloat("cda", value);
  carrierCdaPrefs.end();
  carrierBodyCdA = value;
  carrierBodyCdASet = true;
}

void resetCarrierCdAToDefault() {
  carrierCdaPrefs.begin(CARRIER_CDA_NVS_NAMESPACE, false);
  carrierCdaPrefs.clear();
  carrierCdaPrefs.end();
  carrierBodyCdA = 0.0f;
  carrierBodyCdASet = false;
}
// ==================================================
// STATISTICAL ANALYSIS & OUTLIER REMOVAL (IQR)
// ==================================================
// forceOffsetN (typically from interpolatedCarrierOffsetN()) is subtracted
// from every sample BEFORE IQR filtering and every average/median/std/
// power/CdA calculation, so all of those numbers reflect the bike+rider's
// drag only. The original, un-offset samples are still preserved in
// result.rawForces for charting/audit -- only the statistics pipeline sees
// the corrected values.
bool calculateStatistics(const std::vector<float>& rawForces,
                         float velocityMps,
                         float airDensity,
                         float forceOffsetN,
                         TestResultData& result) {
  int n = rawForces.size();
  if (n < (int)MIN_TEST_SAMPLES) {
    result.valid = false;
    return false;
  }
  std::vector<float> correctedForces(n);
  for (int i = 0; i < n; ++i) {
    correctedForces[i] = rawForces[i] - forceOffsetN;
  }
  std::vector<float> sortedForces = correctedForces;
  std::sort(sortedForces.begin(), sortedForces.end());
  auto getMedian = [](const std::vector<float>& vec, int start, int end) -> float {
    int len = end - start + 1;
    if (len % 2 == 0) {
      return (vec[start + len / 2 - 1] + vec[start + len / 2]) / 2.0f;
    }
    return vec[start + len / 2];
  };
  float q1, q3;
  int mid = n / 2;
  if (n % 2 == 0) {
    q1 = getMedian(sortedForces, 0, mid - 1);
    q3 = getMedian(sortedForces, mid, n - 1);
  } else {
    q1 = getMedian(sortedForces, 0, mid - 1);
    q3 = getMedian(sortedForces, mid + 1, n - 1);
  }
  float iqr = q3 - q1;
  float lowerBound = q1 - 1.5f * iqr;
  float upperBound = q3 + 1.5f * iqr;
  std::vector<float> acceptedForces;
  result.acceptedMask.assign(n, 0);
  for (int i = 0; i < n; ++i) {
    float f = correctedForces[i];
    if (f >= lowerBound && f <= upperBound) {
      acceptedForces.push_back(f);
      result.acceptedMask[i] = 1;
    }
  }
  if (acceptedForces.empty()) {
    acceptedForces = correctedForces;
    std::fill(result.acceptedMask.begin(), result.acceptedMask.end(), 1);
  }
  std::vector<float> sortedAccepted = acceptedForces;
  std::sort(sortedAccepted.begin(), sortedAccepted.end());
  int accCount = acceptedForces.size();
  float sum = 0.0;
  for (float val : acceptedForces) sum += val;
  float avg = sum / accCount;
  float median = (accCount % 2 == 0) ?
                 (sortedAccepted[accCount / 2 - 1] + sortedAccepted[accCount / 2]) / 2.0f :
                 sortedAccepted[accCount / 2];
  float varianceSum = 0.0;
  for (float val : acceptedForces) {
    varianceSum += pow(val - avg, 2);
  }
  float stdDev = sqrt(varianceSum / accCount);
  float power = avg * velocityMps;
  float cdA = 0.0;
  if (velocityMps > 0.01f && airDensity > 0.01f) {
    cdA = (2.0f * avg) / (airDensity * velocityMps * velocityMps);
  }
  result.totalReadings = n;
  result.acceptedReadings = accCount;
  result.excludedReadings = n - accCount;
  result.avgForce = avg;
  result.medianForce = median;
  result.minForce = sortedAccepted.front();
  result.maxForce = sortedAccepted.back();
  result.stdDev = stdDev;
  result.aeroPower = power;
  result.cdA = cdA;
  result.cdaExcludingCarrier = cdA - carrierBodyCdA;  // carrierBodyCdA is 0 until the user sets it
  result.carrierOffsetAppliedN = forceOffsetN;
  result.valid = true;
  result.rawForces = rawForces;  // original, un-offset samples -- for charting/audit only
  return true;
}
bool processMode1Results() {
  latestResult.targetVelocityKmh = testTargetVelocityKmh;
  latestResult.velocityMps = testVelocityMps;
  latestResult.motorPercent = rampTargetPercent;
  latestResult.requestedPwmPercent = testRequiredPwmPercent;
  latestResult.airDensity = testAirDensity;
  latestResult.sampleTimesMs = recordedSampleTimesMs;
  float carrierOffset = interpolatedCarrierOffsetN((float)rampTargetPercent);
  if (!calculateStatistics(recordedRawForces, testVelocityMps,
                           testAirDensity, carrierOffset, latestResult)) {
    testErrorMessage = "Too few fresh wireless force samples. Check the sensor node and Wi-Fi link.";
    return false;
  }
  return true;
}
bool processCurrentMode2Stage() {
  TestResultData calculated;
  calculated.valid = false;
  const Mode2StageTarget& target = mode2Targets[mode2StageIndex];
  float carrierOffset = interpolatedCarrierOffsetN((float)target.motorPercent);
  if (!calculateStatistics(recordedRawForces, target.velocityMps,
                           testAirDensity, carrierOffset, calculated)) {
    testErrorMessage = "Too few fresh force samples at " +
                       String(target.velocityKmh, 0) +
                       " km/h. The incomplete stage was not calculated.";
    return false;
  }
  Mode2StageResult& stage = mode2Results[mode2StageIndex];
  stage.velocityKmh = target.velocityKmh;
  stage.velocityMps = target.velocityMps;
  stage.motorPercent = target.motorPercent;
  stage.requestedPwmPercent = target.requestedPwmPercent;
  stage.totalReadings = calculated.totalReadings;
  stage.acceptedReadings = calculated.acceptedReadings;
  stage.excludedReadings = calculated.excludedReadings;
  stage.avgForce = calculated.avgForce;
  stage.medianForce = calculated.medianForce;
  stage.minForce = calculated.minForce;
  stage.maxForce = calculated.maxForce;
  stage.stdDev = calculated.stdDev;
  stage.aeroPower = calculated.aeroPower;
  stage.cdA = calculated.cdA;
  stage.cdaExcludingCarrier = calculated.cdaExcludingCarrier;
  stage.carrierOffsetAppliedN = calculated.carrierOffsetAppliedN;
  stage.valid = true;
  stage.rawForces = recordedRawForces;
  return true;
}
void finishMode2Results(unsigned long now) {
  float forceSum = 0.0f;
  float powerSum = 0.0f;
  float cdASum = 0.0f;
  float netCdASum = 0.0f;
  int validCdACount = 0;
  int validNetCdACount = 0;
  mode2TotalAccepted = 0;
  mode2TotalExcluded = 0;
  for (size_t i = 0; i < MODE2_SPEED_COUNT; ++i) {
    const Mode2StageResult& stage = mode2Results[i];
    forceSum += stage.avgForce;
    powerSum += stage.aeroPower;
    mode2TotalAccepted += stage.acceptedReadings;
    mode2TotalExcluded += stage.excludedReadings;
    if (stage.valid && isfinite(stage.cdA)) {
      cdASum += stage.cdA;
      ++validCdACount;
    }
    if (stage.valid && isfinite(stage.cdaExcludingCarrier)) {
      netCdASum += stage.cdaExcludingCarrier;
      ++validNetCdACount;
    }
  }
  mode2AverageForce = forceSum / MODE2_SPEED_COUNT;
  mode2AveragePower = powerSum / MODE2_SPEED_COUNT;
  mode2AverageCdA = validCdACount > 0 ? cdASum / validCdACount : 0.0f;
  mode2AverageNetCdA = validNetCdACount > 0 ? netCdASum / validNetCdACount : 0.0f;
  mode2TotalDurationMs = now - testStartTime;
  mode2ResultValid = validCdACount > 0;
}
// ==================================================
// EMBEDDED WEB DASHBOARD
// ==================================================
const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Airflow Control & Aero Analysis</title>
  <style>
    :root {
      --bg: #0b1329;
      --card-bg: #17233f;
      --card-border: #2a3c63;
      --primary: #38bdf8;
      --primary-hover: #0ea5e9;
      --danger: #ef4444;
      --danger-hover: #dc2626;
      --success: #22c55e;
      --text: #f8fafc;
      --text-muted: #94a3b8;
      --accent: #6366f1;
    }
    * { box-sizing: border-box; }
    html { scroll-behavior: smooth; }
    body {
      margin: 0; padding: 16px;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: var(--bg); color: var(--text);
      display: flex; justify-content: center;
    }
    @keyframes fadeInUp {
      from { opacity: 0; transform: translateY(8px); }
      to { opacity: 1; transform: translateY(0); }
    }
    .fade-in-on-show { animation: fadeInUp 0.25s ease-out; }
    .dashboard { width: 100%; max-width: 880px; display: flex; flex-direction: column; gap: 16px; }
    .header {
      display: flex; justify-content: space-between; align-items: center;
      background: var(--card-bg); padding: 16px 20px; border-radius: 16px;
      border: 1px solid var(--card-border);
    }
    .header h1 { margin: 0; font-size: 20px; font-weight: 700; color: var(--primary); }
    .status-badge {
      padding: 6px 14px; border-radius: 20px; font-size: 13px; font-weight: 600;
      background: rgba(148, 163, 184, 0.1); border: 1px solid var(--card-border);
    }
    .calib-warning {
      display: none; background: #3b1118; border: 1px solid var(--danger); border-radius: 12px;
      padding: 12px 16px; color: #fecaca; font-size: 13px; line-height: 1.5;
    }
    .tabs { display: flex; gap: 8px; flex-wrap: wrap; }
    .tab-btn {
      flex: 1; min-width: 120px; padding: 12px; border: 1px solid var(--card-border);
      background: var(--card-bg); color: var(--text-muted);
      border-radius: 12px; font-size: 15px; font-weight: 600; cursor: pointer;
      transition: all 0.2s;
      touch-action: manipulation; -webkit-tap-highlight-color: transparent; user-select: none;
    }
    .tab-btn.active { background: var(--accent); color: #fff; border-color: var(--accent); }
    .tab-btn:disabled { opacity: 0.4; cursor: not-allowed; }
    .tab-content { display: none; }
    .tab-content.active { display: flex; flex-direction: column; gap: 16px; }
    .card {
      background: var(--card-bg); border-radius: 16px; padding: 20px;
      border: 1px solid var(--card-border); box-shadow: 0 10px 15px -3px rgba(0,0,0,0.3);
    }
    .card-title { margin: 0 0 16px 0; font-size: 16px; font-weight: 600; color: var(--text-muted); }
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    .grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px; }
    .grid-4 { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 12px; }
    @media (max-width: 600px) { .grid-2 { grid-template-columns: 1fr; } .grid-3 { grid-template-columns: 1fr; } }
    .live-display { text-align: center; padding: 12px 0; }
    .large-val { font-size: 40px; font-weight: 800; color: var(--primary); }
    .val-unit { font-size: 15px; color: var(--text-muted); margin-left: 4px; }
    .form-group { display: flex; flex-direction: column; gap: 6px; margin-bottom: 12px; }
    label { font-size: 13px; color: var(--text-muted); }
    input[type=number], input[type=range] {
      width: 100%; padding: 10px 12px; background: #0b1329; border: 1px solid var(--card-border);
      border-radius: 8px; color: var(--text); font-size: 15px; outline: none;
    }
    input[type=range] { -webkit-appearance: none; height: 8px; cursor: pointer; padding: 0; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 22px; height: 22px; border-radius: 50%; background: var(--primary); cursor: pointer; }
    input:disabled { opacity: 0.5; cursor: not-allowed; }
    button {
      padding: 12px 20px; border: none; border-radius: 10px; font-size: 15px; font-weight: 600;
      cursor: pointer; transition: all 0.2s;
      touch-action: manipulation; -webkit-tap-highlight-color: transparent; user-select: none;
    }
    button:disabled { opacity: 0.6; cursor: not-allowed; }
    .btn-primary { background: var(--primary); color: #0b1329; width: 100%; font-size: 16px; }
    .btn-primary:hover { background: var(--primary-hover); }
    .btn-danger { background: var(--danger); color: #fff; width: 100%; font-size: 16px; }
    .btn-danger:hover { background: var(--danger-hover); }
    .btn-secondary { background: #2a3c63; color: var(--text); }
    .btn-secondary:hover { background: #3b5284; }
    .btn-success { background: var(--success); color: #0b1329; }
    .btn-success:hover { background: #16a34a; }
    button:disabled { opacity: 0.4; cursor: not-allowed; }
    .test-banner {
      background: #0d2847; border: 1px solid var(--primary); border-radius: 12px;
      padding: 16px; text-align: center; margin-top: 12px; display: none;
    }
    .countdown-val { font-size: 48px; font-weight: 800; color: #fff; margin: 8px 0; }
    .stat-card {
      background: #0b1329; padding: 12px; border-radius: 10px;
      border: 1px solid var(--card-border); text-align: center;
    }
    .stat-card .title { font-size: 12px; color: var(--text-muted); }
    .stat-card .value { font-size: 18px; font-weight: 700; color: var(--text); margin-top: 4px; }
    .highlight { border-color: var(--accent); }
    .highlight .value { color: var(--primary); }
    .history-table-container { overflow-x: auto; margin-top: 12px; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; text-align: left; }
    th, td { padding: 10px; border-bottom: 1px solid var(--card-border); }
    th { color: var(--text-muted); font-weight: 600; }
    details { margin-top: 10px; border: 1px solid var(--card-border); border-radius: 8px; padding: 10px; }
    summary { cursor: pointer; font-size: 14px; font-weight: 600; color: var(--text-muted); }
    .mapping-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; }
    .mapping-item { background:#0b1329; border:1px solid var(--card-border); border-radius:10px; padding:10px; text-align:center; }
    .mapping-item .mi-label { font-size:13px; color:var(--text-muted); }
    .mapping-item .mi-value { font-size:17px; font-weight:700; color:var(--primary); margin-top:4px; }
    .velocity-readout { display:flex; gap:10px; flex-wrap:wrap; margin: 4px 0 12px; }
    .velocity-readout .pill {
      flex: 1; min-width:140px; background:#0b1329; border:1px solid var(--card-border);
      border-radius:10px; padding:10px 12px; text-align:center;
    }
    .velocity-readout .pill .p-label { font-size:12px; color:var(--text-muted); }
    .velocity-readout .pill .p-value { font-size:19px; font-weight:700; color:var(--primary); margin-top:2px; }
    .velocity-row { display:flex; gap:10px; align-items:center; }
    .velocity-row input[type=range] { flex: 3; }
    .velocity-row input[type=number] { flex: 1; min-width:80px; }
    .progress-track { width:100%; height:12px; border-radius:8px; overflow:hidden; background:#0b1329; border:1px solid var(--card-border); }
    .progress-fill { width:0%; height:100%; background:linear-gradient(90deg,var(--accent),var(--primary)); transition:width .25s; }
    .canvas-wrap { position:relative; width:100%; height:280px; margin-top:12px; }
    canvas { display:block; width:100%; height:100%; background:#0b1329; border:1px solid var(--card-border); border-radius:10px; }
    .results-table td, .results-table th { white-space:nowrap; }
    .status-note { font-size:13px; color:var(--text-muted); line-height:1.5; }
    .mode2-progress { display:none; }
    .manual-row { display:flex; gap:16px; align-items:flex-end; flex-wrap:wrap; }
    .manual-row .form-group { flex:1; min-width:200px; margin-bottom:0; }
    .history-thumb { width:64px; height:40px; object-fit:cover; border-radius:6px; border:1px solid var(--card-border); cursor:pointer; background:#0b1329; }
    .thumb-row { display:flex; gap:4px; }
    .image-modal-overlay {
      display:none; position:fixed; inset:0; background:rgba(0,0,0,0.75);
      z-index:1000; align-items:center; justify-content:center; padding:20px;
    }
    .image-modal-content {
      background:var(--card-bg); border:1px solid var(--card-border); border-radius:16px;
      padding:16px; max-width:92vw; max-height:90vh; overflow:auto;
      display:flex; flex-direction:column; gap:12px;
    }
    .image-modal-header { display:flex; justify-content:space-between; align-items:center; gap:12px; font-weight:600; }
    .image-modal-content img { max-width:100%; max-height:65vh; border-radius:8px; border:1px solid var(--card-border); background:#0b1329; }
    #calibration input[type=number] { max-width: 160px; }
    #calibration .history-table-container td { vertical-align: middle; }
    @media (max-width:600px) {
      .mapping-grid { grid-template-columns:repeat(2,1fr); }
      .canvas-wrap { height:230px; }
      .velocity-row { flex-wrap:wrap; }
      #calibration input[type=number] { max-width: 120px; }
    }
  </style>
</head>
<body>
<div class="dashboard">
  <div class="header">
    <h1>AeroDynamic Pro</h1>
    <div id="sensorBadge" class="status-badge">Connecting...</div>
  </div>
  <div id="calibBanner" class="calib-warning">
    ⚠ Air velocity calibration is incomplete or invalid. Velocity-based Mode 1 and Mode 2 tests are
    disabled until all 11 points (0% to 100%) have been calibrated and strictly increase from one
    step to the next. Manual motor control still works.
    <div><button class="btn-secondary" style="margin-top:10px;" onclick="switchTab(3)">Go to Air Velocity Calibration</button></div>
  </div>
  <button id="globalStopBtn" class="btn-danger" onclick="stopAllMotors()">🛑 STOP ALL MOTORS</button>
  <div class="card">
    <h3 class="card-title">Manual Motor Control</h3>
    <p class="status-note">Spin the motors directly at any PWM percentage — useful for rigging the bike or feeling out airflow. The estimated air velocity below comes from the calibration table, not a live measurement. Disabled while an automatic test is running.</p>
    <div class="manual-row">
      <div class="form-group">
        <label>Manual Motor Speed: <span id="manualSpeedLabel" style="font-weight:bold;color:var(--primary);">0%</span></label>
        <input id="manualSpeedSlider" type="range" min="0" max="100" value="0" oninput="onManualSliderInput(this.value)">
      </div>
      <button id="setManualSpeedBtn" class="btn-primary" style="width:auto;padding:12px 24px;" onclick="setManualSpeed(this)">Set Speed</button>
      <button id="manualStopBtn" class="btn-secondary" onclick="resetManualSpeed(this)">Motors Off</button>
    </div>
    <div id="manualEstimate" class="status-note" style="margin-top:8px;">Estimated air velocity: -- km/h (-- m/s)</div>
    <div id="manualSpeedMessage" aria-live="polite" style="font-size:13px;color:var(--text-muted);margin-top:8px;"></div>
  </div>
  <div class="tabs">
    <button class="tab-btn active" id="tabBtn1" onclick="switchTab(1)">Mode 1: Auto Airflow Test</button>
    <button class="tab-btn" id="tabBtn2" onclick="switchTab(2)">Mode 2</button>
    <button class="tab-btn" id="tabBtn3" onclick="switchTab(3)">Air Velocity Calibration</button>
  </div>
  <div id="mode1" class="tab-content active">
    <div class="grid-2">
      <div class="card">
        <h3 class="card-title">Test Configuration</h3>
        <div class="form-group">
          <label>Target Air Velocity: <span id="velocityKmhLabel" style="font-weight:bold;color:var(--primary);">10.0 km/h</span></label>
          <div class="velocity-row">
            <input id="inputVelocityRange" type="range" min="0" max="37" step="0.1" value="10" oninput="onVelocityInput('range')">
            <input id="inputVelocityNumber" type="number" inputmode="decimal" min="0" max="37" step="0.1" value="10.0" oninput="onVelocityInput('number')">
          </div>
        </div>
        <div class="velocity-readout">
          <div class="pill"><div class="p-label">Velocity (m/s)</div><div id="velocityMpsDisplay" class="p-value">--</div></div>
          <div class="pill"><div class="p-label">Calculated PWM (diagnostic)</div><div id="velocityPwmDisplay" class="p-value">--</div></div>
        </div>
        <div class="form-group">
          <label>Air Density (kg/m³)</label>
          <input id="inputDensity" type="number" inputmode="decimal" step="0.001" value="1.225">
        </div>
        <button id="startTestBtn" class="btn-primary" onclick="startAutomaticTest(this)">START TEST</button>
      </div>
      <div class="card">
        <h3 class="card-title">Live Dynamic Readings</h3>
        <div class="grid-3">
          <div class="live-display">
            <span id="liveSpeed" class="large-val">0</span><span class="val-unit">%</span>
            <div style="font-size:12px;color:var(--text-muted);">Live Motor PWM</div>
          </div>
          <div class="live-display">
            <span id="liveForce" class="large-val">--</span><span class="val-unit">N</span>
            <div style="font-size:12px;color:var(--text-muted);">Normal Force</div>
          </div>
          <div class="live-display">
            <span id="liveEstVelocity" class="large-val">--</span><span class="val-unit">km/h</span>
            <div style="font-size:12px;color:var(--text-muted);">Estimated Air Velocity</div>
          </div>
        </div>
        <div id="liveDiagnostics" class="status-note" style="text-align:center;margin-top:4px;">
          Net Force (offset-corrected): <strong id="liveNetForce">--</strong> N
          &nbsp;·&nbsp; Live carrier offset: <strong id="liveCarrierOffsetDisplay">--</strong> N
          &nbsp;·&nbsp; <span id="liveForceAge">sensor updated -- ms ago</span>
        </div>
        <div id="testBanner" class="test-banner">
          <div id="testPhaseLabel" style="font-size:14px;color:var(--primary);text-transform:uppercase;font-weight:bold;">Phase</div>
          <div class="grid-4" style="margin-top:12px;">
            <div class="stat-card"><div class="title">Target Velocity</div><div id="tbTargetVelocity" class="value">--</div></div>
            <div class="stat-card"><div class="title">Estimated Velocity</div><div id="tbEstVelocity" class="value">--</div></div>
            <div class="stat-card"><div class="title">Target PWM</div><div id="tbTargetPwm" class="value">--</div></div>
            <div class="stat-card"><div class="title">Live PWM</div><div id="tbLivePwm" class="value">--</div></div>
          </div>
          <div id="countdownDisplay" class="countdown-val">0</div>
          <div id="testPhaseInstruction" style="font-size:13px;color:var(--text);">Instruction text</div>
        </div>
      </div>
    </div>
    <div class="card" id="resultsSection" style="display:none; border: 2px solid var(--primary);">
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:16px;">
        <h3 class="card-title" style="margin:0; color:var(--primary);">🎉 Test Completed - Results Analysis</h3>
        <span id="resTimestamp" style="font-size:12px;color:var(--text-muted);"></span>
      </div>
      <div class="grid-4">
        <div class="stat-card highlight"><div class="title">CdA</div><div id="resCdA" class="value">-- m²</div></div>
        <div class="stat-card highlight"><div class="title">Net CdA (excl. Carrier)</div><div id="resNetCdA" class="value">-- m²</div></div>
        <div class="stat-card highlight"><div class="title">Aero Power</div><div id="resPower" class="value">-- W</div></div>
        <div class="stat-card"><div class="title">Target Velocity</div><div id="resTargetVelocity" class="value">--</div></div>
        <div class="stat-card"><div class="title">Applied PWM</div><div id="resPwm" class="value">--</div></div>
        <div class="stat-card"><div class="title">Average Force</div><div id="resAvgForce" class="value">-- N</div></div>
        <div class="stat-card"><div class="title">Median Force</div><div id="resMedForce" class="value">-- N</div></div>
        <div class="stat-card"><div class="title">Std Deviation (σ)</div><div id="resStdDev" class="value">-- N</div></div>
        <div class="stat-card"><div class="title">Min / Max Force</div><div id="resMinMax" class="value">-- N</div></div>
        <div class="stat-card"><div class="title">Accepted / Excluded</div><div id="resPoints" class="value">-- / --</div></div>
        <div class="stat-card"><div class="title">Carrier Offset Subtracted</div><div id="resCarrierOffset" class="value">-- N</div></div>
      </div>
      <h3 class="card-title" style="margin-top:20px;">Force versus measurement time</h3>
      <div class="status-note">Grey line: raw samples · Cyan points: accepted samples · Red points: excluded IQR outliers</div>
      <div class="canvas-wrap"><canvas id="mode1ForceChart"></canvas></div>
      <div style="display:flex; gap:10px; margin-top:16px; flex-wrap:wrap;">
        <button id="saveBtn" class="btn-success" style="flex:2;" onclick="saveCurrentResult()">💾 Save Result to History</button>
        <button class="btn-secondary" style="flex:1;" onclick="downloadDataUrl(canvasToDataUrl('mode1ForceChart'), 'mode1_force_chart_' + Date.now() + '.png')">⬇ Download Chart</button>
        <button class="btn-secondary" style="flex:1;" onclick="startNewTestClean()">Start New Test</button>
      </div>
    </div>
    <div class="card">
      <details>
        <summary>Sensor Zero & Calibration Tools</summary>
        <div style="margin-top:16px; display:flex; flex-direction:column; gap:12px;">
          <button id="tareBtn" class="btn-secondary" onclick="tareSensor(this)">Tare (Set Zero)</button>
          <div class="form-group">
            <label>Known Calibration Mass (kg)</label>
            <input id="calMass" type="number" inputmode="decimal" step="0.001" placeholder="e.g. 1.000">
          </div>
          <button id="calibrateMassBtn" class="btn-secondary" onclick="calibrateSensor(this)">Calibrate with Mass</button>
          <div id="calMessage" aria-live="polite" style="font-size:13px; color:var(--primary);"></div>
        </div>
      </details>
    </div>
  </div>
  <div id="mode2" class="tab-content">
    <div class="card">
      <h3 class="card-title">Mode 2: Automatic Speed Sweep</h3>
      <p class="status-note">The six target air speeds run automatically. The PWM required for each is calculated from the calibration table — there is nothing to map manually. Values below are a diagnostic preview.</p>
      <div class="form-group">
        <label>Air Density (kg/m³)</label>
        <input id="mode2Density" class="test-control" type="number" inputmode="decimal" min="0.1" step="0.001" value="1.225">
      </div>
      <h3 class="card-title" style="margin-top:18px;">Calculated PWM preview (from calibration table)</h3>
      <div id="mode2PreviewGrid" class="mapping-grid">
        <div class="mapping-item"><div class="mi-label">10 km/h</div><div class="mi-value" data-preview-index="0">--%</div></div>
        <div class="mapping-item"><div class="mi-label">15 km/h</div><div class="mi-value" data-preview-index="1">--%</div></div>
        <div class="mapping-item"><div class="mi-label">20 km/h</div><div class="mi-value" data-preview-index="2">--%</div></div>
        <div class="mapping-item"><div class="mi-label">25 km/h</div><div class="mi-value" data-preview-index="3">--%</div></div>
        <div class="mapping-item"><div class="mi-label">30 km/h</div><div class="mi-value" data-preview-index="4">--%</div></div>
        <div class="mapping-item"><div class="mi-label">35 km/h</div><div class="mi-value" data-preview-index="5">--%</div></div>
      </div>
      <button id="startMode2Btn" class="btn-primary" style="margin-top:18px;" onclick="startMode2Test(this)">START MODE 2 TEST</button>
    </div>
    <div id="mode2ProgressCard" class="card mode2-progress">
      <h3 class="card-title">Automatic Sweep Progress</h3>
      <div class="grid-4">
        <div class="stat-card highlight"><div class="title">Target Velocity</div><div id="m2Target" class="value">--</div></div>
        <div class="stat-card"><div class="title">Target (m/s)</div><div id="m2Velocity" class="value">--</div></div>
        <div class="stat-card"><div class="title">Target PWM</div><div id="m2Pwm" class="value">--</div></div>
        <div class="stat-card"><div class="title">Estimated Velocity</div><div id="m2EstVelocity" class="value">--</div></div>
        <div class="stat-card"><div class="title">Sequence</div><div id="m2Position" class="value">--</div></div>
        <div class="stat-card"><div class="title">Stage</div><div id="m2Stage" class="value">--</div></div>
        <div class="stat-card"><div class="title">Next Target</div><div id="m2Next" class="value">--</div></div>
        <div class="stat-card"><div class="title">Live PWM</div><div id="m2LiveSpeed" class="value">0%</div></div>
        <div class="stat-card"><div class="title">Live Force</div><div id="m2LiveForce" class="value">-- N</div></div>
      </div>
      <div style="text-align:center;margin:18px 0 12px;">
        <div id="m2Instruction" class="status-note">Waiting</div>
        <div id="m2Countdown" class="countdown-val">--</div>
      </div>
      <div class="progress-track"><div id="m2ProgressFill" class="progress-fill"></div></div>
      <div id="m2ProgressText" class="status-note" style="text-align:center;margin-top:6px;">0%</div>
    </div>
    <div id="mode2Results" class="card" style="display:none;border:2px solid var(--primary);">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:16px;">
        <h3 class="card-title" style="margin:0;color:var(--primary);">Mode 2 Completed — Sweep Results</h3>
        <span id="m2Timestamp" style="font-size:12px;color:var(--text-muted);"></span>
      </div>
      <div class="grid-4">
        <div class="stat-card highlight"><div class="title">Average CdA</div><div id="m2AvgCdA" class="value">-- m²</div></div>
        <div class="stat-card highlight"><div class="title">Average Net CdA</div><div id="m2AvgNetCdA" class="value">-- m²</div></div>
        <div class="stat-card"><div class="title">Average Force</div><div id="m2AvgForce" class="value">-- N</div></div>
        <div class="stat-card"><div class="title">Average Power</div><div id="m2AvgPower" class="value">-- W</div></div>
        <div class="stat-card"><div class="title">Total Duration</div><div id="m2Duration" class="value">-- s</div></div>
        <div class="stat-card"><div class="title">Accepted Samples</div><div id="m2Accepted" class="value">--</div></div>
        <div class="stat-card"><div class="title">Excluded Samples</div><div id="m2Excluded" class="value">--</div></div>
      </div>
      <div class="history-table-container">
        <table class="results-table">
          <thead><tr><th>km/h</th><th>m/s</th><th>PWM</th><th>Total</th><th>Accepted</th><th>Excluded</th><th>Avg N</th><th>Median N</th><th>Min / Max N</th><th>Std N</th><th>Power W</th><th>Offset N</th><th>CdA</th><th>Net CdA</th></tr></thead>
          <tbody id="mode2ResultsBody"></tbody>
        </table>
      </div>
      <h3 class="card-title" style="margin-top:20px;">Average Force versus Airflow Speed</h3>
      <div class="canvas-wrap"><canvas id="mode2ForceChart"></canvas></div>
      <h3 class="card-title" style="margin-top:20px;">Aerodynamic Power versus Airflow Speed</h3>
      <div class="canvas-wrap"><canvas id="mode2PowerChart"></canvas></div>
      <div style="display:flex;gap:10px;margin-top:16px;flex-wrap:wrap;">
        <button id="saveMode2Btn" class="btn-success" style="flex:2;" onclick="saveCurrentResult()">💾 Save Result to History</button>
        <button class="btn-secondary" style="flex:1;" onclick="downloadDataUrl(canvasToDataUrl('mode2ForceChart'), 'mode2_force_chart_' + Date.now() + '.png')">⬇ Force Chart</button>
        <button class="btn-secondary" style="flex:1;" onclick="downloadDataUrl(canvasToDataUrl('mode2PowerChart'), 'mode2_power_chart_' + Date.now() + '.png')">⬇ Power Chart</button>
        <button class="btn-secondary" style="flex:1;" onclick="startNewMode2TestClean()">Start New Sweep</button>
      </div>
    </div>
  </div>
  <div id="calibration" class="tab-content">
    <div class="card">
      <h3 class="card-title">Air Velocity Calibration</h3>
      <p class="status-note">
        Stand next to the rig with an anemometer. For each row below: press SET to run the motors at
        that PWM, read the measured air velocity, type it into the box, then press SAVE CALIBRATION
        (you can save after every point or once at the end — each save only updates the rows you have
        filled in and stores them permanently on the ESP32, so the values survive a page refresh, a
        Wi-Fi reconnect, and an ESP32 restart).
      </p>
      <div class="velocity-readout">
        <div class="pill"><div class="p-label">Currently Running</div><div id="calibRunningPwm" class="p-value">Motors off</div></div>
        <div class="pill"><div class="p-label">Live Force</div><div id="calibLiveForce" class="p-value">-- N</div></div>
      </div>
      <div class="history-table-container">
        <table>
          <thead><tr><th>Motor PWM</th><th>Measured Air Velocity (km/h)</th><th></th></tr></thead>
          <tbody>
            <tr>
              <td>0%</td>
              <td><input id="calibInput0" type="number" inputmode="decimal" step="0.01" min="0" placeholder="0.00" oninput="onCalibInputChange(0, this.value)"></td>
              <td><span class="status-note">Motors off (fixed baseline)</span></td>
            </tr>
            <tr>
              <td>10%</td>
              <td><input id="calibInput10" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(10, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(10, this)">SET 10%</button></td>
            </tr>
            <tr>
              <td>20%</td>
              <td><input id="calibInput20" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(20, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(20, this)">SET 20%</button></td>
            </tr>
            <tr>
              <td>30%</td>
              <td><input id="calibInput30" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(30, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(30, this)">SET 30%</button></td>
            </tr>
            <tr>
              <td>40%</td>
              <td><input id="calibInput40" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(40, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(40, this)">SET 40%</button></td>
            </tr>
            <tr>
              <td>50%</td>
              <td><input id="calibInput50" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(50, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(50, this)">SET 50%</button></td>
            </tr>
            <tr>
              <td>60%</td>
              <td><input id="calibInput60" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(60, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(60, this)">SET 60%</button></td>
            </tr>
            <tr>
              <td>70%</td>
              <td><input id="calibInput70" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(70, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(70, this)">SET 70%</button></td>
            </tr>
            <tr>
              <td>80%</td>
              <td><input id="calibInput80" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(80, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(80, this)">SET 80%</button></td>
            </tr>
            <tr>
              <td>90%</td>
              <td><input id="calibInput90" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(90, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(90, this)">SET 90%</button></td>
            </tr>
            <tr>
              <td>100%</td>
              <td><input id="calibInput100" type="number" inputmode="decimal" step="0.01" min="0" placeholder="enter measured value" oninput="onCalibInputChange(100, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(100, this)">SET 100%</button></td>
            </tr>
          </tbody>
        </table>
      </div>
      <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:16px;">
        <button id="calibStopBtn" class="btn-danger" style="flex:1;min-width:140px;" onclick="stopCalibrationMotors(this)">STOP MOTORS</button>
        <button id="calibValidateBtn" class="btn-secondary" style="flex:1;min-width:140px;" onclick="refreshCalibrationStatus(this)">VALIDATE CALIBRATION</button>
        <button id="calibSaveBtn" class="btn-success" style="flex:1;min-width:140px;" onclick="saveCalibration(this)">SAVE CALIBRATION</button>
        <button id="calibDownloadBtn" class="btn-secondary" style="flex:1;min-width:140px;" onclick="downloadCalibrationCsv()">DOWNLOAD CSV</button>
        <button id="calibResetBtn" class="btn-secondary" style="flex:1;min-width:140px;color:var(--danger);" onclick="resetCalibration(this)">RESET CALIBRATION</button>
      </div>
      <div id="calibSaveMessage" aria-live="polite" style="font-size:13px;color:var(--text-muted);margin-top:10px;"></div>
      <div id="calibStatusText" aria-live="polite" style="font-size:15px;font-weight:700;margin-top:10px;">Calibration Status: --</div>
      <h3 class="card-title" style="margin-top:20px;">Air Velocity Calibration Curve</h3>
      <div class="canvas-wrap"><canvas id="calibrationChart"></canvas></div>
    </div>
    <div class="card">
      <h3 class="card-title">Carrier Body Offset <span style="font-weight:400;color:var(--text-muted);">(optional)</span></h3>
      <p class="status-note">
        Remove the bike so only the mount/stand/fixture that carries it sits in the airflow. For each
        PWM step, press SET, let the reading settle, and enter the force shown by the load cell. That
        value is then subtracted from every raw force sample at that PWM before Mode 1/Mode 2 compute
        CdA and power, so the carrier's own drag is never attributed to the bike. This table is
        entirely optional — any point left blank is simply treated as zero offset, so an uncalibrated
        rig behaves exactly as it always has.
      </p>
      <div class="history-table-container">
        <table>
          <thead><tr><th>Motor PWM</th><th>Measured Carrier Offset (N)</th><th></th></tr></thead>
          <tbody>
            <tr>
              <td>0%</td>
              <td><input id="carrierInput0" type="number" inputmode="decimal" step="0.001" min="0" placeholder="0.000" oninput="onCarrierInputChange(0, this.value)"></td>
              <td><span class="status-note">Motors off</span></td>
            </tr>
            <tr>
              <td>10%</td>
              <td><input id="carrierInput10" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(10, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(10, this)">SET 10%</button></td>
            </tr>
            <tr>
              <td>20%</td>
              <td><input id="carrierInput20" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(20, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(20, this)">SET 20%</button></td>
            </tr>
            <tr>
              <td>30%</td>
              <td><input id="carrierInput30" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(30, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(30, this)">SET 30%</button></td>
            </tr>
            <tr>
              <td>40%</td>
              <td><input id="carrierInput40" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(40, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(40, this)">SET 40%</button></td>
            </tr>
            <tr>
              <td>50%</td>
              <td><input id="carrierInput50" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(50, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(50, this)">SET 50%</button></td>
            </tr>
            <tr>
              <td>60%</td>
              <td><input id="carrierInput60" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(60, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(60, this)">SET 60%</button></td>
            </tr>
            <tr>
              <td>70%</td>
              <td><input id="carrierInput70" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(70, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(70, this)">SET 70%</button></td>
            </tr>
            <tr>
              <td>80%</td>
              <td><input id="carrierInput80" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(80, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(80, this)">SET 80%</button></td>
            </tr>
            <tr>
              <td>90%</td>
              <td><input id="carrierInput90" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(90, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(90, this)">SET 90%</button></td>
            </tr>
            <tr>
              <td>100%</td>
              <td><input id="carrierInput100" type="number" inputmode="decimal" step="0.001" min="0" placeholder="enter measured value" oninput="onCarrierInputChange(100, this.value)"></td>
              <td><button class="btn-secondary calib-set-btn" onclick="setCalibrationPwm(100, this)">SET 100%</button></td>
            </tr>
          </tbody>
        </table>
      </div>
      <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:16px;">
        <button id="carrierStopBtn" class="btn-danger" style="flex:1;min-width:140px;" onclick="stopCalibrationMotors(this)">STOP MOTORS</button>
        <button id="carrierSaveBtn" class="btn-success" style="flex:1;min-width:140px;" onclick="saveCarrierOffset(this)">SAVE CARRIER OFFSET</button>
        <button id="carrierResetBtn" class="btn-secondary" style="flex:1;min-width:140px;color:var(--danger);" onclick="resetCarrierOffset(this)">RESET CARRIER OFFSET</button>
      </div>
      <div id="carrierSaveMessage" aria-live="polite" style="font-size:13px;color:var(--text-muted);margin-top:10px;"></div>
      <div id="carrierStatusText" style="font-size:15px;font-weight:700;margin-top:10px;">Carrier offset calibrated at 0 of 11 points</div>
      <h3 class="card-title" style="margin-top:20px;">Carrier Offset Curve</h3>
      <div class="canvas-wrap"><canvas id="carrierOffsetChart"></canvas></div>
      <h3 class="card-title" style="margin-top:24px;">Carrier Body CdA <span style="font-weight:400;color:var(--text-muted);">(constant, not speed-dependent)</span></h3>
      <p class="status-note">
        If you already know the carrier's own drag coefficient×area (CdA) — from a separate test or a
        published figure — enter it here instead of (or alongside) the force offset above. Because CdA
        is a property of the shape rather than the airspeed, a single value is subtracted from the
        final computed CdA of every Mode 1/Mode 2 result to give a bike+rider-only <strong>Net CdA</strong>.
        Leave it at 0 to make no change.
      </p>
      <div class="manual-row">
        <div class="form-group">
          <label>Carrier Body CdA (m²)</label>
          <input id="carrierCdaInput" type="number" inputmode="decimal" step="0.0001" min="0" placeholder="0.0000" oninput="onCarrierCdaInputChange(this.value)">
        </div>
        <button id="carrierCdaSaveBtn" class="btn-success" style="width:auto;padding:12px 24px;" onclick="saveCarrierCda(this)">Save CdA</button>
        <button id="carrierCdaResetBtn" class="btn-secondary" onclick="resetCarrierCda(this)">Reset</button>
      </div>
      <div id="carrierCdaMessage" aria-live="polite" style="font-size:13px;color:var(--text-muted);margin-top:8px;"></div>
      <div id="carrierCdaStatusText" class="status-note" style="margin-top:4px;">Carrier CdA offset: not set (0.0000 m², no effect)</div>
    </div>
  </div>
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;gap:8px;flex-wrap:wrap;">
      <h3 class="card-title" style="margin:0;">Saved Results — Both Modes</h3>
      <div style="display:flex;gap:8px;">
        <button class="btn-secondary" style="padding:6px 12px;font-size:13px;" onclick="exportCSV()">⬇ Export CSV</button>
        <button class="btn-secondary" style="padding:6px 12px;font-size:13px;color:var(--danger);" onclick="clearHistory()">Clear All</button>
      </div>
    </div>
    <div class="history-table-container">
      <table><thead><tr><th>Mode</th><th>Date & Time</th><th>Setup</th><th>Avg Force</th><th>Avg Power</th><th>CdA</th><th>Samples</th><th>Status</th><th>Graph</th><th>Action</th></tr></thead>
      <tbody id="combinedHistoryBody"></tbody></table>
    </div>
  </div>
</div>
<div id="imageModalOverlay" class="image-modal-overlay" onclick="if(event.target===this) closeImageModal()">
  <div class="image-modal-content">
    <div class="image-modal-header">
      <span id="imageModalTitle">Graph</span>
      <button class="btn-secondary" onclick="closeImageModal()">✕ Close</button>
    </div>
    <img id="imageModalImg" src="" alt="Saved chart">
    <button id="imageModalDownload" class="btn-success">⬇ Download PNG</button>
  </div>
</div>
<script>
  let testInProgress = false;
  let currentResultObject = null;
  let resultProcessedForThisRun = false;
  let mode2ProcessedForThisRun = false;
  let currentActiveTab = 1;
  let calibrationModeActiveClient = false;
  // --- UX helpers: busy-state buttons + auto-clearing status messages ---
  // Wraps an async action so its triggering button is disabled and shows a
  // busy label while the action runs, then always restores it (even on
  // error/exception). btn may be null/undefined (e.g. a programmatic call
  // with no originating click) — in that case the task just runs normally.
  async function withBusy(btn, busyLabel, taskFn) {
    let originalText = null;
    let originalDisabled = false;
    if (btn) {
      originalText = btn.innerText;
      originalDisabled = btn.disabled;
      btn.disabled = true;
      if (busyLabel) btn.innerText = busyLabel;
    }
    try {
      return await taskFn();
    } finally {
      if (btn) {
        btn.innerText = originalText;
        btn.disabled = originalDisabled;
      }
    }
  }
  // Shows a short status message in the given element, then clears it after
  // durationMs (default 4000). Pass durationMs = 0 for an in-progress message
  // that a following call will immediately overwrite (e.g. "Saving..."),
  // so it doesn't get auto-cleared out from under the real result.
  // Shows a block-level element and (re)plays its fade-in-up entrance
  // animation, even if the element was already visible before (forces a
  // reflow so the CSS animation restarts on every reveal, not just the first).
  function revealWithFadeIn(elementId, displayValue) {
    const el = document.getElementById(elementId);
    if (!el) return;
    el.style.display = displayValue || 'block';
    el.classList.remove('fade-in-on-show');
    void el.offsetWidth; // force reflow so the animation class re-triggers
    el.classList.add('fade-in-on-show');
  }
  const transientMessageTimers = {};
  function showTransientMessage(elementId, text, durationMs) {
    const el = document.getElementById(elementId);
    if (!el) return;
    el.innerText = text;
    if (transientMessageTimers[elementId]) {
      clearTimeout(transientMessageTimers[elementId]);
      delete transientMessageTimers[elementId];
    }
    const delay = (durationMs === 0) ? 0 : (durationMs || 4000);
    if (delay > 0 && text) {
      transientMessageTimers[elementId] = setTimeout(function() {
        if (el.innerText === text) el.innerText = '';
        delete transientMessageTimers[elementId];
      }, delay);
    }
  }
  // Fixed Mode 2 sweep points (must match MODE2_TARGET_VELOCITIES_KMH in the firmware).
  const MODE2_VELOCITIES_KMH = [10, 15, 20, 25, 30, 35];
  // Local cache of the calibration table, fetched once from the ESP32 so the
  // browser never needs its own hand-maintained copy. All interpolation here
  // mirrors velocityToMotorPercent()/motorPercentToVelocity() in the firmware.
  let velocityTable = [];
  let tableMinKmh = 0;
  let tableMaxKmh = 37;
  let tableValid = false;
  async function loadVelocityTable() {
    try {
      const res = await fetch('/velocity_table', { cache: 'no-store' });
      const data = await res.json();
      velocityTable = Array.isArray(data.table) ? data.table : [];
      tableMinKmh = Number(data.minVelocityKmh);
      tableMaxKmh = Number(data.maxVelocityKmh);
      tableValid = !!data.valid && velocityTable.length >= 2;
    } catch (e) {
      tableValid = false;
    }
    applyTableToUI();
  }
  function velocityToPwmLocal(kmh) {
    if (!tableValid || !Number.isFinite(kmh)) return NaN;
    if (kmh < tableMinKmh || kmh > tableMaxKmh) return NaN;
    for (let i = 0; i < velocityTable.length - 1; i++) {
      const lo = velocityTable[i], hi = velocityTable[i + 1];
      if (kmh >= lo.kmh && kmh <= hi.kmh) {
        if (kmh === lo.kmh) return lo.pwm;
        if (kmh === hi.kmh) return hi.pwm;
        return lo.pwm + ((kmh - lo.kmh) / (hi.kmh - lo.kmh)) * (hi.pwm - lo.pwm);
      }
    }
    return NaN;
  }
  function pwmToVelocityLocal(pwm) {
    if (!tableValid || !Number.isFinite(pwm)) return NaN;
    const minP = velocityTable[0].pwm, maxP = velocityTable[velocityTable.length - 1].pwm;
    if (pwm < minP || pwm > maxP) return NaN;
    for (let i = 0; i < velocityTable.length - 1; i++) {
      const lo = velocityTable[i], hi = velocityTable[i + 1];
      if (pwm >= lo.pwm && pwm <= hi.pwm) {
        if (pwm === lo.pwm) return lo.kmh;
        if (pwm === hi.pwm) return hi.kmh;
        return lo.kmh + ((pwm - lo.pwm) / (hi.pwm - lo.pwm)) * (hi.kmh - lo.kmh);
      }
    }
    return NaN;
  }
  function applyTableToUI() {
    const calibBanner = document.getElementById('calibBanner');
    calibBanner.style.display = tableValid ? 'none' : 'block';
    const rangeInput = document.getElementById('inputVelocityRange');
    const numberInput = document.getElementById('inputVelocityNumber');
    if (tableValid) {
      rangeInput.min = tableMinKmh; rangeInput.max = tableMaxKmh;
      numberInput.min = tableMinKmh; numberInput.max = tableMaxKmh;
      let clamped = Math.min(Math.max(Number(numberInput.value) || tableMinKmh, tableMinKmh), tableMaxKmh);
      numberInput.value = clamped.toFixed(1);
      rangeInput.value = clamped;
    }
    onVelocityInput('range');
    renderMode2Preview();
  }
  function renderMode2Preview() {
    document.querySelectorAll('#mode2PreviewGrid [data-preview-index]').forEach(function(el) {
      const idx = Number(el.getAttribute('data-preview-index'));
      const kmh = MODE2_VELOCITIES_KMH[idx];
      const pwm = velocityToPwmLocal(kmh);
      el.innerText = Number.isFinite(pwm) ? pwm.toFixed(1) + '%' : '--%';
    });
  }
  function switchTab(mode) {
    if (testInProgress) return;
    if (currentActiveTab === 3 && mode !== 3) {
      stopCalibrationMotors();
    }
    currentActiveTab = mode;
    document.getElementById('tabBtn1').classList.toggle('active', mode === 1);
    document.getElementById('tabBtn2').classList.toggle('active', mode === 2);
    document.getElementById('tabBtn3').classList.toggle('active', mode === 3);
    document.getElementById('mode1').classList.toggle('active', mode === 1);
    document.getElementById('mode2').classList.toggle('active', mode === 2);
    document.getElementById('calibration').classList.toggle('active', mode === 3);
  }
  async function stopAllMotors() {
    try {
      await fetch('/stop', { method: 'POST' });
    } catch(e) { console.error(e); }
  }
  function onManualSliderInput(value) {
    document.getElementById('manualSpeedLabel').innerText = value + '%';
    const est = pwmToVelocityLocal(Number(value));
    document.getElementById('manualEstimate').innerText = Number.isFinite(est)
      ? 'Estimated air velocity: ' + est.toFixed(2) + ' km/h (' + (est / 3.6).toFixed(2) + ' m/s)'
      : 'Estimated air velocity: -- km/h (-- m/s)';
  }
  async function setManualSpeed(btn) {
    await withBusy(btn, 'Setting...', async function() {
      const speed = document.getElementById('manualSpeedSlider').value;
      showTransientMessage('manualSpeedMessage', 'Setting speed...', 0);
      try {
        const res = await fetch(`/manual_speed?speed=${speed}`, { method: 'POST' });
        const txt = await res.text();
        showTransientMessage('manualSpeedMessage', txt);
      } catch(e) {
        showTransientMessage('manualSpeedMessage', 'Failed to set motor speed.');
      }
    });
  }
  async function resetManualSpeed(btn) {
    document.getElementById('manualSpeedSlider').value = 0;
    onManualSliderInput(0);
    await setManualSpeed(btn);
  }
  function onVelocityInput(source) {
    const rangeInput = document.getElementById('inputVelocityRange');
    const numberInput = document.getElementById('inputVelocityNumber');
    let kmh = Number(source === 'number' ? numberInput.value : rangeInput.value);
    if (!Number.isFinite(kmh)) kmh = 0;
    kmh = Math.min(Math.max(kmh, tableMinKmh), tableMaxKmh);
    if (source === 'number') {
      rangeInput.value = kmh;
    } else {
      numberInput.value = kmh.toFixed(1);
    }
    document.getElementById('velocityKmhLabel').innerText = kmh.toFixed(1) + ' km/h';
    document.getElementById('velocityMpsDisplay').innerText = (kmh / 3.6).toFixed(2) + ' m/s';
    const pwm = velocityToPwmLocal(kmh);
    document.getElementById('velocityPwmDisplay').innerText = Number.isFinite(pwm) ? pwm.toFixed(1) + '%' : '--';
  }
  async function startAutomaticTest(btn) {
    const velocity = Number(document.getElementById('inputVelocityNumber').value);
    const density = document.getElementById('inputDensity').value;
    if (!tableValid) {
      alert('The calibration table is invalid — velocity-based tests are disabled.');
      return;
    }
    if (!Number.isFinite(velocity) || velocity < tableMinKmh || velocity > tableMaxKmh) {
      alert('Please enter a target air velocity between ' + tableMinKmh.toFixed(1) + ' and ' + tableMaxKmh.toFixed(1) + ' km/h.');
      return;
    }
    await withBusy(btn, 'Starting...', async function() {
      resultProcessedForThisRun = false;
      document.getElementById('resultsSection').style.display = 'none';
      try {
        const res = await fetch(`/start_test?velocity=${velocity}&density=${density}`, { method: 'POST' });
        const txt = await res.text();
        if (!res.ok) alert(txt);
      } catch (e) {
        alert('Failed to initiate test sequence.');
      }
    });
  }
  function startNewTestClean() {
    document.getElementById('resultsSection').style.display = 'none';
  }
  function canvasToDataUrl(canvasId) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return null;
    try { return canvas.toDataURL('image/png'); } catch(e) { return null; }
  }
  function downloadDataUrl(dataUrl, filename) {
    if (!dataUrl) { alert('No chart available to download yet.'); return; }
    const link = document.createElement('a');
    link.href = dataUrl;
    link.download = filename;
    link.click();
  }
  function openImageModal(dataUrl, title) {
    if (!dataUrl) return;
    document.getElementById('imageModalImg').src = dataUrl;
    document.getElementById('imageModalTitle').innerText = title || 'Graph';
    document.getElementById('imageModalDownload').onclick = function() {
      downloadDataUrl(dataUrl, (title || 'graph').replace(/\s+/g, '_') + '.png');
    };
    document.getElementById('imageModalOverlay').style.display = 'flex';
  }
  function closeImageModal() {
    document.getElementById('imageModalOverlay').style.display = 'none';
  }
  function saveCurrentResult() {
    if (!currentResultObject) return;
    if (currentResultObject.mode === 'Mode 2') {
      currentResultObject.chartImages = {
        force: canvasToDataUrl('mode2ForceChart'),
        power: canvasToDataUrl('mode2PowerChart')
      };
    } else {
      currentResultObject.chartImage = canvasToDataUrl('mode1ForceChart');
    }
    const history = getHistory();
    history.unshift(currentResultObject);
    try {
      localStorage.setItem('airflow_results_log', JSON.stringify(history));
    } catch(e) {
      alert('Could not save: browser storage is full. Export or clear old history, then try again.');
      return;
    }
    const saveBtn = document.getElementById(
      currentResultObject.mode === 'Mode 2' ? 'saveMode2Btn' : 'saveBtn');
    saveBtn.innerText = "✓ Saved Successfully";
    saveBtn.disabled = true;
    renderHistory();
  }
  function clearHistory() {
    if (confirm('Are you sure you want to delete all saved test results?')) {
      localStorage.removeItem('airflow_results_log');
      renderHistory();
    }
  }
  async function tareSensor(btn) {
    await withBusy(btn, 'Zeroing...', async function() {
      showTransientMessage('calMessage', 'Setting zero...', 0);
      try {
        const res = await fetch('/tare', { method: 'POST' });
        showTransientMessage('calMessage', await res.text());
      } catch(e) { showTransientMessage('calMessage', 'Tare failed.'); }
    });
  }
  async function calibrateSensor(btn) {
    const mass = document.getElementById('calMass').value;
    if (!mass || mass <= 0) return alert('Enter a valid mass in kg.');
    await withBusy(btn, 'Calibrating...', async function() {
      showTransientMessage('calMessage', 'Calibrating...', 0);
      try {
        const res = await fetch(`/calibrate?mass=${mass}`, { method: 'POST' });
        showTransientMessage('calMessage', await res.text());
      } catch(e) { showTransientMessage('calMessage', 'Calibration failed.'); }
    });
  }
  async function startMode2Test(btn) {
    const density = Number(document.getElementById('mode2Density').value);
    if (!Number.isFinite(density) || density <= 0) {
      alert('Enter a valid positive air density.');
      return;
    }
    if (!tableValid) {
      alert('The calibration table is invalid — Mode 2 is disabled.');
      return;
    }
    await withBusy(btn, 'Starting...', async function() {
      mode2ProcessedForThisRun = false;
      currentResultObject = null;
      document.getElementById('mode2Results').style.display = 'none';
      try {
        const response = await fetch('/start_mode2?density=' + encodeURIComponent(density), { method: 'POST' });
        const message = await response.text();
        if (!response.ok) alert(message);
      } catch(e) {
        alert('Failed to start the automatic speed sweep.');
      }
    });
  }
  // --- Responsive offline canvas charts ---
  function canvasContext(canvas) {
    const ratio = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(300, Math.floor(rect.width));
    const height = Math.max(200, Math.floor(rect.height));
    canvas.width = Math.floor(width * ratio);
    canvas.height = Math.floor(height * ratio);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    return {ctx:ctx, width:width, height:height};
  }
  function chartGeometry(ctx, width, height, xValues, yValues, xLabel, yLabel) {
    const pad = {left:58, right:20, top:22, bottom:48};
    let xMin = Math.min.apply(null, xValues);
    let xMax = Math.max.apply(null, xValues);
    let yMin = Math.min.apply(null, yValues);
    let yMax = Math.max.apply(null, yValues);
    if (xMax === xMin) xMax = xMin + 1;
    if (yMax === yMin) { yMin -= 0.5; yMax += 0.5; }
    const yMargin = Math.max((yMax - yMin) * 0.1, 0.05);
    yMin = Math.min(0, yMin - yMargin);
    yMax += yMargin;
    const plotW = width - pad.left - pad.right;
    const plotH = height - pad.top - pad.bottom;
    const xPixel = function(x) { return pad.left + (x - xMin) * plotW / (xMax - xMin); };
    const yPixel = function(y) { return pad.top + (yMax - y) * plotH / (yMax - yMin); };
    ctx.strokeStyle = '#2a3c63';
    ctx.fillStyle = '#94a3b8';
    ctx.lineWidth = 1;
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 5; i++) {
      const value = yMin + (yMax - yMin) * i / 5;
      const py = yPixel(value);
      ctx.beginPath(); ctx.moveTo(pad.left, py); ctx.lineTo(width - pad.right, py); ctx.stroke();
      ctx.fillText(value.toFixed(2), pad.left - 7, py);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let i = 0; i <= 5; i++) {
      const value = xMin + (xMax - xMin) * i / 5;
      const px = xPixel(value);
      ctx.fillText(value.toFixed(xMax <= 12 ? 1 : 0), px, height - pad.bottom + 8);
    }
    ctx.fillStyle = '#f8fafc';
    ctx.font = '12px sans-serif';
    ctx.fillText(xLabel, pad.left + plotW / 2, height - 18);
    ctx.save();
    ctx.translate(14, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText(yLabel, 0, 0);
    ctx.restore();
    return {x:xPixel, y:yPixel};
  }
  function drawSeriesChart(canvasId, xValues, yValues, xLabel, yLabel, color) {
    const canvas = document.getElementById(canvasId);
    if (!canvas || !xValues.length || xValues.length !== yValues.length) return;
    const prepared = canvasContext(canvas);
    const map = chartGeometry(prepared.ctx, prepared.width, prepared.height, xValues, yValues, xLabel, yLabel);
    const ctx = prepared.ctx;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    xValues.forEach(function(x, i) {
      const px = map.x(x), py = map.y(yValues[i]);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
    xValues.forEach(function(x, i) {
      ctx.beginPath(); ctx.arc(map.x(x), map.y(yValues[i]), 4, 0, Math.PI * 2); ctx.fill();
    });
  }
  function drawMode1Chart(result) {
    const canvas = document.getElementById('mode1ForceChart');
    const raw = Array.isArray(result.rawForces) ? result.rawForces : [];
    if (!canvas || !raw.length) return;
    const times = Array.isArray(result.sampleTimesMs) && result.sampleTimesMs.length === raw.length
      ? result.sampleTimesMs.map(function(v) { return v / 1000; })
      : raw.map(function(_, i) { return i * 10 / Math.max(1, raw.length - 1); });
    const mask = Array.isArray(result.acceptedMask) ? result.acceptedMask : raw.map(function() { return 1; });
    const prepared = canvasContext(canvas);
    const map = chartGeometry(prepared.ctx, prepared.width, prepared.height, times, raw, 'Measurement time (s)', 'Force (N)');
    const ctx = prepared.ctx;
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    raw.forEach(function(value, i) {
      if (i === 0) ctx.moveTo(map.x(times[i]), map.y(value));
      else ctx.lineTo(map.x(times[i]), map.y(value));
    });
    ctx.stroke();
    raw.forEach(function(value, i) {
      ctx.fillStyle = mask[i] ? '#38bdf8' : '#ef4444';
      ctx.beginPath();
      ctx.arc(map.x(times[i]), map.y(value), mask[i] ? 2.2 : 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  function drawMode2Charts(result) {
    if (!result || !Array.isArray(result.stages)) return;
    const speeds = result.stages.map(function(s) { return Number(s.velocityKmh); });
    drawSeriesChart('mode2ForceChart', speeds,
      result.stages.map(function(s) { return Number(s.avgForce); }),
      'Airflow speed (km/h)', 'Average force (N)', '#38bdf8');
    drawSeriesChart('mode2PowerChart', speeds,
      result.stages.map(function(s) { return Number(s.aeroPower); }),
      'Airflow speed (km/h)', 'Aerodynamic power (W)', '#a78bfa');
  }
  function displayResults(res) {
    currentResultObject = Object.assign({
      id: Date.now(),
      mode: 'Mode 1',
      status: 'Completed',
      timestamp: new Date().toLocaleString()
    }, res);
    document.getElementById('resTimestamp').innerText = currentResultObject.timestamp;
    document.getElementById('resCdA').innerText = Number(res.cdA).toFixed(4) + ' m²';
    document.getElementById('resNetCdA').innerText = Number(res.cdaExcludingCarrier != null ? res.cdaExcludingCarrier : res.cdA).toFixed(4) + ' m²';
    document.getElementById('resPower').innerText = Number(res.aeroPower).toFixed(2) + ' W';
    document.getElementById('resTargetVelocity').innerText =
      Number(res.targetVelocityKmh).toFixed(1) + ' km/h (' + Number(res.velocityMps).toFixed(2) + ' m/s)';
    document.getElementById('resPwm').innerText =
      res.motorPercent + '% (calc ' + Number(res.requestedPwmPercent).toFixed(1) + '%)';
    document.getElementById('resAvgForce').innerText = Number(res.avgForce).toFixed(2) + ' N';
    document.getElementById('resMedForce').innerText = Number(res.medianForce).toFixed(2) + ' N';
    document.getElementById('resStdDev').innerText = '±' + Number(res.stdDev).toFixed(2) + ' N';
    document.getElementById('resMinMax').innerText = Number(res.minForce).toFixed(2) + ' / ' + Number(res.maxForce).toFixed(2);
    document.getElementById('resPoints').innerText = res.acceptedReadings + ' acc (' + res.excludedReadings + ' excl)';
    document.getElementById('resCarrierOffset').innerText = Number(res.carrierOffsetAppliedN || 0).toFixed(3) + ' N';
    const saveBtn = document.getElementById('saveBtn');
    saveBtn.innerText = '💾 Save Result to History';
    saveBtn.disabled = false;
    revealWithFadeIn('resultsSection', 'block');
    requestAnimationFrame(function() { drawMode1Chart(res); });
    document.getElementById('resultsSection').scrollIntoView({behavior:'smooth'});
  }
  function displayMode2Results(res) {
    currentResultObject = {
      id: Date.now(),
      mode: 'Mode 2',
      status: 'Completed',
      timestamp: new Date().toLocaleString(),
      airDensity: res.airDensity,
      averageCdA: res.averageCdA,
      averageNetCdA: res.averageNetCdA,
      averageForce: res.averageForce,
      averagePower: res.averagePower,
      totalDurationMs: res.totalDurationMs,
      totalAccepted: res.totalAccepted,
      totalExcluded: res.totalExcluded,
      stages: res.stages
    };
    document.getElementById('m2Timestamp').innerText = currentResultObject.timestamp;
    document.getElementById('m2AvgCdA').innerText = Number(res.averageCdA).toFixed(4) + ' m²';
    document.getElementById('m2AvgNetCdA').innerText = Number(res.averageNetCdA != null ? res.averageNetCdA : res.averageCdA).toFixed(4) + ' m²';
    document.getElementById('m2AvgForce').innerText = Number(res.averageForce).toFixed(2) + ' N';
    document.getElementById('m2AvgPower').innerText = Number(res.averagePower).toFixed(2) + ' W';
    document.getElementById('m2Duration').innerText = (Number(res.totalDurationMs) / 1000).toFixed(1) + ' s';
    document.getElementById('m2Accepted').innerText = res.totalAccepted;
    document.getElementById('m2Excluded').innerText = res.totalExcluded;
    document.getElementById('mode2ResultsBody').innerHTML = res.stages.map(function(s) {
      return '<tr><td>' + Number(s.velocityKmh).toFixed(0) + '</td><td>' + Number(s.velocityMps).toFixed(2) +
        '</td><td>' + s.motorPercent + '%</td><td>' + s.totalReadings + '</td><td>' + s.acceptedReadings +
        '</td><td>' + s.excludedReadings + '</td><td>' + Number(s.avgForce).toFixed(3) +
        '</td><td>' + Number(s.medianForce).toFixed(3) + '</td><td>' + Number(s.minForce).toFixed(3) +
        ' / ' + Number(s.maxForce).toFixed(3) + '</td><td>' + Number(s.stdDev).toFixed(3) +
        '</td><td>' + Number(s.aeroPower).toFixed(2) + '</td><td>' + Number(s.carrierOffsetAppliedN || 0).toFixed(3) +
        '</td><td>' + Number(s.cdA).toFixed(4) + '</td><td>' + Number(s.cdaExcludingCarrier != null ? s.cdaExcludingCarrier : s.cdA).toFixed(4) + '</td></tr>';
    }).join('');
    const saveBtn = document.getElementById('saveMode2Btn');
    saveBtn.innerText = '💾 Save Result to History';
    saveBtn.disabled = false;
    revealWithFadeIn('mode2Results', 'block');
    requestAnimationFrame(function() { drawMode2Charts(res); });
    document.getElementById('mode2Results').scrollIntoView({behavior:'smooth'});
  }
  function startNewMode2TestClean() {
    document.getElementById('mode2Results').style.display = 'none';
    document.getElementById('mode2ProgressCard').style.display = 'none';
  }
  // --- Air Velocity Calibration (dedicated tab) ---
  const CALIBRATION_PERCENTS = [0,10,20,30,40,50,60,70,80,90,100];
  const CALIB_LOCALSTORAGE_KEY = 'air_velocity_calibration_v1';
  let calibDraft = {};
  CALIBRATION_PERCENTS.forEach(function(p) { calibDraft[p] = null; });
  let calibValid = false;
  let calibErrorMessage = '';
  function calibInputEl(percent) { return document.getElementById('calibInput' + percent); }
  function applyCalibDraftToInputs() {
    CALIBRATION_PERCENTS.forEach(function(p) {
      const el = calibInputEl(p);
      if (!el) return;
      const v = calibDraft[p];
      el.value = (typeof v === 'number' && Number.isFinite(v)) ? v : '';
    });
  }
  function saveDraftToLocalStorage() {
    try { localStorage.setItem(CALIB_LOCALSTORAGE_KEY, JSON.stringify(calibDraft)); } catch(e) {}
  }
  function loadDraftFromLocalStorage() {
    try {
      const cached = JSON.parse(localStorage.getItem(CALIB_LOCALSTORAGE_KEY) || 'null');
      if (cached && typeof cached === 'object') {
        CALIBRATION_PERCENTS.forEach(function(p) {
          if (typeof cached[p] === 'number' && Number.isFinite(cached[p])) calibDraft[p] = cached[p];
        });
      }
    } catch(e) {}
  }
  function onCalibInputChange(percent, rawValue) {
    const parsed = rawValue === '' ? null : Number(rawValue);
    calibDraft[percent] = (parsed !== null && Number.isFinite(parsed) && parsed >= 0) ? parsed : null;
    saveDraftToLocalStorage();
    drawCalibrationChart();
  }
  async function loadCalibrationFromDevice() {
    try {
      const res = await fetch('/calib_get', { cache: 'no-store' });
      const data = await res.json();
      (data.points || []).forEach(function(pt) {
        calibDraft[pt.percent] = pt.hasValue ? Number(pt.velocity) : null;
      });
      calibValid = !!data.valid;
      calibErrorMessage = data.error || '';
      applyCalibDraftToInputs();
      saveDraftToLocalStorage();
      updateCalibStatusUI();
      drawCalibrationChart();
    } catch(e) {
      // Offline: keep whatever was restored from localStorage.
    }
  }
  function updateCalibStatusUI() {
    const el = document.getElementById('calibStatusText');
    if (!el) return;
    if (calibValid) {
      el.innerText = 'Calibration Status: VALID';
      el.style.color = 'var(--success)';
    } else {
      el.innerText = 'Calibration Status: INVALID' + (calibErrorMessage ? ' — ' + calibErrorMessage : '');
      el.style.color = 'var(--danger)';
    }
  }
  async function setCalibrationPwm(percent, btn) {
    await withBusy(btn, 'Setting...', async function() {
      showTransientMessage('calibSaveMessage', 'Setting motors to ' + percent + '%...', 0);
      try {
        const res = await fetch('/calib_set_pwm?percent=' + percent, { method: 'POST' });
        const txt = await res.text();
        showTransientMessage('calibSaveMessage', txt);
      } catch(e) {
        showTransientMessage('calibSaveMessage', 'Failed to set calibration PWM.');
      }
    });
  }
  async function stopCalibrationMotors(btn) {
    await withBusy(btn, 'Stopping...', async function() {
      try {
        await fetch('/calib_stop', { method: 'POST' });
      } catch(e) { /* ignore */ }
    });
  }
  async function saveCalibration(btn) {
    const params = [];
    CALIBRATION_PERCENTS.forEach(function(p) {
      const v = calibDraft[p];
      if (typeof v === 'number' && Number.isFinite(v)) params.push('p' + p + '=' + encodeURIComponent(v));
    });
    if (!params.length) {
      showTransientMessage('calibSaveMessage', 'Enter at least one measured value before saving.');
      return;
    }
    await withBusy(btn, 'Saving...', async function() {
      showTransientMessage('calibSaveMessage', 'Saving calibration...', 0);
      try {
        const res = await fetch('/calib_save?' + params.join('&'), { method: 'POST' });
        if (!res.ok) {
          showTransientMessage('calibSaveMessage', await res.text());
          return;
        }
        const data = await res.json();
        (data.points || []).forEach(function(pt) {
          calibDraft[pt.percent] = pt.hasValue ? Number(pt.velocity) : null;
        });
        calibValid = !!data.valid;
        calibErrorMessage = data.error || '';
        applyCalibDraftToInputs();
        saveDraftToLocalStorage();
        updateCalibStatusUI();
        drawCalibrationChart();
        await loadVelocityTable();
        showTransientMessage('calibSaveMessage', 'Calibration saved to the ESP32.');
      } catch(e) {
        showTransientMessage('calibSaveMessage', 'Failed to save calibration.');
      }
    });
  }
  async function resetCalibration(btn) {
    if (!confirm('Reset ALL calibration points? This cannot be undone.')) return;
    await withBusy(btn, 'Resetting...', async function() {
      showTransientMessage('calibSaveMessage', 'Resetting calibration...', 0);
      try {
        const res = await fetch('/calib_reset', { method: 'POST' });
        const data = await res.json();
        CALIBRATION_PERCENTS.forEach(function(p) { calibDraft[p] = null; });
        (data.points || []).forEach(function(pt) {
          calibDraft[pt.percent] = pt.hasValue ? Number(pt.velocity) : null;
        });
        calibValid = !!data.valid;
        calibErrorMessage = data.error || '';
        applyCalibDraftToInputs();
        saveDraftToLocalStorage();
        updateCalibStatusUI();
        drawCalibrationChart();
        await loadVelocityTable();
        showTransientMessage('calibSaveMessage', 'Calibration reset.');
      } catch(e) {
        showTransientMessage('calibSaveMessage', 'Failed to reset calibration.');
      }
    });
  }
  async function refreshCalibrationStatus(btn) {
    await withBusy(btn, 'Checking...', async function() {
      await loadCalibrationFromDevice();
    });
  }
  function drawCalibrationChart() {
    const canvas = document.getElementById('calibrationChart');
    if (!canvas) return;
    const prepared = canvasContext(canvas);
    const ctx = prepared.ctx;
    const pts = [];
    CALIBRATION_PERCENTS.forEach(function(p) {
      const v = calibDraft[p];
      if (typeof v === 'number' && Number.isFinite(v)) pts.push({p:p, v:v});
    });
    if (!pts.length) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Enter calibration values to see the curve', prepared.width / 2, prepared.height / 2);
      return;
    }
    const xForRange = [0, 100].concat(pts.map(function(pt){ return pt.p; }));
    const yForRange = [0].concat(pts.map(function(pt){ return pt.v; }));
    const map = chartGeometry(ctx, prepared.width, prepared.height, xForRange, yForRange, 'Motor PWM (%)', 'Air Velocity (km/h)');
    ctx.strokeStyle = '#38bdf8';
    ctx.fillStyle = '#38bdf8';
    ctx.lineWidth = 2.5;
    if (pts.length > 1) {
      ctx.beginPath();
      pts.forEach(function(pt, i) {
        const px = map.x(pt.p), py = map.y(pt.v);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    pts.forEach(function(pt) {
      ctx.beginPath();
      ctx.arc(map.x(pt.p), map.y(pt.v), 4.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  function downloadCalibrationCsv() {
    const rows = ['PWM_Percent,Air_Velocity_km_h,Air_Velocity_m_s'];
    CALIBRATION_PERCENTS.forEach(function(p) {
      const v = calibDraft[p];
      const kmh = (typeof v === 'number' && Number.isFinite(v)) ? v : '';
      const mps = (typeof v === 'number' && Number.isFinite(v)) ? (v / 3.6).toFixed(3) : '';
      rows.push(p + ',' + kmh + ',' + mps);
    });
    const blob = new Blob([rows.join('\n') + '\n'], {type:'text/csv'});
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'Air_Velocity_Calibration_' + Date.now() + '.csv';
    link.click();
    setTimeout(function() { URL.revokeObjectURL(url); }, 1000);
  }
  // --- Carrier Body Offset (optional; shares SET/STOP with air-velocity calibration) ---
  const CARRIER_LOCALSTORAGE_KEY = 'carrier_body_offset_v1';
  let carrierDraft = {};
  CALIBRATION_PERCENTS.forEach(function(p) { carrierDraft[p] = null; });
  function carrierInputEl(percent) { return document.getElementById('carrierInput' + percent); }
  function applyCarrierDraftToInputs() {
    CALIBRATION_PERCENTS.forEach(function(p) {
      const el = carrierInputEl(p);
      if (!el) return;
      const v = carrierDraft[p];
      el.value = (typeof v === 'number' && Number.isFinite(v)) ? v : '';
    });
  }
  function saveCarrierDraftToLocalStorage() {
    try { localStorage.setItem(CARRIER_LOCALSTORAGE_KEY, JSON.stringify(carrierDraft)); } catch(e) {}
  }
  function loadCarrierDraftFromLocalStorage() {
    try {
      const cached = JSON.parse(localStorage.getItem(CARRIER_LOCALSTORAGE_KEY) || 'null');
      if (cached && typeof cached === 'object') {
        CALIBRATION_PERCENTS.forEach(function(p) {
          if (typeof cached[p] === 'number' && Number.isFinite(cached[p])) carrierDraft[p] = cached[p];
        });
      }
    } catch(e) {}
  }
  function onCarrierInputChange(percent, rawValue) {
    const parsed = rawValue === '' ? null : Number(rawValue);
    carrierDraft[percent] = (parsed !== null && Number.isFinite(parsed) && parsed >= 0) ? parsed : null;
    saveCarrierDraftToLocalStorage();
    drawCarrierOffsetChart();
  }
  function updateCarrierStatusUI(calibratedCount) {
    const el = document.getElementById('carrierStatusText');
    if (!el) return;
    const count = typeof calibratedCount === 'number' ? calibratedCount :
      CALIBRATION_PERCENTS.filter(function(p) { return typeof carrierDraft[p] === 'number' && Number.isFinite(carrierDraft[p]); }).length;
    el.innerText = count === 0
      ? 'Carrier offset calibrated at 0 of 11 points (offset = 0.00 N everywhere)'
      : 'Carrier offset calibrated at ' + count + ' of 11 points';
  }
  async function loadCarrierOffsetFromDevice() {
    try {
      const res = await fetch('/carrier_offset_get', { cache: 'no-store' });
      const data = await res.json();
      (data.points || []).forEach(function(pt) {
        carrierDraft[pt.percent] = pt.hasValue ? Number(pt.offsetN) : null;
      });
      applyCarrierDraftToInputs();
      saveCarrierDraftToLocalStorage();
      updateCarrierStatusUI(data.calibratedCount);
      drawCarrierOffsetChart();
    } catch(e) {
      // Offline: keep whatever was restored from localStorage.
    }
  }
  async function saveCarrierOffset(btn) {
    const params = [];
    CALIBRATION_PERCENTS.forEach(function(p) {
      const v = carrierDraft[p];
      if (typeof v === 'number' && Number.isFinite(v)) params.push('p' + p + '=' + encodeURIComponent(v));
    });
    if (!params.length) {
      showTransientMessage('carrierSaveMessage', 'Enter at least one measured offset before saving.');
      return;
    }
    await withBusy(btn, 'Saving...', async function() {
      showTransientMessage('carrierSaveMessage', 'Saving carrier offset...', 0);
      try {
        const res = await fetch('/carrier_offset_save?' + params.join('&'), { method: 'POST' });
        if (!res.ok) {
          showTransientMessage('carrierSaveMessage', await res.text());
          return;
        }
        const data = await res.json();
        (data.points || []).forEach(function(pt) {
          carrierDraft[pt.percent] = pt.hasValue ? Number(pt.offsetN) : null;
        });
        applyCarrierDraftToInputs();
        saveCarrierDraftToLocalStorage();
        updateCarrierStatusUI(data.calibratedCount);
        drawCarrierOffsetChart();
        showTransientMessage('carrierSaveMessage', 'Carrier offset saved to the ESP32.');
      } catch(e) {
        showTransientMessage('carrierSaveMessage', 'Failed to save carrier offset.');
      }
    });
  }
  async function resetCarrierOffset(btn) {
    if (!confirm('Reset ALL carrier offset points? This cannot be undone.')) return;
    await withBusy(btn, 'Resetting...', async function() {
      showTransientMessage('carrierSaveMessage', 'Resetting carrier offset...', 0);
      try {
        const res = await fetch('/carrier_offset_reset', { method: 'POST' });
        const data = await res.json();
        CALIBRATION_PERCENTS.forEach(function(p) { carrierDraft[p] = null; });
        (data.points || []).forEach(function(pt) {
          carrierDraft[pt.percent] = pt.hasValue ? Number(pt.offsetN) : null;
        });
        applyCarrierDraftToInputs();
        saveCarrierDraftToLocalStorage();
        updateCarrierStatusUI(data.calibratedCount);
        drawCarrierOffsetChart();
        showTransientMessage('carrierSaveMessage', 'Carrier offset reset.');
      } catch(e) {
        showTransientMessage('carrierSaveMessage', 'Failed to reset carrier offset.');
      }
    });
  }
  function drawCarrierOffsetChart() {
    const canvas = document.getElementById('carrierOffsetChart');
    if (!canvas) return;
    const prepared = canvasContext(canvas);
    const ctx = prepared.ctx;
    const pts = [];
    CALIBRATION_PERCENTS.forEach(function(p) {
      const v = carrierDraft[p];
      if (typeof v === 'number' && Number.isFinite(v)) pts.push({p:p, v:v});
    });
    if (!pts.length) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Enter carrier offset values to see the curve', prepared.width / 2, prepared.height / 2);
      return;
    }
    const xForRange = [0, 100].concat(pts.map(function(pt){ return pt.p; }));
    const yForRange = [0].concat(pts.map(function(pt){ return pt.v; }));
    const map = chartGeometry(ctx, prepared.width, prepared.height, xForRange, yForRange, 'Motor PWM (%)', 'Carrier Offset Force (N)');
    ctx.strokeStyle = '#a78bfa';
    ctx.fillStyle = '#a78bfa';
    ctx.lineWidth = 2.5;
    if (pts.length > 1) {
      ctx.beginPath();
      pts.forEach(function(pt, i) {
        const px = map.x(pt.p), py = map.y(pt.v);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    pts.forEach(function(pt) {
      ctx.beginPath();
      ctx.arc(map.x(pt.p), map.y(pt.v), 4.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  // --- Carrier Body CdA (single constant value, not speed-dependent) ---
  const CARRIER_CDA_LOCALSTORAGE_KEY = 'carrier_body_cda_v1';
  let carrierCdaDraft = null;
  function updateCarrierCdaStatusUI() {
    const el = document.getElementById('carrierCdaStatusText');
    if (!el) return;
    el.innerText = (typeof carrierCdaDraft === 'number' && Number.isFinite(carrierCdaDraft) && carrierCdaDraft > 0)
      ? 'Carrier CdA offset: ' + carrierCdaDraft.toFixed(4) + ' m² (subtracted from every result)'
      : 'Carrier CdA offset: not set (0.0000 m², no effect)';
  }
  function onCarrierCdaInputChange(rawValue) {
    const parsed = rawValue === '' ? null : Number(rawValue);
    carrierCdaDraft = (parsed !== null && Number.isFinite(parsed) && parsed >= 0) ? parsed : null;
    try { localStorage.setItem(CARRIER_CDA_LOCALSTORAGE_KEY, JSON.stringify(carrierCdaDraft)); } catch(e) {}
  }
  function applyCarrierCdaDraftToInput() {
    const el = document.getElementById('carrierCdaInput');
    if (el) el.value = (typeof carrierCdaDraft === 'number' && Number.isFinite(carrierCdaDraft)) ? carrierCdaDraft : '';
    updateCarrierCdaStatusUI();
  }
  function loadCarrierCdaDraftFromLocalStorage() {
    try {
      const cached = JSON.parse(localStorage.getItem(CARRIER_CDA_LOCALSTORAGE_KEY) || 'null');
      if (typeof cached === 'number' && Number.isFinite(cached)) carrierCdaDraft = cached;
    } catch(e) {}
  }
  async function loadCarrierCdaFromDevice() {
    try {
      const res = await fetch('/carrier_cda_get', { cache: 'no-store' });
      const data = await res.json();
      carrierCdaDraft = data.isSet ? Number(data.cda) : null;
      applyCarrierCdaDraftToInput();
      try { localStorage.setItem(CARRIER_CDA_LOCALSTORAGE_KEY, JSON.stringify(carrierCdaDraft)); } catch(e) {}
    } catch(e) {
      // Offline: keep whatever was restored from localStorage.
    }
  }
  async function saveCarrierCda(btn) {
    if (typeof carrierCdaDraft !== 'number' || !Number.isFinite(carrierCdaDraft)) {
      showTransientMessage('carrierCdaMessage', 'Enter a valid carrier CdA value before saving.');
      return;
    }
    await withBusy(btn, 'Saving...', async function() {
      showTransientMessage('carrierCdaMessage', 'Saving carrier CdA...', 0);
      try {
        const res = await fetch('/carrier_cda_save?cda=' + encodeURIComponent(carrierCdaDraft), { method: 'POST' });
        if (!res.ok) {
          showTransientMessage('carrierCdaMessage', await res.text());
          return;
        }
        const data = await res.json();
        carrierCdaDraft = data.isSet ? Number(data.cda) : null;
        applyCarrierCdaDraftToInput();
        try { localStorage.setItem(CARRIER_CDA_LOCALSTORAGE_KEY, JSON.stringify(carrierCdaDraft)); } catch(e) {}
        showTransientMessage('carrierCdaMessage', 'Carrier CdA saved to the ESP32.');
      } catch(e) {
        showTransientMessage('carrierCdaMessage', 'Failed to save carrier CdA.');
      }
    });
  }
  async function resetCarrierCda(btn) {
    if (!confirm('Reset the carrier CdA offset to 0?')) return;
    await withBusy(btn, 'Resetting...', async function() {
      showTransientMessage('carrierCdaMessage', 'Resetting carrier CdA...', 0);
      try {
        const res = await fetch('/carrier_cda_reset', { method: 'POST' });
        const data = await res.json();
        carrierCdaDraft = data.isSet ? Number(data.cda) : null;
        applyCarrierCdaDraftToInput();
        try { localStorage.removeItem(CARRIER_CDA_LOCALSTORAGE_KEY); } catch(e) {}
        showTransientMessage('carrierCdaMessage', 'Carrier CdA reset.');
      } catch(e) {
        showTransientMessage('carrierCdaMessage', 'Failed to reset carrier CdA.');
      }
    });
  }
  window.addEventListener('beforeunload', function() {
    if (calibrationModeActiveClient) {
      try { navigator.sendBeacon('/calib_stop'); } catch(e) {}
    }
  });
  async function updateDashboard() {
    try {
      const response = await fetch('/data', {cache:'no-store'});
      const data = await response.json();
      if (velocityTable.length >= 2) { tableValid = !!data.calibrationValid; }
      document.getElementById('calibBanner').style.display = data.calibrationValid ? 'none' : 'block';
      calibValid = !!data.calibrationValid;
      calibErrorMessage = data.calibrationErrorMessage || '';
      updateCalibStatusUI();
      calibrationModeActiveClient = !!data.calibrationModeActive;
      const runEl = document.getElementById('calibRunningPwm');
      if (runEl) {
        runEl.innerText = data.calibrationModeActive ? (data.calibrationRunningPercent + '% PWM') : 'Motors off';
      }
      const liveForceEl = document.getElementById('calibLiveForce');
      if (liveForceEl) {
        liveForceEl.innerText = (data.ready && data.calibrated) ? Number(data.force).toFixed(2) + ' N' : '-- N';
      }
      const badge = document.getElementById('sensorBadge');
      if (!data.ready) {
        badge.innerText = 'Sensor Disconnected'; badge.style.color = 'var(--danger)';
      } else if (!data.calibrated) {
        badge.innerText = 'Calibration Required'; badge.style.color = '#eab308';
      } else {
        badge.innerText = 'Sensor Online'; badge.style.color = 'var(--success)';
      }
      document.getElementById('liveSpeed').innerText = data.speed;
      document.getElementById('liveForce').innerText =
        (data.ready && data.calibrated) ? Number(data.force).toFixed(2) : '--';
      document.getElementById('liveEstVelocity').innerText =
        data.estimatedVelocityKmh >= 0 ? Number(data.estimatedVelocityKmh).toFixed(1) : '--';
      const liveNetForceEl = document.getElementById('liveNetForce');
      if (liveNetForceEl) {
        liveNetForceEl.innerText = (data.ready && data.calibrated) ? Number(data.netForceN).toFixed(2) : '--';
      }
      const liveOffsetEl = document.getElementById('liveCarrierOffsetDisplay');
      if (liveOffsetEl) {
        liveOffsetEl.innerText = Number(data.liveCarrierOffsetN || 0).toFixed(2);
      }
      const liveForceAgeEl = document.getElementById('liveForceAge');
      if (liveForceAgeEl) {
        liveForceAgeEl.innerText = 'sensor updated ' + (Number.isFinite(data.forceAgeMs) ? data.forceAgeMs : '--') + ' ms ago';
      }
      document.getElementById('m2LiveSpeed').innerText = data.speed + '%';
      document.getElementById('m2LiveForce').innerText =
        (data.ready && data.calibrated) ? Number(data.force).toFixed(2) + ' N' : '-- N';
      document.getElementById('m2EstVelocity').innerText =
        data.estimatedVelocityKmh >= 0 ? Number(data.estimatedVelocityKmh).toFixed(1) + ' km/h' : '--';
      handleTestStateUI(data);
    } catch(e) {
      const badge = document.getElementById('sensorBadge');
      badge.innerText = 'Connection Lost'; badge.style.color = 'var(--danger)';
    }
  }
  function handleTestStateUI(data) {
    testInProgress = data.state >= 1 && data.state <= 3;
    const calibActive = !!data.calibrationModeActive;
    const disableStarts = testInProgress || !data.calibrationValid || calibActive;
    document.getElementById('tabBtn1').disabled = testInProgress;
    document.getElementById('tabBtn2').disabled = testInProgress;
    document.getElementById('tabBtn3').disabled = testInProgress;
    document.getElementById('startTestBtn').disabled = disableStarts;
    document.getElementById('startMode2Btn').disabled = disableStarts;
    document.getElementById('inputVelocityRange').disabled = testInProgress;
    document.getElementById('inputVelocityNumber').disabled = testInProgress;
    document.getElementById('inputDensity').disabled = testInProgress;
    document.getElementById('manualSpeedSlider').disabled = testInProgress;
    document.getElementById('setManualSpeedBtn').disabled = testInProgress;
    document.getElementById('manualStopBtn').disabled = testInProgress;
    document.querySelectorAll('.test-control').forEach(function(el) { el.disabled = testInProgress; });
    document.querySelectorAll('.calib-set-btn').forEach(function(btn) { btn.disabled = testInProgress; });
    CALIBRATION_PERCENTS.forEach(function(p) { const el = calibInputEl(p); if (el) el.disabled = testInProgress; });
    const calibSaveBtnEl = document.getElementById('calibSaveBtn'); if (calibSaveBtnEl) calibSaveBtnEl.disabled = testInProgress;
    const calibResetBtnEl = document.getElementById('calibResetBtn'); if (calibResetBtnEl) calibResetBtnEl.disabled = testInProgress;
    const calibValidateBtnEl = document.getElementById('calibValidateBtn'); if (calibValidateBtnEl) calibValidateBtnEl.disabled = testInProgress;
    CALIBRATION_PERCENTS.forEach(function(p) { const el = carrierInputEl(p); if (el) el.disabled = testInProgress; });
    const carrierSaveBtnEl = document.getElementById('carrierSaveBtn'); if (carrierSaveBtnEl) carrierSaveBtnEl.disabled = testInProgress;
    const carrierResetBtnEl = document.getElementById('carrierResetBtn'); if (carrierResetBtnEl) carrierResetBtnEl.disabled = testInProgress;
    const carrierCdaInputEl = document.getElementById('carrierCdaInput'); if (carrierCdaInputEl) carrierCdaInputEl.disabled = testInProgress;
    const carrierCdaSaveBtnEl = document.getElementById('carrierCdaSaveBtn'); if (carrierCdaSaveBtnEl) carrierCdaSaveBtnEl.disabled = testInProgress;
    const carrierCdaResetBtnEl = document.getElementById('carrierCdaResetBtn'); if (carrierCdaResetBtnEl) carrierCdaResetBtnEl.disabled = testInProgress;
    const banner = document.getElementById('testBanner');
    if (data.mode === 1 && testInProgress) {
      banner.style.display = 'block';
      banner.style.background = '#0d2847';
      banner.style.borderColor = 'var(--primary)';
      document.getElementById('testPhaseLabel').style.color = 'var(--primary)';
      document.getElementById('tbTargetVelocity').innerText = Number(data.targetVelocityKmh).toFixed(1) + ' km/h';
      document.getElementById('tbEstVelocity').innerText =
        data.estimatedVelocityKmh >= 0 ? Number(data.estimatedVelocityKmh).toFixed(1) + ' km/h' : '--';
      document.getElementById('tbTargetPwm').innerText = data.targetPwm + '%';
      document.getElementById('tbLivePwm').innerText = data.speed + '%';
      if (data.state === 1) {
        document.getElementById('testPhaseLabel').innerText = '1. Motor Acceleration (3s)';
        document.getElementById('countdownDisplay').innerText = data.speed + '%';
        document.getElementById('testPhaseInstruction').innerText = 'Motors accelerating...';
      } else if (data.state === 2) {
        document.getElementById('testPhaseLabel').innerText = '2. Preparation Countdown (5s)';
        document.getElementById('countdownDisplay').innerText = Math.ceil(data.countdown);
        document.getElementById('testPhaseInstruction').innerText = 'Please remain still on the bike.';
      } else {
        document.getElementById('testPhaseLabel').innerText = '3. Measurement in Progress (10s)';
        document.getElementById('countdownDisplay').innerText = Math.ceil(data.countdown);
        document.getElementById('testPhaseInstruction').innerText = 'Measurement in progress—please do not move.';
      }
    } else if (data.mode === 1 && data.state === 5) {
      banner.style.display = 'block';
      banner.style.background = '#3b1118';
      banner.style.borderColor = 'var(--danger)';
      document.getElementById('testPhaseLabel').style.color = 'var(--danger)';
      document.getElementById('testPhaseLabel').innerText = data.cancelled ? 'TEST CANCELLED' : 'TEST STOPPED — SAFETY ERROR';
      document.getElementById('countdownDisplay').innerText = '!';
      document.getElementById('testPhaseInstruction').innerText = data.error || 'Test stopped.';
    } else {
      banner.style.display = 'none';
    }
    const m2Card = document.getElementById('mode2ProgressCard');
    if (data.mode === 2 && (testInProgress || data.state === 5 || data.state === 4)) {
      m2Card.style.display = 'block';
      document.getElementById('m2Target').innerText = Number(data.targetVelocityKmh || 0).toFixed(0) + ' km/h';
      document.getElementById('m2Velocity').innerText = Number(data.targetVelocityMps || 0).toFixed(2) + ' m/s';
      document.getElementById('m2Pwm').innerText = (data.targetPwm || 0) + '%';
      document.getElementById('m2Position').innerText = 'Speed ' + (data.position || 1) + ' of 6';
      document.getElementById('m2Next').innerText = data.nextVelocityKmh > 0 ? Number(data.nextVelocityKmh).toFixed(0) + ' km/h' : 'None';
      document.getElementById('m2ProgressFill').style.width = Math.max(0, Math.min(100, Number(data.progress || 0))) + '%';
      document.getElementById('m2ProgressText').innerText = Number(data.progress || 0).toFixed(0) + '% complete';
      let stage = 'Waiting', instruction = '', count = '--';
      if (data.state === 1) {
        stage = 'Accelerating'; instruction = 'Ramping smoothly to the calculated PWM.'; count = Math.ceil(data.countdown);
      } else if (data.state === 2) {
        stage = 'Preparing'; instruction = 'Please remain still on the bike.'; count = Math.ceil(data.countdown);
      } else if (data.state === 3) {
        stage = 'Measuring'; instruction = 'Measuring at ' + Number(data.targetVelocityKmh).toFixed(0) + ' km/h—please do not move.'; count = Math.ceil(data.countdown);
      } else if (data.state === 4) {
        stage = 'Completed'; instruction = 'All six speed stages completed. Motors are off.'; count = '✓';
      } else if (data.state === 5) {
        stage = data.cancelled ? 'Cancelled' : 'Safety Error'; instruction = data.error || 'Sweep stopped.'; count = '!';
      }
      document.getElementById('m2Stage').innerText = stage;
      document.getElementById('m2Instruction').innerText = instruction;
      document.getElementById('m2Countdown').innerText = count;
    } else if (data.mode !== 2) {
      m2Card.style.display = 'none';
    }
    if (data.state === 4 && data.mode === 1 && data.result && data.result.valid && !resultProcessedForThisRun) {
      displayResults(data.result);
      resultProcessedForThisRun = true;
    }
    if (data.state === 4 && data.mode === 2 && data.mode2Result && data.mode2Result.valid && !mode2ProcessedForThisRun) {
      displayMode2Results(data.mode2Result);
      mode2ProcessedForThisRun = true;
    }
  }
  // --- Backward-compatible combined history ---
  function getHistory() {
    try {
      const parsed = JSON.parse(localStorage.getItem('airflow_results_log') || '[]');
      return Array.isArray(parsed) ? parsed : [];
    } catch(e) {
      return [];
    }
  }
  function deleteHistoryIndex(index) {
    const history = getHistory();
    history.splice(index, 1);
    localStorage.setItem('airflow_results_log', JSON.stringify(history));
    renderHistory();
  }
  function renderHistory() {
    const history = getHistory();
    const tbody = document.getElementById('combinedHistoryBody');
    if (!history.length) {
      tbody.innerHTML = '<tr><td colspan="10" style="text-align:center;color:var(--text-muted);">No saved results yet.</td></tr>';
      return;
    }
    tbody.innerHTML = history.map(function(item, index) {
      const mode = item.mode || 'Mode 1';
      const isMode2 = mode === 'Mode 2' || Array.isArray(item.stages);
      const setup = isMode2 ? '10–35 km/h sweep' :
        ((item.targetVelocityKmh !== undefined ? Number(item.targetVelocityKmh).toFixed(1) : '--') + ' km/h (' +
        (item.motorPercent !== undefined ? item.motorPercent : '--') + '% PWM)');
      const avgForce = isMode2 ? item.averageForce : item.avgForce;
      const avgPower = isMode2 ? item.averagePower : item.aeroPower;
      const cda = isMode2 ? item.averageCdA : item.cdA;
      const accepted = isMode2 ? item.totalAccepted : item.acceptedReadings;
      const total = isMode2 ? Number(item.totalAccepted || 0) + Number(item.totalExcluded || 0) : item.totalReadings;
      let graphCell = '<span style="color:var(--text-muted);font-size:12px;">No graph</span>';
      if (isMode2 && item.chartImages && (item.chartImages.force || item.chartImages.power)) {
        const forceImg = item.chartImages.force;
        const powerImg = item.chartImages.power;
        graphCell = '<div class="thumb-row">' +
          (forceImg ? '<img class="history-thumb" src="' + forceImg + '" title="Force vs speed" onclick="openImageModal(\'' + forceImg + '\',\'Force vs Airflow Speed\')">' : '') +
          (powerImg ? '<img class="history-thumb" src="' + powerImg + '" title="Power vs speed" onclick="openImageModal(\'' + powerImg + '\',\'Power vs Airflow Speed\')">' : '') +
          '</div>';
      } else if (!isMode2 && item.chartImage) {
        graphCell = '<img class="history-thumb" src="' + item.chartImage + '" title="Force vs time" onclick="openImageModal(\'' + item.chartImage + '\',\'Force vs Measurement Time\')">';
      }
      return '<tr><td>' + mode + '</td><td>' + (item.timestamp || 'Unknown') + '</td><td>' + setup +
        '</td><td>' + (Number.isFinite(Number(avgForce)) ? Number(avgForce).toFixed(2) + ' N' : '--') +
        '</td><td>' + (Number.isFinite(Number(avgPower)) ? Number(avgPower).toFixed(1) + ' W' : '--') +
        '</td><td style="font-weight:bold;color:var(--primary);">' +
        (Number.isFinite(Number(cda)) ? Number(cda).toFixed(4) + ' m²' : '--') +
        '</td><td>' + (accepted !== undefined ? accepted : '--') + '/' + (total !== undefined ? total : '--') +
        '</td><td>' + (item.status || 'Completed') +
        '</td><td>' + graphCell +
        '</td><td><button class="btn-secondary" style="padding:3px 8px;font-size:12px;color:var(--danger);" onclick="deleteHistoryIndex(' +
        index + ')">✕</button></td></tr>';
    }).join('');
  }
  function csvCell(value) {
    const text = value === undefined || value === null ? '' : String(value);
    return '"' + text.replace(/"/g, '""') + '"';
  }
  function exportCSV() {
    const history = getHistory();
    if (!history.length) {
      alert('No saved data available to export.');
      return;
    }
    const headers = [
      'Mode','Status','Timestamp','Air Density (kg/m3)','Stage','Target Velocity (km/h)',
      'Velocity (m/s)','Calculated PWM (%)','Total Samples','Accepted Samples',
      'Excluded Samples','Average Force (N)','Median Force (N)','Min Force (N)',
      'Max Force (N)','Std Dev (N)','Power (W)','Carrier Offset Applied (N)',
      'Stage CdA (m2)','Stage Net CdA excl. Carrier (m2)',
      'Final Average CdA (m2)','Final Average Net CdA excl. Carrier (m2)'
    ];
    const rows = [headers.map(csvCell).join(',')];
    history.forEach(function(record) {
      const mode = record.mode || 'Mode 1';
      if ((mode === 'Mode 2' || Array.isArray(record.stages)) && Array.isArray(record.stages)) {
        record.stages.forEach(function(stage, index) {
          rows.push([
            'Mode 2', record.status || 'Completed', record.timestamp, record.airDensity,
            index + 1, stage.velocityKmh, stage.velocityMps, stage.motorPercent,
            stage.totalReadings, stage.acceptedReadings, stage.excludedReadings,
            stage.avgForce, stage.medianForce, stage.minForce, stage.maxForce,
            stage.stdDev, stage.aeroPower, stage.carrierOffsetAppliedN || 0,
            stage.cdA, stage.cdaExcludingCarrier != null ? stage.cdaExcludingCarrier : stage.cdA,
            record.averageCdA, record.averageNetCdA != null ? record.averageNetCdA : record.averageCdA
          ].map(csvCell).join(','));
        });
      } else {
        rows.push([
          'Mode 1', record.status || 'Completed', record.timestamp, record.airDensity,
          1, record.targetVelocityKmh, record.velocityMps, record.motorPercent,
          record.totalReadings, record.acceptedReadings, record.excludedReadings,
          record.avgForce, record.medianForce, record.minForce, record.maxForce,
          record.stdDev, record.aeroPower, record.carrierOffsetAppliedN || 0,
          record.cdA, record.cdaExcludingCarrier != null ? record.cdaExcludingCarrier : record.cdA,
          record.cdA, record.cdaExcludingCarrier != null ? record.cdaExcludingCarrier : record.cdA
        ].map(csvCell).join(','));
      }
    });
    const blob = new Blob([rows.join('\n') + '\n'], {type:'text/csv'});
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'Aero_Test_Data_' + Date.now() + '.csv';
    link.click();
    setTimeout(function() { URL.revokeObjectURL(url); }, 1000);
  }
  window.addEventListener('resize', function() {
    if (currentResultObject) {
      if (currentResultObject.mode === 'Mode 2') drawMode2Charts(currentResultObject);
      else drawMode1Chart(currentResultObject);
    }
    drawCalibrationChart();
    drawCarrierOffsetChart();
  });
  // Pause the 300ms polling loop while the tab/window is hidden (backgrounded,
  // minimized, screen off) to save battery/bandwidth on the browsing device,
  // and resume — with one immediate refresh — the instant it's visible again.
  // The ESP32 keeps running its test state machine regardless; this only
  // affects how often the browser asks it for a status update.
  let dashboardPollHandle = null;
  function startDashboardPolling() {
    if (dashboardPollHandle) return;
    updateDashboard();
    dashboardPollHandle = setInterval(updateDashboard, 300);
  }
  function stopDashboardPolling() {
    if (dashboardPollHandle) {
      clearInterval(dashboardPollHandle);
      dashboardPollHandle = null;
    }
  }
  document.addEventListener('visibilitychange', function() {
    if (document.hidden) stopDashboardPolling();
    else startDashboardPolling();
  });
  startDashboardPolling();
  loadVelocityTable();
  renderHistory();
  loadDraftFromLocalStorage();
  applyCalibDraftToInputs();
  drawCalibrationChart();
  loadCalibrationFromDevice();
  loadCarrierDraftFromLocalStorage();
  applyCarrierDraftToInputs();
  drawCarrierOffsetChart();
  loadCarrierOffsetFromDevice();
  loadCarrierCdaDraftFromLocalStorage();
  applyCarrierCdaDraftToInput();
  loadCarrierCdaFromDevice();
</script>
</body>
</html>
)rawliteral";
// ==================================================
// WIRELESS SENSOR NODE COMMUNICATION
// ==================================================
void handleSensorUpdate() {
  if (!server.hasArg("token") || server.arg("token") != SENSOR_TOKEN) {
    server.send(403, "text/plain", "Forbidden");
    return;
  }
  if (!server.hasArg("ready") ||
      !server.hasArg("calibrated") ||
      !server.hasArg("force") ||
      !server.hasArg("sample_force") ||
      !server.hasArg("sequence")) {
    server.send(400, "text/plain", "Incomplete sensor packet");
    return;
  }
  float receivedDisplayForce = server.arg("force").toFloat();
  float receivedSampleForce = server.arg("sample_force").toFloat();
  bool receivedReady = server.arg("ready").toInt() == 1;
  bool receivedCalibrated = server.arg("calibrated").toInt() == 1;
  uint32_t receivedSequence = strtoul(server.arg("sequence").c_str(), nullptr, 10);
  if (!isfinite(receivedDisplayForce) ||
      !isfinite(receivedSampleForce) ||
      receivedDisplayForce < 0.0f ||
      receivedSampleForce < 0.0f) {
    server.send(400, "text/plain", "Invalid sensor values");
    return;
  }
  sensorNodeIP = server.client().remoteIP();
  sensorNodeKnown = true;
  sensorConnected = receivedReady;
  calibrated = receivedCalibrated;
  if (receivedReady) {
    lastGoodSensorReading = millis();
  }
  if (testIsActive() && receivedReady && !receivedCalibrated) {
    abortActiveTest(
      "The wireless force sensor became uncalibrated. Motors were stopped automatically.");
  }
  displayedForceN = receivedDisplayForce;
  lastForceUpdateMs = millis();
  // Store each wireless HX711 conversion only once.
  if (receivedReady &&
      receivedCalibrated &&
      receivedSequence != lastSensorSequence) {
    lastSensorSequence = receivedSequence;
    pendingSampleForceN = receivedSampleForce;
    newSensorSample = true;
  }
  server.send(200, "text/plain", "OK");
}
bool sendCommandToSensor(const String& commandPath,
                         String& responseText) {
  if (!sensorNodeKnown || !sensorConnected) {
    responseText = "Wireless sensor node is not connected.";
    return false;
  }
  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(1500);
  http.setTimeout(6000);
  String url = "http://" + sensorNodeIP.toString() + commandPath;
  if (!http.begin(client, url)) {
    responseText = "Could not open connection to sensor node.";
    return false;
  }
  int statusCode = http.POST("");
  responseText = (statusCode > 0)
    ? http.getString()
    : "Sensor-node command failed.";
  bool success = statusCode >= 200 && statusCode < 300;
  http.end();
  return success;
}
// ==================================================
// WEB HANDLERS
// ==================================================
void handleMainPage() {
  lastBrowserContact = millis();
  server.send_P(200, "text/html", WEBPAGE);
}
void handleVelocityTable() {
  lastBrowserContact = millis();
  String json = "{";
  json.reserve(1024);
  json += "\"valid\":" + String(calibrationValid ? "true" : "false");
  json += ",\"minVelocityKmh\":" + String(calibrationValid ? calibrationMinVelocityKmh() : 0.0f, 2);
  json += ",\"maxVelocityKmh\":" + String(calibrationValid ? calibrationMaxVelocityKmh() : 0.0f, 2);
  json += ",\"table\":[";
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    if (i) json += ",";
    json += "{\"pwm\":" + String(calibrationTable[i].motorPercent) +
            ",\"kmh\":" + String(calibrationTable[i].hasValue ? calibrationTable[i].velocityKmh : 0.0f, 2) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}
String buildCalibrationJson() {
  String json = "{";
  json.reserve(1024);
  json += "\"valid\":" + String(calibrationValid ? "true" : "false");
  String safeError = calibrationErrorMessage;
  safeError.replace("\\", "\\\\");
  safeError.replace("\"", "\\\"");
  json += ",\"error\":\"" + safeError + "\"";
  json += ",\"points\":[";
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    if (i) json += ",";
    json += "{\"percent\":" + String(calibrationTable[i].motorPercent);
    json += ",\"hasValue\":" + String(calibrationTable[i].hasValue ? "true" : "false");
    json += ",\"velocity\":" + String(calibrationTable[i].hasValue ? calibrationTable[i].velocityKmh : 0.0f, 3);
    json += "}";
  }
  json += "]}";
  return json;
}
void handleCalibGet() {
  lastBrowserContact = millis();
  server.send(200, "application/json", buildCalibrationJson());
}
String buildCarrierOffsetJson() {
  String json = "{";
  json.reserve(768);
  json += "\"calibratedCount\":" + String(carrierOffsetCalibratedCount());
  json += ",\"points\":[";
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    if (i) json += ",";
    json += "{\"percent\":" + String(carrierOffsetTable[i].motorPercent);
    json += ",\"hasValue\":" + String(carrierOffsetTable[i].hasValue ? "true" : "false");
    json += ",\"offsetN\":" + String(carrierOffsetTable[i].hasValue ? carrierOffsetTable[i].offsetForceN : 0.0f, 4);
    json += "}";
  }
  json += "]}";
  return json;
}
void handleCarrierOffsetGet() {
  lastBrowserContact = millis();
  server.send(200, "application/json", buildCarrierOffsetJson());
}
void handleCarrierOffsetSave() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot save carrier offset while a Mode 1/2 test is active.");
    return;
  }
  bool anyProvided = false;
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    String key = "p" + String(carrierOffsetTable[i].motorPercent);
    if (server.hasArg(key)) {
      float value = server.arg(key).toFloat();
      if (!isfinite(value) || value < 0.0f) {
        server.send(400, "text/plain", key + " must be a non-negative number.");
        return;
      }
      carrierOffsetTable[i].offsetForceN = value;
      carrierOffsetTable[i].hasValue = true;
      anyProvided = true;
    }
  }
  if (!anyProvided) {
    server.send(400, "text/plain", "No carrier offset values were provided.");
    return;
  }
  saveCarrierOffsetToStorage();
  server.send(200, "application/json", buildCarrierOffsetJson());
}
void handleCarrierOffsetReset() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot reset carrier offset while a Mode 1/2 test is active.");
    return;
  }
  resetCarrierOffsetToDefaults();
  server.send(200, "application/json", buildCarrierOffsetJson());
}
String buildCarrierCdaJson() {
  String json = "{";
  json.reserve(96);
  json += "\"isSet\":" + String(carrierBodyCdASet ? "true" : "false");
  json += ",\"cda\":" + String(carrierBodyCdA, 5);
  json += "}";
  return json;
}
void handleCarrierCdaGet() {
  lastBrowserContact = millis();
  server.send(200, "application/json", buildCarrierCdaJson());
}
void handleCarrierCdaSave() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot save carrier CdA while a Mode 1/2 test is active.");
    return;
  }
  if (!server.hasArg("cda")) {
    server.send(400, "text/plain", "Missing cda parameter.");
    return;
  }
  float value = server.arg("cda").toFloat();
  if (!isfinite(value) || value < 0.0f) {
    server.send(400, "text/plain", "Carrier CdA must be a non-negative number.");
    return;
  }
  saveCarrierCdAToStorage(value);
  server.send(200, "application/json", buildCarrierCdaJson());
}
void handleCarrierCdaReset() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot reset carrier CdA while a Mode 1/2 test is active.");
    return;
  }
  resetCarrierCdAToDefault();
  server.send(200, "application/json", buildCarrierCdaJson());
}
void handleCalibSetPwm() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot run calibration while a Mode 1/2 test is active. Use STOP first.");
    return;
  }
  if (!server.hasArg("percent")) {
    server.send(400, "text/plain", "Missing percent parameter.");
    return;
  }
  int requestedPercent = server.arg("percent").toInt();
  if (requestedPercent < 0 || requestedPercent > 100 || requestedPercent % CALIBRATION_STEP_PERCENT != 0) {
    server.send(400, "text/plain", "percent must be one of 0,10,20,...,100.");
    return;
  }
  currentTestState = TEST_IDLE;
  currentTestMode = MODE_NONE;
  calibrationModeActive = true;
  calibrationRunningPercent = requestedPercent;
  setMotorSpeed(requestedPercent);
  server.send(200, "text/plain", "Motors set to " + String(requestedPercent) + "% for calibration.");
}
void handleCalibStop() {
  lastBrowserContact = millis();
  calibrationModeActive = false;
  calibrationRunningPercent = -1;
  stopAllMotors();
  server.send(200, "text/plain", "Calibration motors stopped.");
}
void handleCalibSave() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot save calibration while a Mode 1/2 test is active.");
    return;
  }
  bool anyProvided = false;
  for (int i = 0; i < CALIBRATION_POINT_COUNT; ++i) {
    String key = "p" + String(calibrationTable[i].motorPercent);
    if (server.hasArg(key)) {
      float value = server.arg(key).toFloat();
      if (!isfinite(value) || value < 0.0f) {
        server.send(400, "text/plain", key + " must be a non-negative number.");
        return;
      }
      calibrationTable[i].velocityKmh = value;
      calibrationTable[i].hasValue = true;
      anyProvided = true;
    }
  }
  if (!anyProvided) {
    server.send(400, "text/plain", "No calibration values were provided.");
    return;
  }
  saveCalibrationToStorage();
  refreshCalibrationValidity();
  server.send(200, "application/json", buildCalibrationJson());
}
void handleCalibReset() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot reset calibration while a Mode 1/2 test is active.");
    return;
  }
  resetCalibrationToDefaults();
  refreshCalibrationValidity();
  server.send(200, "application/json", buildCalibrationJson());
}
void handleData() {
  lastBrowserContact = millis();
  float countdownVal = 0.0;
  unsigned long elapsed = millis() - stateStartTime;
  if (currentTestState == TEST_ACCELERATING) {
    countdownVal = (RAMP_DURATION_MS - min(elapsed, RAMP_DURATION_MS)) / 1000.0f;
  } else if (currentTestState == TEST_PREPARATION) {
    countdownVal = (PREP_DURATION_MS - min(elapsed, PREP_DURATION_MS)) / 1000.0f;
  } else if (currentTestState == TEST_MEASURING) {
    countdownVal = (MEASURE_DURATION_MS - min(elapsed, MEASURE_DURATION_MS)) / 1000.0f;
  }
  float estimatedVelocity = calibrationValid ? motorPercentToVelocity((float)motorSpeedPercent) : NAN;
  float liveCarrierOffset = interpolatedCarrierOffsetN((float)motorSpeedPercent);
  float netForceN = displayedForceN - liveCarrierOffset;
  unsigned long forceAgeMs = (lastForceUpdateMs > 0) ? (millis() - lastForceUpdateMs) : 0;
  String json = "{";
  json.reserve(currentTestState == TEST_COMPLETED ? 50000 : 1500);
  json += "\"ready\":" + String(sensorConnected ? "true" : "false");
  json += ",\"calibrated\":" + String(calibrated ? "true" : "false");
  json += ",\"calibrationValid\":" + String(calibrationValid ? "true" : "false");
  {
    String safeCalibError = calibrationErrorMessage;
    safeCalibError.replace("\\", "\\\\");
    safeCalibError.replace("\"", "\\\"");
    json += ",\"calibrationErrorMessage\":\"" + safeCalibError + "\"";
  }
  json += ",\"calibrationModeActive\":" + String(calibrationModeActive ? "true" : "false");
  json += ",\"calibrationRunningPercent\":" + String(calibrationRunningPercent);
  json += ",\"force\":" + String(displayedForceN, 3);
  json += ",\"netForceN\":" + String(netForceN, 3);
  json += ",\"liveCarrierOffsetN\":" + String(liveCarrierOffset, 3);
  json += ",\"forceAgeMs\":" + String(forceAgeMs);
  json += ",\"speed\":" + String(motorSpeedPercent);
  json += ",\"estimatedVelocityKmh\":" + String(isfinite(estimatedVelocity) ? estimatedVelocity : -1.0f, 2);
  json += ",\"state\":" + String((int)currentTestState);
  json += ",\"mode\":" + String((int)currentTestMode);
  json += ",\"countdown\":" + String(countdownVal, 1);
  json += ",\"cancelled\":" + String(testWasCancelled ? "true" : "false");
  // Currently active stage target (Mode 1: the single requested velocity;
  // Mode 2: whichever sweep stage is active right now).
  float curTargetVelocityKmh = -1.0f, curTargetVelocityMps = -1.0f, curRequestedPwm = -1.0f;
  int curTargetPwm = -1;
  if (currentTestMode == MODE_ONE) {
    curTargetVelocityKmh = testTargetVelocityKmh;
    curTargetVelocityMps = testVelocityMps;
    curRequestedPwm = testRequiredPwmPercent;
    curTargetPwm = rampTargetPercent;
  } else if (currentTestMode == MODE_TWO) {
    size_t safeIndex = min(mode2StageIndex, MODE2_SPEED_COUNT - 1);
    curTargetVelocityKmh = mode2Targets[safeIndex].velocityKmh;
    curTargetVelocityMps = mode2Targets[safeIndex].velocityMps;
    curRequestedPwm = mode2Targets[safeIndex].requestedPwmPercent;
    curTargetPwm = mode2Targets[safeIndex].motorPercent;
  }
  json += ",\"targetVelocityKmh\":" + String(curTargetVelocityKmh, 2);
  json += ",\"targetVelocityMps\":" + String(curTargetVelocityMps, 3);
  json += ",\"targetPwm\":" + String(curTargetPwm);
  json += ",\"requestedPwmPercent\":" + String(curRequestedPwm, 2);
  if (currentTestMode == MODE_TWO) {
    size_t safeIndex = min(mode2StageIndex, MODE2_SPEED_COUNT - 1);
    float nextKmh = safeIndex + 1 < MODE2_SPEED_COUNT
                      ? mode2Targets[safeIndex + 1].velocityKmh : -1.0f;
    float stageFraction = 0.0f;
    if (currentTestState == TEST_ACCELERATING) {
      stageFraction = min(1.0f, (float)elapsed / RAMP_DURATION_MS) * (3.0f / 18.0f);
    } else if (currentTestState == TEST_PREPARATION) {
      stageFraction = (3.0f + min(5.0f, (float)elapsed / 1000.0f)) / 18.0f;
    } else if (currentTestState == TEST_MEASURING) {
      stageFraction = (8.0f + min(10.0f, (float)elapsed / 1000.0f)) / 18.0f;
    } else if (currentTestState == TEST_COMPLETED) {
      stageFraction = 1.0f;
    }
    float overallProgress = currentTestState == TEST_COMPLETED
      ? 100.0f
      : 100.0f * ((float)safeIndex + stageFraction) / MODE2_SPEED_COUNT;
    json += ",\"position\":" + String((int)safeIndex + 1);
    json += ",\"nextVelocityKmh\":" + String(nextKmh, 2);
    json += ",\"progress\":" + String(overallProgress, 1);
  }
  if (currentTestState == TEST_ERROR) {
    String safeError = testErrorMessage;
    safeError.replace("\\", "\\\\");
    safeError.replace("\"", "\\\"");
    json += ",\"error\":\"" + safeError + "\"";
  }
  if (currentTestState == TEST_COMPLETED &&
      currentTestMode == MODE_ONE && latestResult.valid) {
    json += ",\"result\":{";
    json += "\"targetVelocityKmh\":" + String(latestResult.targetVelocityKmh, 2);
    json += ",\"velocityMps\":" + String(latestResult.velocityMps, 3);
    json += ",\"motorPercent\":" + String(latestResult.motorPercent);
    json += ",\"requestedPwmPercent\":" + String(latestResult.requestedPwmPercent, 2);
    json += ",\"airDensity\":" + String(latestResult.airDensity, 3);
    json += ",\"totalReadings\":" + String(latestResult.totalReadings);
    json += ",\"acceptedReadings\":" + String(latestResult.acceptedReadings);
    json += ",\"excludedReadings\":" + String(latestResult.excludedReadings);
    json += ",\"avgForce\":" + String(latestResult.avgForce, 3);
    json += ",\"medianForce\":" + String(latestResult.medianForce, 3);
    json += ",\"minForce\":" + String(latestResult.minForce, 3);
    json += ",\"maxForce\":" + String(latestResult.maxForce, 3);
    json += ",\"stdDev\":" + String(latestResult.stdDev, 3);
    json += ",\"aeroPower\":" + String(latestResult.aeroPower, 2);
    json += ",\"cdA\":" + String(latestResult.cdA, 5);
    json += ",\"cdaExcludingCarrier\":" + String(latestResult.cdaExcludingCarrier, 5);
    json += ",\"carrierOffsetAppliedN\":" + String(latestResult.carrierOffsetAppliedN, 4);
    json += ",\"rawForces\":[";
    for (size_t i = 0; i < latestResult.rawForces.size(); ++i) {
      if (i) json += ",";
      json += String(latestResult.rawForces[i], 3);
    }
    json += "],\"sampleTimesMs\":[";
    for (size_t i = 0; i < latestResult.sampleTimesMs.size(); ++i) {
      if (i) json += ",";
      json += String(latestResult.sampleTimesMs[i]);
    }
    json += "],\"acceptedMask\":[";
    for (size_t i = 0; i < latestResult.acceptedMask.size(); ++i) {
      if (i) json += ",";
      json += String(latestResult.acceptedMask[i]);
    }
    json += "]";
    json += ",\"valid\":true";
    json += "}";
  }
  if (currentTestState == TEST_COMPLETED &&
      currentTestMode == MODE_TWO && mode2ResultValid) {
    json += ",\"mode2Result\":{";
    json += "\"airDensity\":" + String(testAirDensity, 3);
    json += ",\"averageCdA\":" + String(mode2AverageCdA, 5);
    json += ",\"averageNetCdA\":" + String(mode2AverageNetCdA, 5);
    json += ",\"averageForce\":" + String(mode2AverageForce, 3);
    json += ",\"averagePower\":" + String(mode2AveragePower, 2);
    json += ",\"totalDurationMs\":" + String(mode2TotalDurationMs);
    json += ",\"totalAccepted\":" + String(mode2TotalAccepted);
    json += ",\"totalExcluded\":" + String(mode2TotalExcluded);
    json += ",\"stages\":[";
    for (size_t i = 0; i < MODE2_SPEED_COUNT; ++i) {
      if (i) json += ",";
      const Mode2StageResult& stage = mode2Results[i];
      json += "{";
      json += "\"velocityKmh\":" + String(stage.velocityKmh, 2);
      json += ",\"velocityMps\":" + String(stage.velocityMps, 3);
      json += ",\"motorPercent\":" + String(stage.motorPercent);
      json += ",\"requestedPwmPercent\":" + String(stage.requestedPwmPercent, 2);
      json += ",\"totalReadings\":" + String(stage.totalReadings);
      json += ",\"acceptedReadings\":" + String(stage.acceptedReadings);
      json += ",\"excludedReadings\":" + String(stage.excludedReadings);
      json += ",\"avgForce\":" + String(stage.avgForce, 3);
      json += ",\"medianForce\":" + String(stage.medianForce, 3);
      json += ",\"minForce\":" + String(stage.minForce, 3);
      json += ",\"maxForce\":" + String(stage.maxForce, 3);
      json += ",\"stdDev\":" + String(stage.stdDev, 3);
      json += ",\"aeroPower\":" + String(stage.aeroPower, 2);
      json += ",\"cdA\":" + String(stage.cdA, 5);
      json += ",\"cdaExcludingCarrier\":" + String(stage.cdaExcludingCarrier, 5);
      json += ",\"carrierOffsetAppliedN\":" + String(stage.carrierOffsetAppliedN, 4);
      json += "}";
    }
    json += "],\"valid\":true}";
  }
  json += "}";
  server.send(200, "application/json", json);
}
void handleStartTest() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(400, "text/plain", "Test already in progress.");
    return;
  }
  if (calibrationModeActive) {
    server.send(409, "text/plain", "Cannot start: calibration mode is active. Press STOP MOTORS in the calibration tab first.");
    return;
  }
  if (!sensorConnected) {
    server.send(503, "text/plain", "Cannot start: wireless force-sensor node is not responding.");
    return;
  }
  if (!calibrated) {
    server.send(400, "text/plain", "Cannot start: calibrate the force sensor first.");
    return;
  }
  if (!calibrationValid) {
    server.send(400, "text/plain",
                "Cannot start: the air velocity calibration is invalid. "
                "Open the Air Velocity Calibration tab and fix it.");
    return;
  }
  if (!server.hasArg("velocity")) {
    server.send(400, "text/plain", "Missing target air velocity.");
    return;
  }
  float reqVelocityKmh = server.arg("velocity").toFloat();
  float reqDensity = server.hasArg("density") ? server.arg("density").toFloat() : 1.225;
  if (!isfinite(reqVelocityKmh) ||
      reqVelocityKmh < calibrationMinVelocityKmh() ||
      reqVelocityKmh > calibrationMaxVelocityKmh()) {
    server.send(400, "text/plain",
                "Target air velocity must be between " +
                String(calibrationMinVelocityKmh(), 1) + " and " +
                String(calibrationMaxVelocityKmh(), 1) + " km/h.");
    return;
  }
  if (!isfinite(reqDensity) || reqDensity <= 0.0) {
    server.send(400, "text/plain", "Invalid air density.");
    return;
  }
  float requiredPwm = velocityToMotorPercent(reqVelocityKmh);
  if (isnan(requiredPwm)) {
    server.send(400, "text/plain", "Could not calculate a motor PWM for that velocity.");
    return;
  }
  testTargetVelocityKmh = reqVelocityKmh;
  testVelocityMps = reqVelocityKmh / 3.6f;
  testRequiredPwmPercent = requiredPwm;
  testAirDensity = reqDensity;
  recordedRawForces.clear();
  recordedRawForces.reserve(MAX_TEST_SAMPLES);
  recordedSampleTimesMs.clear();
  recordedSampleTimesMs.reserve(MAX_TEST_SAMPLES);
  latestResult.valid = false;
  latestResult.rawForces.clear();
  latestResult.sampleTimesMs.clear();
  latestResult.acceptedMask.clear();
  testErrorMessage = "";
  testWasCancelled = false;
  newSensorSample = false;
  currentTestMode = MODE_ONE;
  currentTestState = TEST_ACCELERATING;
  stateStartTime = millis();
  testStartTime = stateStartTime;
  rampStartPercent = 0;
  rampTargetPercent = (int)roundf(requiredPwm);  // round only when sending to the motor
  setMotorSpeed(0);
  server.send(200, "text/plain", "Test initiated.");
}
void handleStartMode2() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Another test is already in progress.");
    return;
  }
  if (calibrationModeActive) {
    server.send(409, "text/plain", "Cannot start: calibration mode is active. Press STOP MOTORS in the calibration tab first.");
    return;
  }
  if (!sensorConnected) {
    server.send(503, "text/plain", "Cannot start: wireless force-sensor node is not responding.");
    return;
  }
  if (!calibrated) {
    server.send(400, "text/plain", "Cannot start: calibrate the force sensor first.");
    return;
  }
  if (!calibrationValid) {
    server.send(400, "text/plain",
                "Cannot start: the air velocity calibration is invalid. "
                "Open the Air Velocity Calibration tab and fix it.");
    return;
  }
  float requestedDensity = server.hasArg("density")
    ? server.arg("density").toFloat() : 1.225f;
  if (!isfinite(requestedDensity) || requestedDensity <= 0.0f) {
    server.send(400, "text/plain", "Invalid air density.");
    return;
  }
  for (size_t i = 0; i < MODE2_SPEED_COUNT; ++i) {
    float velocityKmh = MODE2_TARGET_VELOCITIES_KMH[i];
    float pwm = velocityToMotorPercent(velocityKmh);
    if (isnan(pwm)) {
      server.send(400, "text/plain",
                  "The calibration table cannot produce a PWM for " +
                  String(velocityKmh, 0) + " km/h. Sweep aborted.");
      return;
    }
    mode2Targets[i].velocityKmh = velocityKmh;
    mode2Targets[i].velocityMps = velocityKmh / 3.6f;
    mode2Targets[i].requestedPwmPercent = pwm;
    mode2Targets[i].motorPercent = (int)roundf(pwm);
  }
  for (size_t i = 0; i < MODE2_SPEED_COUNT; ++i) {
    mode2Results[i] = Mode2StageResult();
    mode2Results[i].valid = false;
    mode2Results[i].rawForces.clear();
  }
  testAirDensity = requestedDensity;
  mode2StageIndex = 0;
  mode2AverageCdA = 0.0f;
  mode2AverageForce = 0.0f;
  mode2AveragePower = 0.0f;
  mode2TotalAccepted = 0;
  mode2TotalExcluded = 0;
  mode2TotalDurationMs = 0;
  mode2ResultValid = false;
  recordedRawForces.clear();
  recordedRawForces.reserve(MAX_TEST_SAMPLES);
  recordedSampleTimesMs.clear();
  testErrorMessage = "";
  testWasCancelled = false;
  newSensorSample = false;
  currentTestMode = MODE_TWO;
  currentTestState = TEST_ACCELERATING;
  stateStartTime = millis();
  testStartTime = stateStartTime;
  rampStartPercent = 0;
  rampTargetPercent = mode2Targets[0].motorPercent;
  setMotorSpeed(0);
  server.send(200, "text/plain", "Mode 2 automatic speed sweep initiated.");
}
void handleStop() {
  lastBrowserContact = millis();
  calibrationModeActive = false;
  calibrationRunningPercent = -1;
  if (testIsActive()) {
    abortActiveTest("Emergency stop pressed. The active test was cancelled.", true);
    server.send(200, "text/plain", "All motors stopped and the test was cancelled.");
  } else {
    currentTestState = TEST_IDLE;
    currentTestMode = MODE_NONE;
    testErrorMessage = "";
    testWasCancelled = false;
    stopAllMotors();
    server.send(200, "text/plain", "All motors stopped.");
  }
}
void handleManualSpeed() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot set motor speed manually while an automatic test is running. Use the stop button first.");
    return;
  }
  if (!server.hasArg("speed")) {
    server.send(400, "text/plain", "Missing speed parameter.");
    return;
  }
  int requestedPercent = server.arg("speed").toInt();
  if (requestedPercent < 0 || requestedPercent > 100) {
    server.send(400, "text/plain", "Speed must be between 0 and 100.");
    return;
  }
  currentTestState = TEST_IDLE;
  currentTestMode = MODE_NONE;
  calibrationModeActive = false;
  calibrationRunningPercent = -1;
  testErrorMessage = "";
  testWasCancelled = false;
  setMotorSpeed(requestedPercent);
  server.send(200, "text/plain", "Motor speed manually set to " + String(requestedPercent) + "%.");
}
void handleTare() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot tare while a test is running. Use emergency stop first.");
    return;
  }
  stopAllMotors();
  calibrationModeActive = false;
  calibrationRunningPercent = -1;
  currentTestState = TEST_IDLE;
  currentTestMode = MODE_NONE;
  String responseText;
  String path = "/tare?token=" + String(SENSOR_TOKEN);
  bool success = sendCommandToSensor(path, responseText);
  if (success) {
    displayedForceN = 0.0f;
  }
  server.send(success ? 200 : 503, "text/plain", responseText);
}
void handleCalibration() {
  lastBrowserContact = millis();
  if (testIsActive()) {
    server.send(409, "text/plain", "Cannot calibrate while a test is running. Use emergency stop first.");
    return;
  }
  stopAllMotors();
  calibrationModeActive = false;
  calibrationRunningPercent = -1;
  currentTestState = TEST_IDLE;
  currentTestMode = MODE_NONE;
  if (!server.hasArg("mass")) {
    server.send(400, "text/plain", "Enter a calibration mass.");
    return;
  }
  float knownMassKg = server.arg("mass").toFloat();
  if (knownMassKg <= 0.0) {
    server.send(400, "text/plain", "Invalid calibration mass.");
    return;
  }
  String responseText;
  String path = "/calibrate?token=" + String(SENSOR_TOKEN) +
                "&mass=" + String(knownMassKg, 4);
  bool success = sendCommandToSensor(path, responseText);
  server.send(success ? 200 : 503, "text/plain", responseText);
}
// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  loadCalibrationFromStorage();
  refreshCalibrationValidity();
  loadCarrierOffsetFromStorage();
  Serial.println("Carrier-body offset: " + String(carrierOffsetCalibratedCount()) +
                  " of " + String(CALIBRATION_POINT_COUNT) + " points calibrated" +
                  (carrierOffsetCalibratedCount() == 0 ? " (offset = 0 N everywhere)." : "."));
  loadCarrierCdAFromStorage();
  Serial.println(carrierBodyCdASet
    ? "Carrier-body CdA offset: " + String(carrierBodyCdA, 5) + " m^2."
    : "Carrier-body CdA offset: not set (0 m^2, no effect on CdA results).");
  if (!calibrationValid) {
    Serial.println("WARNING: Air velocity calibration is incomplete/invalid: " + calibrationErrorMessage +
                    " Velocity-based Mode 1 and Mode 2 tests are disabled until it is fixed from the "
                    "Air Velocity Calibration tab on the dashboard.");
  } else {
    Serial.println("Air velocity calibration OK: " + String(CALIBRATION_POINT_COUNT) +
                    " points, " + String(calibrationMinVelocityKmh(), 2) + "-" +
                    String(calibrationMaxVelocityKmh(), 2) + " km/h.");
  }
  for (size_t i = 0; i < MODE2_SPEED_COUNT; ++i) {
    mode2Results[i].valid = false;
  }
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(MOTOR_PWM_PIN, HIGH);
  pwmConfigured = ledcAttach(MOTOR_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  if (pwmConfigured) {
    ledcWrite(MOTOR_PWM_PIN, MOTOR_OFF_PWM);
    Serial.println("Motor PWM configured.");
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_NAME, WIFI_PASSWORD, 1);
  Serial.print("Dashboard address: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("Waiting for wireless force-sensor node...");
  server.on("/", HTTP_GET, handleMainPage);
  server.on("/data", HTTP_GET, handleData);
  server.on("/velocity_table", HTTP_GET, handleVelocityTable);
  server.on("/calib_get", HTTP_GET, handleCalibGet);
  server.on("/calib_set_pwm", HTTP_POST, handleCalibSetPwm);
  server.on("/calib_stop", HTTP_POST, handleCalibStop);
  server.on("/calib_save", HTTP_POST, handleCalibSave);
  server.on("/calib_reset", HTTP_POST, handleCalibReset);
  server.on("/carrier_offset_get", HTTP_GET, handleCarrierOffsetGet);
  server.on("/carrier_offset_save", HTTP_POST, handleCarrierOffsetSave);
  server.on("/carrier_offset_reset", HTTP_POST, handleCarrierOffsetReset);
  server.on("/carrier_cda_get", HTTP_GET, handleCarrierCdaGet);
  server.on("/carrier_cda_save", HTTP_POST, handleCarrierCdaSave);
  server.on("/carrier_cda_reset", HTTP_POST, handleCarrierCdaReset);
  server.on("/start_test", HTTP_POST, handleStartTest);
  server.on("/start_mode2", HTTP_POST, handleStartMode2);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/manual_speed", HTTP_POST, handleManualSpeed);
  server.on("/tare", HTTP_POST, handleTare);
  server.on("/calibrate", HTTP_POST, handleCalibration);
  server.on("/sensor_update", HTTP_POST, handleSensorUpdate);
  server.begin();
  lastBrowserContact = millis();
}
// ==================================================
// MAIN LOOP & STATE MACHINE
// ==================================================
void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();
  // 1. Consume each fresh wireless force sample exactly once.
  if (newSensorSample) {
    float sampleForceN = pendingSampleForceN;
    newSensorSample = false;
    if (currentTestState == TEST_MEASURING &&
        currentMillis - stateStartTime < MEASURE_DURATION_MS) {
      if (recordedRawForces.size() >= MAX_TEST_SAMPLES) {
        abortActiveTest("Too many wireless sensor samples received.");
      } else {
        recordedRawForces.push_back(sampleForceN);
        if (currentTestMode == MODE_ONE) {
          recordedSampleTimesMs.push_back(currentMillis - stateStartTime);
        }
      }
    }
  }
  // 2. A missing sensor during an active test is a safety fault.
  if (currentMillis - lastGoodSensorReading > SENSOR_TIMEOUT_MS) {
    sensorConnected = false;
    if (testIsActive()) {
      abortActiveTest("Wireless force-sensor signal was lost. Motors were stopped automatically.");
    }
  }
  // 3. Shared non-blocking Mode 1 / Mode 2 execution sequence.
  switch (currentTestState) {
    case TEST_IDLE:
    case TEST_COMPLETED:
    case TEST_ERROR:
      break;
    // Stage 1: three-second linear ramp from the previous PWM to the target.
    case TEST_ACCELERATING: {
      unsigned long elapsed = currentMillis - stateStartTime;
      if (elapsed < RAMP_DURATION_MS) {
        float progress = (float)elapsed / (float)RAMP_DURATION_MS;
        int rampSpeed = (int)roundf(
          rampStartPercent +
          progress * (rampTargetPercent - rampStartPercent));
        setMotorSpeed(rampSpeed);
      } else {
        setMotorSpeed(rampTargetPercent);
        currentTestState = TEST_PREPARATION;
        stateStartTime = currentMillis;
      }
      break;
    }
    // Stage 2: five-second preparation interval at constant PWM.
    case TEST_PREPARATION: {
      setMotorSpeed(rampTargetPercent);
      unsigned long elapsed = currentMillis - stateStartTime;
      if (elapsed >= PREP_DURATION_MS) {
        recordedRawForces.clear();
        recordedSampleTimesMs.clear();
        newSensorSample = false;  // Discard the last pre-measurement packet.
        currentTestState = TEST_MEASURING;
        stateStartTime = currentMillis;
      }
      break;
    }
    // Stage 3: ten-second measurement interval at constant PWM.
    case TEST_MEASURING: {
      setMotorSpeed(rampTargetPercent);
      unsigned long elapsed = currentMillis - stateStartTime;
      if (elapsed >= MEASURE_DURATION_MS) {
        if (currentTestMode == MODE_ONE) {
          stopAllMotors();
          if (processMode1Results()) {
            currentTestState = TEST_COMPLETED;
          } else {
            currentTestState = TEST_ERROR;
          }
        } else if (currentTestMode == MODE_TWO) {
          if (!processCurrentMode2Stage()) {
            stopAllMotors();
            currentTestState = TEST_ERROR;
            break;
          }
          if (mode2StageIndex + 1 >= MODE2_SPEED_COUNT) {
            stopAllMotors();
            finishMode2Results(currentMillis);
            currentTestState = TEST_COMPLETED;
          } else {
            // Keep the motors running and ramp directly to the next stage.
            ++mode2StageIndex;
            recordedRawForces.clear();
            recordedSampleTimesMs.clear();
            rampStartPercent = motorSpeedPercent;
            rampTargetPercent = mode2Targets[mode2StageIndex].motorPercent;
            currentTestState = TEST_ACCELERATING;
            stateStartTime = currentMillis;
          }
        } else {
          abortActiveTest("Invalid test mode.");
        }
      }
      break;
    }
  }
  // 4. Debug output every 500ms
  if (currentMillis - lastSerialPrint >= 500) {
    lastSerialPrint = currentMillis;
    Serial.printf("State: %d | Speed: %d%% | Force: %.2f N\n", (int)currentTestState, motorSpeedPercent, displayedForceN);
  }
  delay(2);
}
