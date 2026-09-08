firmware/Force_Sensor_ESP32_Mini/
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <math.h>
#include "HX711.h"

// ==================================================
// HX711 PINS
// ==================================================
// ESP32-C3 SuperMini: DOUT -> GPIO3, SCK -> GPIO4
// Classic ESP32 Mini/D1 Mini: DOUT -> GPIO25, SCK -> GPIO26

const int HX711_DOUT = 3;
const int HX711_SCK  = 4;

// HX711 VCC -> this ESP32 Mini 3.3 V
// HX711 GND -> this ESP32 Mini GND
// Do NOT connect this ground to the motor-controller ground.

// Place these directly beside HX711 VCC/GND:
// 100 nF ceramic capacitor (no polarity)
// 100 uF electrolytic (+ to VCC, striped - leg to GND)

// ==================================================
// WIRELESS SETTINGS — MUST MATCH MAIN ESP32
// ==================================================
const char* WIFI_NAME = "Mighty-Airflow-Control";
const char* WIFI_PASSWORD = "Mighty2026";
const char* SENSOR_TOKEN = "MightySensor2026";
const char* MAIN_ESP32_ADDRESS = "192.168.4.1";

// ==================================================
// FORCE SETTINGS
// ==================================================
const float GRAVITY = 9.80665f;
const float FILTER_ALPHA = 0.08f;
const float ZERO_DEAD_ZONE_N = 0.30f;
const unsigned long SENSOR_TIMEOUT_MS = 2000;
const unsigned long STATUS_PACKET_INTERVAL_MS = 500;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

HX711 scale;
WebServer commandServer(80);
Preferences preferences;

float calibrationFactor = 0.0f;
float filteredSignedForceN = 0.0f;
float displayedForceN = 0.0f;
float instantaneousForceN = 0.0f;

long rawReading = 0;
bool calibrated = false;
bool sensorConnected = false;
bool firstForceReading = true;

uint32_t sampleSequence = 0;
unsigned long lastGoodSensorReading = 0;
unsigned long lastStatusPacket = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastSerialPrint = 0;

// ==================================================
// HX711 HELPERS
// ==================================================
bool waitForHX711(unsigned long timeoutMs) {
  unsigned long startTime = millis();

  while (!scale.is_ready()) {
    if (millis() - startTime >= timeoutMs) {
      return false;
    }

    delay(2);
  }

  return true;
}

bool requestAuthorized() {
  if (!commandServer.hasArg("token") ||
      commandServer.arg("token") != SENSOR_TOKEN) {
    commandServer.send(403, "text/plain", "Forbidden");
    return false;
  }

  return true;
}

// ==================================================
// SEND SENSOR DATA TO MAIN ESP32
// ==================================================
void sendSensorPacket() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(300);
  http.setTimeout(500);

  String url;
  url.reserve(300);
  url = "http://";
  url += MAIN_ESP32_ADDRESS;
  url += "/sensor_update?token=";
  url += SENSOR_TOKEN;
  url += "&ready=";
  url += sensorConnected ? "1" : "0";
  url += "&calibrated=";
  url += calibrated ? "1" : "0";
  url += "&force=";
  url += String(displayedForceN, 4);
  url += "&sample_force=";
  url += String(instantaneousForceN, 4);
  url += "&sequence=";
  url += String(sampleSequence);

  if (http.begin(client, url)) {
    http.POST("");
    http.end();
  }

  lastStatusPacket = millis();
}

// ==================================================
// COMMANDS RECEIVED FROM THE DASHBOARD ESP32
// ==================================================
void handleRoot() {
  commandServer.send(
    200,
    "text/plain",
    "Wireless HX711 sensor node is running. Use the main dashboard at http://192.168.4.1"
  );
}

void handleTare() {
  if (!requestAuthorized()) {
    return;
  }

  if (!waitForHX711(2000)) {
    sensorConnected = false;
    commandServer.send(503, "text/plain", "HX711 is not responding.");
    return;
  }

  scale.tare(20);
  filteredSignedForceN = 0.0f;
  displayedForceN = 0.0f;
  instantaneousForceN = 0.0f;
  firstForceReading = true;
  sensorConnected = true;
  lastGoodSensorReading = millis();
  sampleSequence++;

  commandServer.send(200, "text/plain", "Force zero successfully set on sensor node.");
}

void handleCalibration() {
  if (!requestAuthorized()) {
    return;
  }

  if (!commandServer.hasArg("mass")) {
    commandServer.send(400, "text/plain", "Calibration mass is missing.");
    return;
  }

  float knownMassKg = commandServer.arg("mass").toFloat();

  if (knownMassKg <= 0.0f) {
    commandServer.send(400, "text/plain", "Invalid calibration mass.");
    return;
  }

  if (!waitForHX711(2000)) {
    sensorConnected = false;
    commandServer.send(503, "text/plain", "HX711 is not responding.");
    return;
  }

  long loadedReading = scale.read_average(20);
  long zeroOffset = scale.get_offset();
  long calibrationSpan = loadedReading - zeroOffset;
  float knownForceN = knownMassKg * GRAVITY;

  if (labs(calibrationSpan) < 50) {
    commandServer.send(
      400,
      "text/plain",
      "Calibration failed: load-cell reading changed too little."
    );
    return;
  }

  calibrationFactor = calibrationSpan / knownForceN;

  if (!isfinite(calibrationFactor) ||
      fabsf(calibrationFactor) < 0.01f) {
    commandServer.send(400, "text/plain", "Invalid calibration factor.");
    return;
  }

  scale.set_scale(calibrationFactor);
  preferences.putFloat("factor", calibrationFactor);

  calibrated = true;
  sensorConnected = true;
  firstForceReading = true;
  filteredSignedForceN = knownForceN;
  displayedForceN = knownForceN;
  instantaneousForceN = knownForceN;
  lastGoodSensorReading = millis();
  sampleSequence++;

  String response = "Calibration completed on sensor node: ";
  response += String(knownForceN, 2);
  response += " N";
  commandServer.send(200, "text/plain", response);
}

// ==================================================
// WI-FI
// ==================================================
void connectToMotorESP32() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  lastWiFiAttempt = millis();
  WiFi.disconnect();
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  Serial.print("Connecting to ");
  Serial.println(WIFI_NAME);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  preferences.begin("forceSensor", false);
  scale.begin(HX711_DOUT, HX711_SCK);

  Serial.print("HX711 DOUT pin: GPIO");
  Serial.println(HX711_DOUT);
  Serial.print("HX711 SCK pin: GPIO");
  Serial.println(HX711_SCK);

  if (waitForHX711(3000)) {
    sensorConnected = true;
    lastGoodSensorReading = millis();

    calibrationFactor = preferences.getFloat("factor", 0.0f);

    if (isfinite(calibrationFactor) &&
        fabsf(calibrationFactor) > 0.01f) {
      scale.set_scale(calibrationFactor);
      calibrated = true;
      Serial.println("Stored calibration factor loaded.");
    } else {
      scale.set_scale(1.0f);
      calibrated = false;
      Serial.println("Calibration required.");
    }

    Serial.println("Remove all force. Setting startup zero...");
    scale.tare(20);
    Serial.println("Startup zero set.");
  } else {
    sensorConnected = false;
    Serial.println("HX711 not detected.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  connectToMotorESP32();

  unsigned long connectionStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - connectionStart < 15000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Sensor-node address: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi not connected yet. Automatic retry enabled.");
  }

  commandServer.on("/", HTTP_GET, handleRoot);
  commandServer.on("/tare", HTTP_POST, handleTare);
  commandServer.on("/calibrate", HTTP_POST, handleCalibration);
  commandServer.begin();
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  commandServer.handleClient();
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED &&
      currentMillis - lastWiFiAttempt >= WIFI_RETRY_INTERVAL_MS) {
    connectToMotorESP32();
  }

  bool newConversion = false;

  if (scale.is_ready()) {
    rawReading = scale.read();

    bool rawReadingValid =
      rawReading != 0 &&
      rawReading != 8388607L &&
      rawReading != -8388608L;

    if (rawReadingValid) {
      sensorConnected = true;
      lastGoodSensorReading = currentMillis;
      newConversion = true;

      if (calibrated) {
        float newSignedForceN =
          (rawReading - scale.get_offset()) / calibrationFactor;

        if (isfinite(newSignedForceN)) {
          if (firstForceReading) {
            filteredSignedForceN = newSignedForceN;
            firstForceReading = false;
          } else {
            filteredSignedForceN =
              FILTER_ALPHA * newSignedForceN +
              (1.0f - FILTER_ALPHA) * filteredSignedForceN;
          }

          if (fabsf(filteredSignedForceN) < ZERO_DEAD_ZONE_N) {
            filteredSignedForceN = 0.0f;
          }

          // Normal force is treated as a magnitude after signed filtering.
          displayedForceN = fabsf(filteredSignedForceN);
          instantaneousForceN = fabsf(newSignedForceN);
          sampleSequence++;
        }
      } else {
        displayedForceN = 0.0f;
        instantaneousForceN = 0.0f;
      }
    }
  }

  if (currentMillis - lastGoodSensorReading > SENSOR_TIMEOUT_MS) {
    sensorConnected = false;
  }

  if (newConversion ||
      currentMillis - lastStatusPacket >= STATUS_PACKET_INTERVAL_MS) {
    sendSensorPacket();
  }

  
  delay(2);
}
