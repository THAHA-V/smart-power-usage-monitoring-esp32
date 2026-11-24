/*
 THIS PROJECT IS MADE BY SST TEAM
 
 MEMEBERS : 1.SABITH
            2.SHAHSAD
            3.THAHA
 
  ESP32 Energy Monitor -> Local PUMS Web Server (10.195.211.243:5000)
  Pins:
    - Voltage Sensor: GPIO 35
    - Current Sensor (ACS712): GPIO 34
    - Cal Button: GPIO 4
    - I2C LCD: SDA = GPIO 21, SCL = GPIO 22 (Address: 0x3E)

  FIXES IN THIS VERSION
    1. Removed the stray ACS sampling + delay(500) block from loop().
    2. Current is forced to 0 whenever mains voltage is absent (V < 80 V).
    3. Current noise floor raised (150 mA) + spike rejection + low-pass filter.
    4. Current sampled over 10 AC cycles for steadier readings.
    5. Removed the late "Irms < 0.13" zeroing in postData().
    6. Removed duplicate updateLCD() call inside measureAndAccumulate().
    7. Optional automatic zero-point re-calibration while mains is OFF.
    8. Fixed the compile error (stray text after "} else {" in updateLCD()).
*/

#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "LiquidCrystalI2C.h"
#include "ACS712.h"

LiquidCrystalI2C lcd(0x3E, 16, 2);

/* ================= 1. SERVER & WIFI SETTINGS ================= */
const char* ssid        = "sstteam";
const char* pass        = "9746238111";

const char* server_ip   = "10.195.211.243";
const uint16_t server_port = 5000;
const char* DEVICE_ID   = "meter1";

const unsigned long DATA_INTERVAL = 500UL;   // Stream update every 500 ms
const uint16_t HTTP_TIMEOUT_MS    = 1000;

WiFiClient plainClient;

/* ================= 2. BUTTON & PINS ================= */
#define CAL_BUTTON 4
unsigned long buttonPressTime = 0;
bool buttonHeld = false;

#define VOLTAGE_SENSOR_PIN 35
#define CURRENT_SENSOR_PIN 34

// ACS712 5A = 185 mV/A (using 127.2 sensitivity tuning)
ACS712 ACS(CURRENT_SENSOR_PIN, 3.3, 4095, 127.2);

// Voltage sensor sampling window: 100ms (5 complete 50Hz AC cycles)
const unsigned long SAMPLE_WINDOW_US = 100000UL;
const float VOLTAGE_SCALE = 400.0;

/* ================= CURRENT TUNING ================= */
// Readings below this are treated as noise (mA). Set to ~3x your measured noise.
const float CURRENT_NOISE_FLOOR_mA = 150.0;
// Tune with a known load: CURRENT_SCALE = true_current / displayed_current
const float CURRENT_SCALE          = 1.00;
// Number of AC cycles averaged for each current reading
const uint16_t CURRENT_CYCLES      = 10;
// Spike rejection: a sample bigger than this multiple of the filtered value is clamped
const float SPIKE_RATIO            = 3.0;
// Low-pass filter weight of the new sample (0..1). Lower = smoother
const float CURRENT_FILTER_ALPHA   = 0.3;

// Auto re-zero of ACS712 midpoint while mains is OFF (safe: no load can be flowing)
const bool          AUTO_ZERO_WHEN_OFF   = true;
const unsigned long AUTO_ZERO_OFF_MS     = 5000UL;   // mains must be off this long
const unsigned long AUTO_ZERO_INTERVAL   = 60000UL;  // at most once a minute

/* ================= 3. MEASUREMENTS & STATUS ================= */
float vCalibration    = 1.00;
float actualVoltage   = 230.0;
float actualCurrent   = 0.06;
float actualPower     = 0.0;
bool  calibrationRequested = false;

float Vrms = 0.0;
float Irms = 0.0;
float realPower = 0.0;
float powerFactor = 1.00;   // No phase measurement, so power = V x I (apparent power)

float kWh = 0.0;
float cost = 0.0;
const float ratePerkWh = 6.5;
unsigned long lastMillis = 0;

/* ================= 4. DAILY TRACKER & EEPROM ================= */
float dailyKWh = 0.0;
float dailyCost = 0.0;
unsigned long dailyTrackerStartMillis = 0;
const unsigned long dailyTrackerInterval = 86400000UL;

const int addrVcal = 0;
const int addrKWh  = 12;
const int addrCost = 16;

float lastSavedKWh = 0.0;
unsigned long lastEepromSaveMillis = 0;
const unsigned long EEPROM_SAVE_INTERVAL = 300000UL;
const float KWH_SAVE_THRESHOLD = 0.01;

/* ================= 5. TIMERS & STATUS ================= */
int displayPage = 0;
unsigned long lastMeasureMs = 0;
unsigned long lastPageMs = 0;
unsigned long lastDataMs = 0;
unsigned long lastWiFiAttempt = 0;
const unsigned long wifiRetryInterval = 10000;
bool serverOK = false;

float lastGoodVoltage = 0.0;
float lastGoodCurrent = 0.0;
float smoothed_voltage = 0.0;
float iSmoothed = 0.0;               // filtered current (A)

unsigned long mainsOffSinceMs = 0;   // 0 = mains currently present
unsigned long lastAutoZeroMs = 0;

const float VOLTAGE_OFF_THRESHOLD   = 80.0; // Resets cleanly to 0.0V when disconnected
const float MAX_VALID_VOLTAGE       = 300.0;

void updateLCD();
void lcdShow(const char* line0, const char* line1);
void startCalibration();

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  Wire.begin();
  lcd.begin();

  EEPROM.begin(64);
  EEPROM.get(addrVcal, vCalibration);
  if (isnan(vCalibration) || vCalibration <= 0.1 || vCalibration > 10.0) vCalibration = 1.0;

  readEnergyDataFromEEPROM();

  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_SENSOR_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR_PIN, ADC_11db);

  lcd.clear();
  lcdShow("Local Meter Boot", "Starting...");
  delay(1000);

  // ACS712 5A Zero-current calibration (make sure the LOAD IS OFF at boot!)
  lcdShow("Calibrating...", "Load must be OFF");
  Serial.println("ACS712 5A Calibration");
  Serial.println("----------------------");
  Serial.println("Calibrating... (load must be OFF)");
  ACS.autoMidPoint(50, 20);

  Serial.print("Midpoint: ");
  Serial.println(ACS.getMidPoint());

  // Measure noise level (use this to choose CURRENT_NOISE_FLOOR_mA)
  float noise = ACS.mVNoiseLevel(50, 10);
  Serial.print("Noise: ");
  Serial.print(noise, 3);
  Serial.println(" mV");
  Serial.println("Calibration completed.");

  lcdShow("Zero Cal Done", "Current Ready");
  delay(600);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, pass);

  lastMillis = millis();
  dailyTrackerStartMillis = millis();
}

/* ================= MAIN LOOP ================= */
void loop() {
  unsigned long now = millis();

  maintainWiFi();
  checkButtonCalibration();

  // Continuous real-time measurement & LCD update every 100 ms
  if (now - lastMeasureMs >= 100UL) {
    lastMeasureMs = now;
    measureAndAccumulate();
    updateLCD();
  }

  // Stream data to website every 500 ms
  if (now - lastDataMs >= DATA_INTERVAL) {
    lastDataMs = now;
    postData();
  }

  if (calibrationRequested) {
    calibrationRequested = false;
    startCalibration();
  }

  // Re-zero the ACS712 while mains is off to remove drift / fake current
  autoZeroIfMainsOff();
}

/* ================= WIFI ================= */
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastWiFiAttempt >= wifiRetryInterval) {
    lastWiFiAttempt = now;
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, pass);
  }
}

/* ================= SEND DATA TO WEBSITE ================= */
void postData() {
  if (WiFi.status() != WL_CONNECTED) { serverOK = false; return; }

  static HTTPClient http;
  static bool httpInit = false;

  if (!httpInit) {
    String url = String("http://") + server_ip + ":" + String(server_port) + "/api/update";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.setReuse(true);
    http.setTimeout(HTTP_TIMEOUT_MS);
    httpInit = true;
  }

  // Values were already cleaned in measureAndAccumulate(); send them as-is.
  char body[256];
  snprintf(body, sizeof(body),
    "{\"device\":\"%s\",\"voltage\":%.2f,\"current\":%.3f,\"power\":%.2f,\"kwh\":%.5f,\"cost\":%.2f}",
    DEVICE_ID, Vrms, Irms, realPower, kWh, cost);

  int code = http.POST((uint8_t*)body, strlen(body));
  serverOK = (code >= 200 && code < 300);
  if (code < 0) {
    http.end();
    httpInit = false;
  }
}

/* ================= SENSOR SAMPLING ================= */
void sampleSensors(float &voltsRms, float &ampsRms) {
  // 1. Current sensor reading using ACS712 library (10 cycles for stability)
  float current_mA = ACS.mA_AC_sampling(50, CURRENT_CYCLES);
  current_mA *= CURRENT_SCALE;
  if (current_mA < CURRENT_NOISE_FLOOR_mA) {
    current_mA = 0.0f;
  }
  ampsRms = current_mA / 1000.0f; // Convert mA to A

  // 2. Voltage sensor reading on GPIO 35 (ZMPT101B AC RMS)
  double sumV = 0, sumV2 = 0;
  unsigned long n = 0;

  unsigned long start = micros();
  while (micros() - start < SAMPLE_WINDOW_US) {
    double v = analogReadMilliVolts(VOLTAGE_SENSOR_PIN);
    sumV += v;  sumV2 += v * v;
    n++;
  }

  if (n == 0) { voltsRms = 0; return; }

  double varV = (sumV2 / n) - (sumV / n) * (sumV / n);
  if (varV < 0) varV = 0;
  double vRms_mV = sqrt(varV);
  voltsRms = (float)((vRms_mV / 1000.0) * VOLTAGE_SCALE);
}

/* ================= MEASURE ================= */
void measureAndAccumulate() {
  float raw_V, raw_I;
  sampleSensors(raw_V, raw_I);

  float newV = raw_V * vCalibration;
  float newI = raw_I;

  if (isnan(newV) || newV < 0.0) newV = 0.0;
  if (isnan(newI) || newI < 0.0) newI = 0.0;

  // ---------- Voltage filtering ----------
  bool mainsPresent = true;
  if (newV < VOLTAGE_OFF_THRESHOLD) {
    mainsPresent = false;
    newV = 0.0;
    smoothed_voltage = 0.0;
  } else if (newV > MAX_VALID_VOLTAGE) {
    newV = lastGoodVoltage;
  } else {
    if (smoothed_voltage < VOLTAGE_OFF_THRESHOLD) smoothed_voltage = newV;
    else smoothed_voltage = (smoothed_voltage * 0.8f) + (newV * 0.2f);
    newV = smoothed_voltage;
  }

  // Track how long mains has been absent (used by auto-zero)
  if (mainsPresent) mainsOffSinceMs = 0;
  else if (mainsOffSinceMs == 0) mainsOffSinceMs = millis();

  // ---------- Current filtering ----------
  // No mains voltage => no load current can exist => force current to 0 instantly
  if (!mainsPresent || newI <= 0.0f) {
    iSmoothed = 0.0f;
  } else if (iSmoothed <= 0.0f) {
    iSmoothed = newI;                              // first sample after zero
  } else {
    if (newI > iSmoothed * SPIKE_RATIO) newI = iSmoothed * 1.5f;   // reject spike
    iSmoothed = iSmoothed * (1.0f - CURRENT_FILTER_ALPHA) + newI * CURRENT_FILTER_ALPHA;
  }
  Irms = iSmoothed;

  Vrms = newV;
  realPower = Vrms * Irms;   // apparent power (VA); no phase measurement available

  if (Vrms >= VOLTAGE_OFF_THRESHOLD) lastGoodVoltage = Vrms;
  if (Irms > 0.0) lastGoodCurrent = Irms;

  // ---------- Energy accumulation (only when power > 0) ----------
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - lastMillis;
  lastMillis = currentMillis;

  float deltaKWh = (realPower * elapsedMillis) / 3600000000.0;
  if (deltaKWh < 0) deltaKWh = 0;

  kWh += deltaKWh;
  dailyKWh += deltaKWh;
  cost = kWh * ratePerkWh;
  dailyCost = dailyKWh * ratePerkWh;

  handleDailyTracker(currentMillis);
  saveEnergyDataToEEPROM();
  // updateLCD() is called once from loop(); duplicate call removed
}

/* ================= AUTO ZERO (mains OFF only) ================= */
void autoZeroIfMainsOff() {
  if (!AUTO_ZERO_WHEN_OFF) return;
  if (mainsOffSinceMs == 0) return;                       // mains present
  unsigned long now = millis();
  if (now - mainsOffSinceMs < AUTO_ZERO_OFF_MS) return;   // not off long enough
  if (now - lastAutoZeroMs < AUTO_ZERO_INTERVAL) return;  // too soon since last

  lastAutoZeroMs = now;
  ACS.autoMidPoint(50, 20);
  Serial.print("Auto re-zero, midpoint: ");
  Serial.println(ACS.getMidPoint());
  iSmoothed = 0.0f;
  lastMillis = millis();   // don't count the blocked time as energy
}

/* ================= DAILY TRACKER ================= */
void handleDailyTracker(unsigned long currentMillis) {
  if (currentMillis - dailyTrackerStartMillis >= dailyTrackerInterval) {
    dailyKWh = 0.0;
    dailyCost = 0.0;
    dailyTrackerStartMillis = currentMillis;
  }
}

/* ================= BUTTON ================= */
void checkButtonCalibration() {
  if (digitalRead(CAL_BUTTON) == LOW) {
    if (!buttonHeld) {
      buttonPressTime = millis();
      buttonHeld = true;
    }
    if (millis() - buttonPressTime >= 3000) {
      startCalibration();
      buttonHeld = false;
      while (digitalRead(CAL_BUTTON) == LOW) delay(10);
    }
  } else {
    buttonHeld = false;
  }
}

/* ================= CALIBRATION ================= */
void startCalibration() {
  lcdShow("Calibrating...", "Load must be OFF");
  ACS.autoMidPoint(50, 20);
  ACS.mVNoiseLevel(50, 10);
  iSmoothed = 0.0f;
  lcdShow("Calibration Done", "");
  delay(1000);
  lastMillis = millis();
}

/* ================= EEPROM ================= */
void readEnergyDataFromEEPROM() {
  EEPROM.get(addrKWh, kWh);
  EEPROM.get(addrCost, cost);
  if (isnan(kWh) || kWh < 0) kWh = 0;
  if (isnan(cost) || cost < 0) cost = 0;
  lastSavedKWh = kWh;
}

void saveEnergyDataToEEPROM() {
  unsigned long currentMillis = millis();
  if (((kWh - lastSavedKWh) >= KWH_SAVE_THRESHOLD) ||
      ((currentMillis - lastEepromSaveMillis >= EEPROM_SAVE_INTERVAL) && (kWh > lastSavedKWh))) {
    EEPROM.put(addrKWh, kWh);
    EEPROM.put(addrCost, cost);
    EEPROM.commit();
    lastSavedKWh = kWh;
    lastEepromSaveMillis = currentMillis;
  }
}

/* ================= DISPLAY ================= */
void lcdShow(const char* line0, const char* line1) {
  char b[17];
  snprintf(b, sizeof(b), "%-16.16s", line0);
  lcd.printAt(0, 0, b);
  snprintf(b, sizeof(b), "%-16.16s", line1);
  lcd.printAt(0, 1, b);
}

void updateLCD() {
  char l0[17], l1[17];

  if (displayPage == 0) {
    // Screen 1: ALL 4 PARAMETERS (V, I, P, E)
    snprintf(l0, sizeof(l0), "V:%5.1fV I:%4.2fA", Vrms, Irms);

    char e_buf[8];
    if (kWh < 10.0) snprintf(e_buf, sizeof(e_buf), "%5.3fk", kWh);
    else            snprintf(e_buf, sizeof(e_buf), "%5.2fk", kWh);

    if (realPower < 100.0) snprintf(l1, sizeof(l1), "P:%4.1fW E:%s", realPower, e_buf);
    else                   snprintf(l1, sizeof(l1), "P:%4.0fW E:%s", realPower, e_buf);
  } else {
    // Screen 2: Cost & Connectivity Status
    snprintf(l0, sizeof(l0), "Cost:Rs.%-8.2f", cost);
    snprintf(l1, sizeof(l1), "WiFi:%s Web:%s",
             WiFi.status() == WL_CONNECTED ? "OK" : "NO",
             serverOK ? "OK" : "NO");
  }

  lcdShow(l0, l1);
}
