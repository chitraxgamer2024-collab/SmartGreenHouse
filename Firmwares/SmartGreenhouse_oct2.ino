/*
  Smart Greenhouse for ESP32-S3-DevKitC-1 N16R8

  Libraries to install in Arduino IDE Library Manager:
    Adafruit GFX Library
    Adafruit ILI9341
    XPT2046_Touchscreen by Paul Stoffregen
    DHT sensor library by Adafruit

  Board package: esp32 by Espressif Systems
  Board: ESP32S3 Dev Module

  Wiring matches the supplied smart_greenhouse_wiring_guide.pdf.
  T_IRQ is intentionally unused. The touch controller is polled.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <DHT.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Pins ----------
constexpr uint8_t PIN_SOIL       = 1;
constexpr uint8_t PIN_DHT        = 4;
constexpr uint8_t PIN_PUMP       = 5;
constexpr uint8_t PIN_FAN_IN     = 6;
constexpr uint8_t PIN_TOUCH_CS   = 7;
constexpr uint8_t PIN_TFT_DC     = 8;
constexpr uint8_t PIN_TFT_RST    = 9;
constexpr uint8_t PIN_TFT_CS     = 10;
constexpr uint8_t PIN_SPI_MOSI   = 11;
constexpr uint8_t PIN_SPI_SCK    = 12;
constexpr uint8_t PIN_SPI_MISO   = 13;
constexpr uint8_t PIN_FAN_OUT    = 15;
constexpr uint8_t PIN_TACH_IN    = 16;
constexpr uint8_t PIN_TACH_OUT   = 17;

constexpr uint8_t DHT_TYPE = DHT22;
constexpr uint32_t FAN_PWM_HZ = 25000;
constexpr uint8_t FAN_PWM_BITS = 8;

// The NPN fan interface is electrically inverted: transistor ON pulls PWM LOW.
constexpr bool FAN_PWM_INVERTED = true;

// Change these four values after checking the touch panel's raw readings.
constexpr int TOUCH_MIN_X = 250;
constexpr int TOUCH_MAX_X = 3850;
constexpr int TOUCH_MIN_Y = 250;
constexpr int TOUCH_MAX_Y = 3850;

Adafruit_ILI9341 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
XPT2046_Touchscreen touch(PIN_TOUCH_CS);
DHT dht(PIN_DHT, DHT_TYPE);
Preferences prefs;

// ---------- Plant profiles ----------
struct PlantProfile {
  const char *name;
  float tempIdealLow;
  float tempIdealHigh;
  float tempFanStart;
  float tempDanger;
  float humidityIdealLow;
  float humidityIdealHigh;
  float humidityFanStart;
  int moistureIdealLow;
  int moistureIdealHigh;
  int waterStart;
  int waterStop;
  uint32_t pumpPulseMs;
  uint32_t soakMs;
  uint32_t minWaterIntervalMs;
  uint32_t dailyPumpLimitMs;
  uint8_t minimumVentPercent;
};

// Percentages require calibration in the Settings screen.
const PlantProfile PROFILES[] = {
  // Tulsi likes warm conditions and evenly moist, freely draining soil.
  {"Tulsi", 22.0, 30.0, 30.0, 36.0, 45.0, 70.0, 76.0,
   45, 72, 39, 55, 2200, 90000, 20UL * 60UL * 1000UL, 45000, 18},
  // Coriander is cooler and is watered sooner to keep moisture consistent.
  {"Coriander", 16.0, 24.0, 25.0, 31.0, 45.0, 70.0, 75.0,
   52, 76, 46, 62, 1800, 120000, 25UL * 60UL * 1000UL, 36000, 14}
};
constexpr uint8_t PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);

enum ScreenPage : uint8_t { PAGE_DASHBOARD, PAGE_MENU, PAGE_MANUAL, PAGE_SETTINGS };

struct SensorState {
  float temperature = NAN;
  float humidity = NAN;
  float moisture = NAN;
  int soilRaw = 0;
  bool airValid = false;
  bool soilValid = false;
};

SensorState sensors;
ScreenPage page = PAGE_DASHBOARD;
uint8_t profileIndex = 0;
bool automaticMode = true;
bool pumpOn = false;
uint8_t fanPercent = 0;
uint8_t manualFanPercent = 40;
float healthScore = 0;
String healthLabel = "Starting";
String actionLabel = "Reading sensors";

// Soil calibration defaults are deliberately conservative placeholders.
// Use Settings -> SET DRY and SET WET with the actual sensor.
int soilRawDry = 3000;
int soilRawWet = 1350;

uint32_t lastSensorRead = 0;
uint32_t lastDisplayDraw = 0;
uint32_t lastBleNotify = 0;
uint32_t pumpStartedAt = 0;
uint32_t lastWaterFinishedAt = 0;
uint32_t soakUntil = 0;
uint32_t dayWindowStartedAt = 0;
uint32_t pumpUsedThisWindowMs = 0;
uint8_t dryConfirmations = 0;
uint32_t sensorRevision = 0;
uint32_t lastWaterDecisionRevision = 0;
bool displayDirty = true;
bool bleConnected = false;

// ---------- BLE ----------
constexpr char BLE_NAME[] = "SmartGreenhouse";
constexpr char SERVICE_UUID[] = "59c90001-842e-4a8d-a735-7e05a0dc1e01";
constexpr char STATUS_UUID[]  = "59c90002-842e-4a8d-a735-7e05a0dc1e01";
constexpr char COMMAND_UUID[] = "59c90003-842e-4a8d-a735-7e05a0dc1e01";
BLECharacteristic *statusCharacteristic = nullptr;

// ---------- Colours ----------
constexpr uint16_t C_BG       = 0x0841;
constexpr uint16_t C_PANEL    = 0x10E3;
constexpr uint16_t C_PANEL_2  = 0x1944;
constexpr uint16_t C_TEXT     = 0xEF7D;
constexpr uint16_t C_MUTED    = 0x9CF3;
constexpr uint16_t C_RED      = 0xE1C7;
constexpr uint16_t C_BLUE     = 0x4D7F;
constexpr uint16_t C_YELLOW   = 0xFDC7;

// ---------- Helpers ----------
float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float rangeScore(float value, float idealLow, float idealHigh, float dangerLow, float dangerHigh) {
  if (value >= idealLow && value <= idealHigh) return 100.0f;
  if (value < idealLow) return 100.0f * (value - dangerLow) / (idealLow - dangerLow);
  return 100.0f * (dangerHigh - value) / (dangerHigh - idealHigh);
}

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t healthColour(float score) {
  score = clampf(score, 0, 100);
  // Brown -> amber -> fresh green.
  const uint8_t brown[3] = {120, 67, 35};
  const uint8_t amber[3] = {202, 151, 45};
  const uint8_t green[3] = {58, 190, 92};
  const uint8_t *a = score < 50 ? brown : amber;
  const uint8_t *b = score < 50 ? amber : green;
  float t = score < 50 ? score / 50.0f : (score - 50.0f) / 50.0f;
  return rgb565(a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t);
}

void setPump(bool on) {
  if (on == pumpOn) return;
  pumpOn = on;
  digitalWrite(PIN_PUMP, on ? HIGH : LOW); // LR7843 assumed active-high.
  if (on) {
    pumpStartedAt = millis();
  } else if (pumpStartedAt != 0) {
    uint32_t used = millis() - pumpStartedAt;
    pumpUsedThisWindowMs += used;
    lastWaterFinishedAt = millis();
    soakUntil = millis() + PROFILES[profileIndex].soakMs;
    pumpStartedAt = 0;
  }
  displayDirty = true;
}

void setFan(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  fanPercent = percent;
  uint8_t duty = map(percent, 0, 100, 0, 255);
  if (FAN_PWM_INVERTED) duty = 255 - duty;
  ledcWrite(PIN_FAN_IN, duty);
  ledcWrite(PIN_FAN_OUT, duty);
}

int moisturePercentFromRaw(int raw) {
  if (soilRawDry == soilRawWet) return 0;
  float pct = 100.0f * (raw - soilRawDry) / (float)(soilRawWet - soilRawDry);
  return constrain((int)roundf(pct), 0, 100);
}

void saveSettings() {
  prefs.begin("greenhouse", false);
  prefs.putUChar("plant", profileIndex);
  prefs.putInt("soilDry", soilRawDry);
  prefs.putInt("soilWet", soilRawWet);
  prefs.end();
}

void loadSettings() {
  prefs.begin("greenhouse", true);
  profileIndex = prefs.getUChar("plant", 0);
  if (profileIndex >= PROFILE_COUNT) profileIndex = 0;
  soilRawDry = prefs.getInt("soilDry", 3000);
  soilRawWet = prefs.getInt("soilWet", 1350);
  prefs.end();
}

// ---------- Sensing and control ----------
void readSensors() {
  int raw = analogRead(PIN_SOIL);
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  sensors.soilRaw = raw;
  sensors.soilValid = raw > 20 && raw < 4075 && abs(soilRawDry - soilRawWet) >= 200;
  if (sensors.soilValid) {
    float m = moisturePercentFromRaw(raw);
    sensors.moisture = isnan(sensors.moisture) ? m : 0.72f * sensors.moisture + 0.28f * m;
  }

  bool validAir = !isnan(t) && !isnan(h) && t > -20 && t < 80 && h >= 0 && h <= 100;
  if (validAir) {
    if (!sensors.airValid) {
      sensors.temperature = t;
      sensors.humidity = h;
    } else {
      sensors.temperature = 0.75f * sensors.temperature + 0.25f * t;
      sensors.humidity = 0.75f * sensors.humidity + 0.25f * h;
    }
  }
  sensors.airValid = validAir;
  sensorRevision++;
  displayDirty = true;
}

void updateHealth() {
  const PlantProfile &p = PROFILES[profileIndex];
  float total = 0;
  float weight = 0;

  if (sensors.soilValid) {
    total += 0.50f * clampf(rangeScore(sensors.moisture, p.moistureIdealLow,
                                       p.moistureIdealHigh, 8, 96), 0, 100);
    weight += 0.50f;
  }
  if (sensors.airValid) {
    total += 0.30f * clampf(rangeScore(sensors.temperature, p.tempIdealLow,
                                       p.tempIdealHigh, 7, 43), 0, 100);
    total += 0.20f * clampf(rangeScore(sensors.humidity, p.humidityIdealLow,
                                       p.humidityIdealHigh, 15, 96), 0, 100);
    weight += 0.50f;
  }
  healthScore = weight > 0 ? total / weight : 0;

  if (weight < 0.99f) healthLabel = "Sensor check";
  else if (healthScore >= 82) healthLabel = "Thriving";
  else if (healthScore >= 62) healthLabel = "Healthy";
  else if (healthScore >= 38) healthLabel = "Stressed";
  else healthLabel = "Critical";
}

uint8_t calculateFanDemand() {
  const PlantProfile &p = PROFILES[profileIndex];
  if (!sensors.airValid) return 35; // Fail-safe ventilation on DHT failure.

  float heatDemand = 0;
  if (sensors.temperature > p.tempFanStart) {
    heatDemand = 35.0f + 65.0f * (sensors.temperature - p.tempFanStart) /
                 max(1.0f, p.tempDanger - p.tempFanStart);
  }
  float humidityDemand = 0;
  if (sensors.humidity > p.humidityFanStart) {
    humidityDemand = 35.0f + 65.0f * (sensors.humidity - p.humidityFanStart) /
                     max(1.0f, 95.0f - p.humidityFanStart);
  }

  uint8_t demand = constrain((int)roundf(max(heatDemand, humidityDemand)), 0, 100);
  if (demand == 0) demand = p.minimumVentPercent;

  // When colder than the plant's ideal range, retain only gentle air exchange.
  if (sensors.temperature < p.tempIdealLow && demand > 10) demand = 10;
  return demand;
}

void automaticControl() {
  const PlantProfile &p = PROFILES[profileIndex];

  if (millis() - dayWindowStartedAt >= 24UL * 60UL * 60UL * 1000UL) {
    dayWindowStartedAt = millis();
    pumpUsedThisWindowMs = 0;
  }

  setFan(calculateFanDemand());

  if (pumpOn) {
    if (millis() - pumpStartedAt >= p.pumpPulseMs ||
        pumpUsedThisWindowMs + (millis() - pumpStartedAt) >= p.dailyPumpLimitMs ||
        !sensors.soilValid || sensors.moisture >= p.waterStop) {
      setPump(false);
    }
    actionLabel = "Watering pulse";
    return;
  }

  if (!sensors.soilValid) {
    dryConfirmations = 0;
    actionLabel = "Soil sensor check";
    return;
  }
  if (pumpUsedThisWindowMs >= p.dailyPumpLimitMs) {
    actionLabel = "Daily water limit";
    return;
  }
  if ((int32_t)(soakUntil - millis()) > 0) {
    actionLabel = "Soaking / rechecking";
    return;
  }
  if (lastWaterFinishedAt != 0 && millis() - lastWaterFinishedAt < p.minWaterIntervalMs) {
    actionLabel = "Water cooldown";
    return;
  }

  // Count dryness only once per new reading, never once per loop iteration.
  if (sensorRevision != lastWaterDecisionRevision) {
    lastWaterDecisionRevision = sensorRevision;
    if (sensors.moisture <= p.waterStart) dryConfirmations++;
    else if (sensors.moisture >= p.waterStop) dryConfirmations = 0;
  }

  if (dryConfirmations >= 3) {
    dryConfirmations = 0;
    setPump(true);
    actionLabel = "Watering pulse";
  } else if (fanPercent > p.minimumVentPercent) {
    actionLabel = sensors.temperature > p.tempFanStart ? "Cooling airflow" : "Reducing humidity";
  } else {
    actionLabel = "Conditions stable";
  }
}

// ---------- Display ----------
void textAt(int16_t x, int16_t y, const String &s, uint8_t size = 1, uint16_t colour = C_TEXT) {
  tft.setCursor(x, y);
  tft.setTextSize(size);
  tft.setTextColor(colour);
  tft.print(s);
}

void roundedButton(int x, int y, int w, int h, const String &label, uint16_t colour) {
  tft.fillRoundRect(x, y, w, h, 6, colour);
  tft.drawRoundRect(x, y, w, h, 6, rgb565(110, 120, 116));
  tft.setTextSize(1);
  tft.setTextColor(C_TEXT);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  tft.setCursor(x + (w - tw) / 2, y + (h - th) / 2);
  tft.print(label);
}

void drawPlantAvatar(int cx, int baseY, float health) {
  uint16_t accent = healthColour(health);
  uint16_t stem = health > 35 ? rgb565(64, 158, 79) : rgb565(126, 91, 49);
  int droop = map((int)clampf(health, 0, 100), 0, 100, 16, 0);
  int height = map((int)clampf(health, 0, 100), 0, 100, 38, 67);
  int topY = baseY - height;

  // Pot and soil.
  tft.fillRoundRect(cx - 27, baseY, 54, 13, 4, rgb565(118, 65, 42));
  tft.fillTriangle(cx - 23, baseY + 10, cx + 23, baseY + 10, cx + 16, baseY + 39,
                   rgb565(137, 74, 46));
  tft.fillRect(cx - 19, baseY + 8, 38, 5, rgb565(76, 50, 31));

  // Stem leans as health drops.
  int topX = cx + map((int)clampf(health, 0, 100), 0, 100, 10, 0);
  tft.drawLine(cx, baseY + 6, topX, topY, stem);
  tft.drawLine(cx + 1, baseY + 6, topX + 1, topY, stem);

  int leaves = health >= 80 ? 6 : health >= 55 ? 5 : health >= 30 ? 3 : 2;
  for (int i = 0; i < leaves; ++i) {
    int side = (i % 2 == 0) ? -1 : 1;
    int ly = baseY - 13 - i * 8 + droop;
    int sx = cx + side * 2;
    int ex = cx + side * (health < 40 ? 19 : 25);
    int ey = ly + (health < 55 ? 8 : -4);
    tft.drawLine(sx, ly, ex, ey, stem);
    tft.fillCircle(ex + side * 4, ey, health < 40 ? 6 : 9, accent);
  }
  tft.fillCircle(topX, topY, health > 65 ? 8 : 5, accent);

  if (!sensors.airValid || !sensors.soilValid) {
    tft.fillCircle(cx + 32, topY - 6, 8, C_YELLOW);
    textAt(cx + 29, topY - 10, "!", 1, C_BG);
  }
}

void drawSensorCard(int x, int y, int w, const char *label, const String &value, uint16_t colour) {
  tft.fillRoundRect(x, y, w, 43, 6, C_PANEL);
  textAt(x + 8, y + 7, label, 1, C_MUTED);
  textAt(x + 8, y + 21, value, 2, colour);
}

void drawHeader(const String &title) {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, 30, C_PANEL);
  textAt(10, 8, title, 2, C_TEXT);
}

void drawDashboard() {
  uint16_t accent = healthColour(healthScore);
  drawHeader(PROFILES[profileIndex].name);
  roundedButton(264, 4, 50, 22, "MENU", accent);

  drawSensorCard(8, 39, 98, "TEMPERATURE",
                 sensors.airValid ? String(sensors.temperature, 1) + " C" : "--", C_RED);
  drawSensorCard(112, 39, 98, "HUMIDITY",
                 sensors.airValid ? String(sensors.humidity, 0) + " %" : "--", C_BLUE);
  drawSensorCard(8, 88, 98, "SOIL",
                 sensors.soilValid ? String(sensors.moisture, 0) + " %" : "CAL", accent);
  drawSensorCard(112, 88, 98, "AIRFLOW", String(fanPercent) + " %", C_TEXT);

  tft.fillRoundRect(216, 39, 96, 139, 7, C_PANEL);
  drawPlantAvatar(264, 132, healthScore);
  textAt(226, 149, String((int)roundf(healthScore)) + "%", 2, accent);
  textAt(226, 168, healthLabel, 1, accent);

  tft.fillRoundRect(8, 139, 202, 39, 6, C_PANEL_2);
  textAt(16, 147, automaticMode ? "AUTO" : "MANUAL", 1, accent);
  textAt(16, 161, actionLabel, 1, C_TEXT);

  tft.fillRoundRect(8, 187, 304, 43, 6, C_PANEL);
  textAt(16, 195, pumpOn ? "PUMP ON" : "PUMP OFF", 1, pumpOn ? C_BLUE : C_MUTED);
  const PlantProfile &p = PROFILES[profileIndex];
  textAt(16, 211, "Water at " + String(p.waterStart) + "%  Stop " + String(p.waterStop) + "%", 1, C_MUTED);
}

void drawMenu() {
  drawHeader("Menu");
  roundedButton(12, 47, 142, 54, "MANUAL CONTROL", healthColour(healthScore));
  roundedButton(166, 47, 142, 54, "SETTINGS", C_PANEL_2);
  roundedButton(12, 113, 142, 54, automaticMode ? "AUTO: ON" : "AUTO: OFF", C_PANEL_2);
  roundedButton(166, 113, 142, 54, "BACK", C_PANEL_2);
  textAt(14, 190, "BLE: SmartGreenhouse", 1, bleConnected ? healthColour(90) : C_MUTED);
  textAt(14, 207, "Plant: " + String(PROFILES[profileIndex].name), 1, C_MUTED);
}

void drawManual() {
  drawHeader("Manual control");
  roundedButton(10, 45, 92, 48, pumpOn ? "PUMP OFF" : "PUMP PULSE", pumpOn ? C_RED : C_BLUE);
  roundedButton(114, 45, 92, 48, "FAN -", C_PANEL_2);
  roundedButton(218, 45, 92, 48, "FAN +", C_PANEL_2);
  textAt(20, 111, "Fan: " + String(manualFanPercent) + "%", 2, C_TEXT);
  textAt(20, 139, "Pump safety limit remains active", 1, C_MUTED);
  roundedButton(10, 181, 142, 43, automaticMode ? "AUTO MODE" : "START AUTO", healthColour(healthScore));
  roundedButton(168, 181, 142, 43, "BACK", C_PANEL_2);
}

void drawSettings() {
  drawHeader("Settings");
  textAt(10, 39, "PLANT PRESET", 1, C_MUTED);
  roundedButton(10, 54, 144, 42, profileIndex == 0 ? "TULSI *" : "TULSI", profileIndex == 0 ? healthColour(90) : C_PANEL_2);
  roundedButton(166, 54, 144, 42, profileIndex == 1 ? "CORIANDER *" : "CORIANDER", profileIndex == 1 ? healthColour(90) : C_PANEL_2);

  textAt(10, 111, "SOIL CALIBRATION  RAW " + String(sensors.soilRaw), 1, C_MUTED);
  roundedButton(10, 127, 144, 42, "SET DRY", C_PANEL_2);
  roundedButton(166, 127, 144, 42, "SET WET", C_PANEL_2);
  textAt(10, 175, "Dry " + String(soilRawDry) + "   Wet " + String(soilRawWet), 1, C_TEXT);
  roundedButton(168, 193, 142, 37, "BACK", C_PANEL_2);
}

void redraw() {
  if (page == PAGE_DASHBOARD) drawDashboard();
  else if (page == PAGE_MENU) drawMenu();
  else if (page == PAGE_MANUAL) drawManual();
  else drawSettings();
  displayDirty = false;
  lastDisplayDraw = millis();
}

bool touched(int &x, int &y) {
  if (!touch.touched()) return false;
  TS_Point pt = touch.getPoint();
  x = constrain(map(pt.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, 319), 0, 319);
  y = constrain(map(pt.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, 239), 0, 239);
  delay(160); // Debounce; control timing continues in the main loop.
  return true;
}

bool inBox(int x, int y, int bx, int by, int bw, int bh) {
  return x >= bx && x < bx + bw && y >= by && y < by + bh;
}

void chooseProfile(uint8_t index) {
  if (index >= PROFILE_COUNT) return;
  setPump(false);
  profileIndex = index;
  dryConfirmations = 0;
  soakUntil = 0;
  saveSettings();
  updateHealth();
  displayDirty = true;
}

void manualPumpPulse() {
  const PlantProfile &p = PROFILES[profileIndex];
  if (pumpOn) setPump(false);
  else if (pumpUsedThisWindowMs < p.dailyPumpLimitMs) {
    automaticMode = false;
    setPump(true);
  }
}

void handleTouch() {
  int x, y;
  if (!touched(x, y)) return;

  if (page == PAGE_DASHBOARD && inBox(x, y, 260, 0, 60, 32)) page = PAGE_MENU;
  else if (page == PAGE_MENU) {
    if (inBox(x, y, 12, 47, 142, 54)) page = PAGE_MANUAL;
    else if (inBox(x, y, 166, 47, 142, 54)) page = PAGE_SETTINGS;
    else if (inBox(x, y, 12, 113, 142, 54)) {
      automaticMode = !automaticMode;
      if (!automaticMode) { setPump(false); setFan(manualFanPercent); }
    } else if (inBox(x, y, 166, 113, 142, 54)) page = PAGE_DASHBOARD;
  } else if (page == PAGE_MANUAL) {
    if (inBox(x, y, 10, 45, 92, 48)) manualPumpPulse();
    else if (inBox(x, y, 114, 45, 92, 48)) {
      automaticMode = false;
      manualFanPercent = max(0, (int)manualFanPercent - 10);
      setFan(manualFanPercent);
    } else if (inBox(x, y, 218, 45, 92, 48)) {
      automaticMode = false;
      manualFanPercent = min(100, (int)manualFanPercent + 10);
      setFan(manualFanPercent);
    } else if (inBox(x, y, 10, 181, 142, 43)) {
      automaticMode = true;
      setPump(false);
    } else if (inBox(x, y, 168, 181, 142, 43)) page = PAGE_MENU;
  } else if (page == PAGE_SETTINGS) {
    if (inBox(x, y, 10, 54, 144, 42)) chooseProfile(0);
    else if (inBox(x, y, 166, 54, 144, 42)) chooseProfile(1);
    else if (inBox(x, y, 10, 127, 144, 42)) { soilRawDry = sensors.soilRaw; saveSettings(); }
    else if (inBox(x, y, 166, 127, 144, 42)) { soilRawWet = sensors.soilRaw; saveSettings(); }
    else if (inBox(x, y, 168, 193, 142, 37)) page = PAGE_MENU;
  }
  displayDirty = true;
}

// ---------- BLE protocol ----------
String statusJson() {
  // Compact keys keep notifications below a typical negotiated BLE MTU.
  // p=plant, t=temp, h=humidity, m=moisture, r=raw, s=health score,
  // l=health label, o=mode, f=fan, w=pump, a=action.
  String s = "{";
  s += "\"p\":\"" + String(PROFILES[profileIndex].name) + "\",";
  s += "\"t\":" + String(sensors.airValid ? sensors.temperature : -999, 1) + ",";
  s += "\"h\":" + String(sensors.airValid ? sensors.humidity : -1, 1) + ",";
  s += "\"m\":" + String(sensors.soilValid ? sensors.moisture : -1, 1) + ",";
  s += "\"r\":" + String(sensors.soilRaw) + ",";
  s += "\"s\":" + String(healthScore, 0) + ",";
  s += "\"l\":\"" + healthLabel + "\",";
  s += "\"o\":\"" + String(automaticMode ? "AUTO" : "MANUAL") + "\",";
  s += "\"f\":" + String(fanPercent) + ",";
  s += "\"w\":" + String(pumpOn ? "true" : "false") + ",";
  s += "\"a\":\"" + actionLabel + "\"}";
  return s;
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "PLANT:TULSI") chooseProfile(0);
  else if (command == "PLANT:CORIANDER") chooseProfile(1);
  else if (command == "MODE:AUTO") { automaticMode = true; setPump(false); }
  else if (command == "MODE:MANUAL") { automaticMode = false; setPump(false); setFan(manualFanPercent); }
  else if (command == "PUMP:PULSE") manualPumpPulse();
  else if (command == "PUMP:OFF") setPump(false);
  else if (command.startsWith("FAN:")) {
    automaticMode = false;
    manualFanPercent = constrain(command.substring(4).toInt(), 0, 100);
    setFan(manualFanPercent);
  } else if (command == "CAL:DRY") { soilRawDry = sensors.soilRaw; saveSettings(); }
  else if (command == "CAL:WET") { soilRawWet = sensors.soilRaw; saveSettings(); }
  displayDirty = true;
}

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue().c_str();
    processCommand(value);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; displayDirty = true; }
  void onDisconnect(BLEServer *server) override {
    bleConnected = false;
    displayDirty = true;
    server->getAdvertising()->start();
  }
};

void setupBle() {
  BLEDevice::init(BLE_NAME);
  BLEDevice::setMTU(185);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);

  statusCharacteristic = service->createCharacteristic(
      STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *command = service->createCharacteristic(
      COMMAND_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  command->setCallbacks(new CommandCallbacks());

  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
}

void notifyStatus() {
  if (!statusCharacteristic) return;
  String status = statusJson();
  statusCharacteristic->setValue(status.c_str());
  if (bleConnected) statusCharacteristic->notify();
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);
  pinMode(PIN_TACH_IN, INPUT_PULLUP);
  pinMode(PIN_TACH_OUT, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);

  loadSettings();
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TFT_CS);
  pinMode(PIN_TFT_CS, OUTPUT);
  pinMode(PIN_TOUCH_CS, OUTPUT);
  digitalWrite(PIN_TFT_CS, HIGH);
  digitalWrite(PIN_TOUCH_CS, HIGH);

  tft.begin(27000000);
  tft.setRotation(1);
  tft.setTextWrap(false);
  touch.begin();
  touch.setRotation(1);
  dht.begin();

  ledcAttach(PIN_FAN_IN, FAN_PWM_HZ, FAN_PWM_BITS);
  ledcAttach(PIN_FAN_OUT, FAN_PWM_HZ, FAN_PWM_BITS);
  setFan(PROFILES[profileIndex].minimumVentPercent);

  dayWindowStartedAt = millis();
  setupBle();
  drawHeader("Smart Greenhouse");
  textAt(66, 104, "Starting sensors...", 2, C_TEXT);
}

void loop() {
  uint32_t now = millis();

  if (now - lastSensorRead >= 2500 || lastSensorRead == 0) {
    lastSensorRead = now;
    readSensors();
    updateHealth();
  }

  if (automaticMode) automaticControl();
  else if (pumpOn && millis() - pumpStartedAt >= PROFILES[profileIndex].pumpPulseMs) setPump(false);

  handleTouch();

  if (displayDirty || now - lastDisplayDraw >= 1500) redraw();
  if (now - lastBleNotify >= 2000) {
    lastBleNotify = now;
    notifyStatus();
  }
  delay(10);
}
