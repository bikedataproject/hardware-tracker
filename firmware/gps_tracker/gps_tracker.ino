#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <mbedtls/aes.h>
#include "esp_sleep.h"

// --- Pin definitions ---
#define GPS_RX_PIN  20
#define GPS_TX_PIN  21
#define ACCEL_SDA   6
#define ACCEL_SCL   7
#define ACCEL_INT1  5
#define BOOT_BUTTON 9

// --- Settings ---
#define GPS_LOG_INTERVAL_MS  5000
#define WAKE_THRESHOLD       2.0   // for waking from sleep — needs real movement
#define RIDE_THRESHOLD       1.5   // for tracking — vibration keeps ride alive
#define GPS_SPEED_THRESHOLD  3.0   // km/h — above this means cycling
#define STOP_TIMEOUT_MS      30000
#define COOLDOWN_MS          5000
#define SLEEP_DELAY_MS       120000
#define ACCEL_ACT_THRESHOLD  20    // 20 * 62.5mg = 1.25g for ADXL345 activity interrupt
#define RESET_HOLD_MS        5000
#define MAX_POINTS           1000
#define SAVE_EVERY           10
#define BATCH_SIZE           10
#define POINTS_FILE          "/points.bin"

// --- BLE UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_BATCH_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_COUNT_UUID     "8ca20d91-5a20-4d5b-a7f1-7c4e3a1c2b3d"
#define CHAR_CLEAR_UUID     "9da30e02-6b31-4e6c-b802-8d5f4b2d3c4e"
#define CHAR_PAIR_UUID      "a1b2c3d4-1234-5678-9abc-def012345678"
#define CHAR_STATUS_UUID    "b2c3d4e5-2345-6789-abcd-ef0123456789"
#define CHAR_STATE_UUID     "c3d4e5f6-3456-789a-bcde-f01234567890"
#define CHAR_SEEK_UUID      "d4e5f607-4567-89ab-cdef-012345678901"

// --- GPS point storage ---
struct GpsPoint {
  float lat;
  float lng;
  uint32_t epoch;  // seconds since 2000-01-01
};

GpsPoint points[MAX_POINTS];
int pointCount = 0;
int sendIndex = 0;

// --- State machine ---
enum TrackerState { IDLE, ACQUIRING, TRACKING, PAUSED };
TrackerState state = IDLE;

// --- Encryption ---
Preferences prefs;
uint8_t encKey[16];
bool isPaired = false;

// --- Objects ---
HardwareSerial gpsSerial(0);
TinyGPSPlus gps;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// --- BLE ---
BLEServer* pServer = NULL;
BLECharacteristic* pBatchChar = NULL;
BLECharacteristic* pCountChar = NULL;
BLECharacteristic* pStatusChar = NULL;
BLECharacteristic* pStateChar = NULL;
bool deviceConnected = false;

// --- Timers ---
unsigned long lastLogTime = 0;
unsigned long lastMotionTime = 0;
unsigned long stopTime = 0;
unsigned long idleStartTime = 0;

// --- GPS epoch helper ---
// Returns seconds since 2000-01-01 00:00:00 UTC
uint32_t gpsToEpoch() {
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;

  int y = gps.date.year();
  int m = gps.date.month();
  int d = gps.date.day();
  int hh = gps.time.hour();
  int mm = gps.time.minute();
  int ss = gps.time.second();

  // Days from 2000-01-01 to given date
  uint32_t days = 0;
  for (int i = 2000; i < y; i++) {
    days += (i % 4 == 0 && (i % 100 != 0 || i % 400 == 0)) ? 366 : 365;
  }
  static const int mdays[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
  for (int i = 1; i < m; i++) {
    days += mdays[i];
    if (i == 2 && (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))) days++;
  }
  days += d - 1;

  return days * 86400UL + hh * 3600UL + mm * 60UL + ss;
}

// --- Encryption helpers ---
void encryptBlock(const uint8_t* input, uint8_t* output, int len) {
  int padLen = ((len / 16) + 1) * 16;
  uint8_t padded[256] = {0};
  memcpy(padded, input, len);
  uint8_t padVal = padLen - len;
  for (int i = len; i < padLen; i++) padded[i] = padVal;

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, encKey, 128);
  for (int i = 0; i < padLen; i += 16) {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, padded + i, output + i);
  }
  mbedtls_aes_free(&aes);
}

// --- Flash storage ---
void savePoints() {
  File f = LittleFS.open(POINTS_FILE, "w");
  if (!f) return;
  f.write((uint8_t*)&pointCount, sizeof(pointCount));
  f.write((uint8_t*)points, sizeof(GpsPoint) * pointCount);
  f.close();
}

void loadPoints() {
  File f = LittleFS.open(POINTS_FILE, "r");
  if (!f) { pointCount = 0; return; }
  f.read((uint8_t*)&pointCount, sizeof(pointCount));
  if (pointCount > MAX_POINTS) pointCount = 0;
  f.read((uint8_t*)points, sizeof(GpsPoint) * pointCount);
  f.close();
}

void clearPoints() {
  pointCount = 0;
  sendIndex = 0;
  LittleFS.remove(POINTS_FILE);
}

void factoryReset() {
  Serial.println(">>> FACTORY RESET");
  prefs.clear();
  clearPoints();
  isPaired = false;
  Serial.println("Key and data cleared. Restarting...");
  Serial.flush();
  ESP.restart();
}

void checkFactoryReset() {
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  if (digitalRead(BOOT_BUTTON) == LOW) {
    Serial.println("BOOT button held — hold 5 seconds for factory reset");
    unsigned long start = millis();
    while (digitalRead(BOOT_BUTTON) == LOW) {
      if (millis() - start > RESET_HOLD_MS) {
        factoryReset();
      }
    }
    Serial.println("Released too early — no reset");
  }
}

// --- Deep sleep (ADXL345 interrupt) ---
void enterDeepSleep() {
  savePoints();
  Serial.println(">>> Entering deep sleep. Waiting for motion interrupt.");
  Serial.flush();

  // Configure ADXL345 activity interrupt on INT1
  Wire.beginTransmission(0x53);
  Wire.write(0x24);  // THRESH_ACT
  Wire.write(ACCEL_ACT_THRESHOLD);
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x27);  // ACT_INACT_CTL
  Wire.write(0x70);  // activity on X, Y, Z (AC-coupled)
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x2E);  // INT_ENABLE
  Wire.write(0x10);  // enable activity interrupt
  Wire.endTransmission();

  Wire.beginTransmission(0x53);
  Wire.write(0x2F);  // INT_MAP
  Wire.write(0x00);  // map to INT1 pin
  Wire.endTransmission();

  // Clear any pending interrupt
  Wire.beginTransmission(0x53);
  Wire.write(0x30);  // INT_SOURCE
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  if (Wire.available()) Wire.read();

  // Small delay to let interrupt line settle
  delay(50);

  // Pull-down on INT1 to prevent floating
  gpio_pulldown_en((gpio_num_t)ACCEL_INT1);
  gpio_pullup_dis((gpio_num_t)ACCEL_INT1);

  esp_deep_sleep_enable_gpio_wakeup(1 << ACCEL_INT1, ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

// --- BLE callbacks ---
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    idleStartTime = millis();
    sendIndex = 0;
    Serial.println("BLE: client connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    idleStartTime = millis();
    Serial.println("BLE: client disconnected");
    pServer->startAdvertising();
  }
};

class PairCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    if (isPaired) {
      Serial.println("BLE: already paired, ignoring");
      return;
    }
    String data = pChar->getValue();
    if (data.length() == 16) {
      memcpy(encKey, (const uint8_t*)data.c_str(), 16);
      prefs.putBytes("key", encKey, 16);
      isPaired = true;
      pStatusChar->setValue("PAIRED");
      Serial.println("BLE: paired successfully");
    } else {
      Serial.println("BLE: invalid key length");
    }
  }
};

class ClearCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    if (!isPaired) return;
    // Only allow clear when not tracking
    if (state == TRACKING) {
      Serial.println("BLE: clear rejected — still tracking");
      return;
    }
    clearPoints();
    Serial.println("BLE: points cleared");
  }
};

class SeekCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    if (!isPaired) return;
    String data = pChar->getValue();
    int idx = atoi(data.c_str());
    if (idx >= 0 && idx <= pointCount) {
      sendIndex = idx;
      Serial.print("BLE: seek to ");
      Serial.println(sendIndex);
    }
  }
};

// Batch read: sends up to BATCH_SIZE points as binary, encrypted
// Format: [2B startIndex] [2B count] then for each point:
//   Reference (first in batch): [4B lat] [4B lng] [4B epoch] = 12 bytes
//   Delta (rest):               [2B dlat] [2B dlng] [2B dt]  = 6 bytes
// Batch of 10: 4 + 12 + 9*6 = 70 bytes plaintext → 80 bytes encrypted
class BatchReadCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* pChar) {
    if (!isPaired) {
      uint8_t locked[] = {'L','O','C','K','E','D'};
      pChar->setValue(locked, 6);
      return;
    }

    if (sendIndex >= pointCount) {
      uint8_t done[] = {'D','O','N','E'};
      pChar->setValue(done, 4);
      sendIndex = 0;
      return;
    }

    // Build batch
    int batchCount = min(BATCH_SIZE, pointCount - sendIndex);
    uint8_t buf[256];
    int pos = 0;

    // Header: start index (2B) + count (2B)
    buf[pos++] = (sendIndex >> 8) & 0xFF;
    buf[pos++] = sendIndex & 0xFF;
    buf[pos++] = (batchCount >> 8) & 0xFF;
    buf[pos++] = batchCount & 0xFF;

    // Reference point (full)
    GpsPoint* ref = &points[sendIndex];
    memcpy(buf + pos, &ref->lat, 4); pos += 4;
    memcpy(buf + pos, &ref->lng, 4); pos += 4;
    memcpy(buf + pos, &ref->epoch, 4); pos += 4;

    // Delta points
    for (int i = 1; i < batchCount; i++) {
      GpsPoint* prev = &points[sendIndex + i - 1];
      GpsPoint* cur = &points[sendIndex + i];

      // Delta in 0.000001 degree units (microdegrees)
      int16_t dlat = (int16_t)((cur->lat - prev->lat) * 1000000.0f);
      int16_t dlng = (int16_t)((cur->lng - prev->lng) * 1000000.0f);
      uint16_t dt = (uint16_t)(cur->epoch - prev->epoch);

      memcpy(buf + pos, &dlat, 2); pos += 2;
      memcpy(buf + pos, &dlng, 2); pos += 2;
      memcpy(buf + pos, &dt, 2); pos += 2;
    }

    // Encrypt
    int encLen = ((pos / 16) + 1) * 16;
    uint8_t encrypted[256];
    encryptBlock(buf, encrypted, pos);

    pChar->setValue(encrypted, encLen);
    sendIndex += batchCount;

    Serial.print("BLE: sent batch ");
    Serial.print(sendIndex - batchCount);
    Serial.print("-");
    Serial.println(sendIndex - 1);
  }
};

class CountReadCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* pChar) {
    if (!isPaired) {
      pChar->setValue("LOCKED");
      return;
    }
    char countStr[8];
    snprintf(countStr, sizeof(countStr), "%d", pointCount);
    pChar->setValue(countStr);
  }
};

void setState(TrackerState newState) {
  state = newState;
  switch (state) {
    case IDLE:      pStateChar->setValue("IDLE"); break;
    case ACQUIRING: pStateChar->setValue("ACQUIRING"); break;
    case TRACKING:  pStateChar->setValue("TRACKING"); break;
    case PAUSED:    pStateChar->setValue("PAUSED"); break;
  }
}

float getMotion() {
  sensors_event_t event;
  accel.getEvent(&event);
  float magnitude = sqrt(event.acceleration.x * event.acceleration.x +
                         event.acceleration.y * event.acceleration.y +
                         event.acceleration.z * event.acceleration.z);
  return abs(magnitude - 9.8);
}

void logPoint() {
  if (gps.satellites.value() == 0 || !gps.location.isValid() || pointCount >= MAX_POINTS) return;

  uint32_t epoch = gpsToEpoch();
  if (epoch == 0) return;  // no valid time yet

  points[pointCount].lat = gps.location.lat();
  points[pointCount].lng = gps.location.lng();
  points[pointCount].epoch = epoch;
  pointCount++;

  Serial.print("[");
  Serial.print(pointCount);
  Serial.print("] Lat: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" Lng: ");
  Serial.print(gps.location.lng(), 6);
  Serial.print(" Epoch: ");
  Serial.print(epoch);
  Serial.print(" Sats: ");
  Serial.println(gps.satellites.value());

  if (pointCount % SAVE_EVERY == 0) {
    savePoints();
    Serial.println("(saved to flash)");
  }
}

void setup() {
  Serial.begin(115200);
  checkFactoryReset();

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Woke from motion interrupt");
  }

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(ACCEL_SDA, ACCEL_SCL);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS failed!");
  }
  loadPoints();
  Serial.print("Loaded ");
  Serial.print(pointCount);
  Serial.println(" points from flash.");

  prefs.begin("tracker", false);
  isPaired = prefs.getBytesLength("key") == 16;
  if (isPaired) {
    prefs.getBytes("key", encKey, 16);
    Serial.println("Device is paired.");
  } else {
    Serial.println("Device is NOT paired — waiting for setup.");
  }

  if (!accel.begin()) {
    Serial.println("No ADXL345 detected!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_4_G);

  const char* bleName = isPaired ? "BikeTracker" : "BikeTracker (Setup)";
  BLEDevice::init(bleName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID), 30);

  pBatchChar = pService->createCharacteristic(CHAR_BATCH_UUID, BLECharacteristic::PROPERTY_READ);
  pBatchChar->setCallbacks(new BatchReadCallbacks());

  pCountChar = pService->createCharacteristic(CHAR_COUNT_UUID, BLECharacteristic::PROPERTY_READ);
  pCountChar->setCallbacks(new CountReadCallbacks());

  BLECharacteristic* pClearChar = pService->createCharacteristic(CHAR_CLEAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pClearChar->setCallbacks(new ClearCallbacks());

  BLECharacteristic* pPairChar = pService->createCharacteristic(CHAR_PAIR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pPairChar->setCallbacks(new PairCallbacks());

  BLECharacteristic* pSeekChar = pService->createCharacteristic(CHAR_SEEK_UUID, BLECharacteristic::PROPERTY_WRITE);
  pSeekChar->setCallbacks(new SeekCallbacks());

  pStatusChar = pService->createCharacteristic(CHAR_STATUS_UUID, BLECharacteristic::PROPERTY_READ);
  pStatusChar->setValue(isPaired ? "PAIRED" : "SETUP");

  pStateChar = pService->createCharacteristic(CHAR_STATE_UUID, BLECharacteristic::PROPERTY_READ);
  pStateChar->setValue("IDLE");

  pService->start();
  pServer->getAdvertising()->start();

  Serial.print("BLE advertising as '");
  Serial.print(bleName);
  Serial.println("'");
  Serial.println("Waiting for motion...");

  lastMotionTime = millis();
  idleStartTime = millis();
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  unsigned long now = millis();
  float motion = getMotion();

  bool wakeMoving = motion > WAKE_THRESHOLD;
  bool accelMoving = motion > RIDE_THRESHOLD;

  // GPS speed-based movement detection
  bool gpsMoving = false;
  if (gps.speed.isValid() && gps.speed.kmph() > GPS_SPEED_THRESHOLD) {
    gpsMoving = true;
  }

  bool rideMoving = accelMoving || gpsMoving;

  if (rideMoving) {
    lastMotionTime = now;
    idleStartTime = now;
  }

  bool hasFix = gps.location.isValid() && gps.satellites.value() > 0;

  switch (state) {
    case IDLE:
      if (wakeMoving && (now - stopTime > COOLDOWN_MS)) {
        if (hasFix) {
          setState(TRACKING);
          Serial.println(">>> Motion + GPS fix — tracking");
        } else {
          setState(ACQUIRING);
          Serial.println(">>> Motion detected — acquiring GPS...");
        }
        idleStartTime = now;
      }
      if (!deviceConnected && (now - idleStartTime > SLEEP_DELAY_MS)) {
        enterDeepSleep();
      }
      break;

    case ACQUIRING:
      if (hasFix) {
        setState(TRACKING);
        Serial.println(">>> GPS fix acquired — tracking");
      }
      if (!rideMoving && (now - lastMotionTime > STOP_TIMEOUT_MS)) {
        setState(IDLE);
        stopTime = now;
        idleStartTime = now;
        Serial.println(">>> No fix, stopped moving — idle");
      }
      break;

    case TRACKING:
      if (now - lastLogTime >= GPS_LOG_INTERVAL_MS) {
        logPoint();
        lastLogTime = now;
      }
      if (!rideMoving) {
        setState(PAUSED);
        Serial.println(">>> No motion — paused");
      }
      break;

    case PAUSED:
      if (rideMoving) {
        if (hasFix) {
          setState(TRACKING);
        } else {
          setState(ACQUIRING);
        }
        Serial.println(">>> Motion resumed");
      }
      if (now - lastMotionTime > STOP_TIMEOUT_MS) {
        setState(IDLE);
        stopTime = now;
        idleStartTime = now;
        savePoints();
        Serial.print(">>> Ride ended — ");
        Serial.print(pointCount);
        Serial.println(" points saved.");
      }
      break;
  }
}
