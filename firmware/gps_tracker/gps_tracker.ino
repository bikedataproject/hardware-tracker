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
#define BOOT_BUTTON 9

// --- Settings ---
#define GPS_LOG_INTERVAL_MS  5000
#define MOTION_THRESHOLD     3.0
#define STOP_TIMEOUT_MS      30000
#define COOLDOWN_MS          5000
#define SLEEP_DELAY_MS       120000
#define SLEEP_CHECK_SEC      30      // wake every 30s to check motion
#define RESET_HOLD_MS        5000
#define MAX_POINTS           1000
#define SAVE_EVERY           10
#define POINTS_FILE          "/points.bin"

// --- BLE UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_POINT_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_COUNT_UUID     "8ca20d91-5a20-4d5b-a7f1-7c4e3a1c2b3d"
#define CHAR_CLEAR_UUID     "9da30e02-6b31-4e6c-b802-8d5f4b2d3c4e"
#define CHAR_PAIR_UUID      "a1b2c3d4-1234-5678-9abc-def012345678"
#define CHAR_STATUS_UUID    "b2c3d4e5-2345-6789-abcd-ef0123456789"
#define CHAR_STATE_UUID     "c3d4e5f6-3456-789a-bcde-f01234567890"

// --- GPS point storage ---
struct GpsPoint {
  float lat;
  float lng;
  uint32_t timestamp;
};

GpsPoint points[MAX_POINTS];
int pointCount = 0;
int sendIndex = 0;

// --- State machine ---
enum TrackerState { IDLE, TRACKING };
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
BLECharacteristic* pPointChar = NULL;
BLECharacteristic* pCountChar = NULL;
BLECharacteristic* pStatusChar = NULL;
BLECharacteristic* pStateChar = NULL;
bool deviceConnected = false;

// --- Timers ---
unsigned long lastLogTime = 0;
unsigned long lastMotionTime = 0;
unsigned long stopTime = 0;
unsigned long idleStartTime = 0;

// --- Encryption helpers ---
void toHex(const uint8_t* data, int len, char* out) {
  for (int i = 0; i < len; i++) {
    sprintf(out + i * 2, "%02x", data[i]);
  }
  out[len * 2] = '\0';
}

String encryptString(const char* plaintext) {
  int len = strlen(plaintext);
  int padLen = ((len / 16) + 1) * 16;
  uint8_t input[80] = {0};
  uint8_t output[80] = {0};
  memcpy(input, plaintext, len);

  uint8_t padVal = padLen - len;
  for (int i = len; i < padLen; i++) input[i] = padVal;

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, encKey, 128);

  for (int i = 0; i < padLen; i += 16) {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input + i, output + i);
  }
  mbedtls_aes_free(&aes);

  char hex[161];
  toHex(output, padLen, hex);
  return String(hex);
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

// --- Deep sleep (timer-based) ---
void enterDeepSleep() {
  savePoints();
  Serial.println(">>> Entering deep sleep. Wakes every 30s to check motion.");
  Serial.flush();

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_CHECK_SEC * 1000000ULL);
  esp_deep_sleep_start();
}

bool quickMotionCheck() {
  Wire.begin(ACCEL_SDA, ACCEL_SCL);
  if (!accel.begin()) return false;
  accel.setRange(ADXL345_RANGE_4_G);
  delay(50);

  for (int i = 0; i < 20; i++) {
    sensors_event_t ev;
    accel.getEvent(&ev);
    float mag = sqrt(ev.acceleration.x * ev.acceleration.x +
                     ev.acceleration.y * ev.acceleration.y +
                     ev.acceleration.z * ev.acceleration.z);
    if (abs(mag - 9.8) > MOTION_THRESHOLD) return true;
    delay(100);
  }
  return false;
}

// --- BLE callbacks ---
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    idleStartTime = millis();
    Serial.println("BLE: client connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    idleStartTime = millis();
    sendIndex = 0;
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
      Serial.println("BLE: paired successfully. Restart to update name.");
    } else {
      Serial.println("BLE: invalid key length");
    }
  }
};

class ClearCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    if (!isPaired) return;
    clearPoints();
    pCountChar->setValue("0");
    Serial.println("BLE: points cleared");
  }
};

class PointReadCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* pChar) {
    if (!isPaired) {
      pChar->setValue("LOCKED");
      return;
    }

    if (sendIndex < pointCount) {
      char buf[64];
      snprintf(buf, sizeof(buf), "%d,%.6f,%.6f,%lu",
               sendIndex, points[sendIndex].lat, points[sendIndex].lng, points[sendIndex].timestamp);

      String encrypted = encryptString(buf);
      pChar->setValue(encrypted.c_str());
      sendIndex++;
    } else {
      pChar->setValue("DONE");
      sendIndex = 0;
    }
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

bool isMoving() {
  sensors_event_t event;
  accel.getEvent(&event);
  float magnitude = sqrt(event.acceleration.x * event.acceleration.x +
                         event.acceleration.y * event.acceleration.y +
                         event.acceleration.z * event.acceleration.z);
  return abs(magnitude - 9.8) > MOTION_THRESHOLD;
}

void logPoint() {
  if (gps.satellites.value() == 0 || !gps.location.isValid() || pointCount >= MAX_POINTS) return;

  points[pointCount].lat = gps.location.lat();
  points[pointCount].lng = gps.location.lng();
  points[pointCount].timestamp = millis();
  pointCount++;

  Serial.print("[");
  Serial.print(pointCount);
  Serial.print("] Lat: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" Lng: ");
  Serial.print(gps.location.lng(), 6);
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

  // If woken by timer, quickly check for motion before full boot
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    if (!quickMotionCheck()) {
      // No motion — go straight back to sleep
      esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_CHECK_SEC * 1000000ULL);
      esp_deep_sleep_start();
    }
    Serial.println("Timer wake — motion detected, full boot");
  }

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(ACCEL_SDA, ACCEL_SCL);

  // Init filesystem
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS failed!");
  }
  loadPoints();
  Serial.print("Loaded ");
  Serial.print(pointCount);
  Serial.println(" points from flash.");

  // Load encryption key
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

  // BLE setup
  const char* bleName = isPaired ? "BikeTracker" : "BikeTracker (Setup)";
  BLEDevice::init(bleName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID), 20);

  pPointChar = pService->createCharacteristic(CHAR_POINT_UUID, BLECharacteristic::PROPERTY_READ);
  pPointChar->setCallbacks(new PointReadCallbacks());

  pCountChar = pService->createCharacteristic(CHAR_COUNT_UUID, BLECharacteristic::PROPERTY_READ);
  pCountChar->setCallbacks(new CountReadCallbacks());

  BLECharacteristic* pClearChar = pService->createCharacteristic(CHAR_CLEAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pClearChar->setCallbacks(new ClearCallbacks());

  BLECharacteristic* pPairChar = pService->createCharacteristic(CHAR_PAIR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pPairChar->setCallbacks(new PairCallbacks());

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
  bool moving = isMoving();

  if (moving) {
    lastMotionTime = now;
    idleStartTime = now;
  }

  switch (state) {
    case IDLE:
      if (moving && (now - stopTime > COOLDOWN_MS)) {
        state = TRACKING;
        idleStartTime = now;
        pStateChar->setValue("TRACKING");
        Serial.println(">>> Motion detected — tracking started");
      }
      if (!deviceConnected && (now - idleStartTime > SLEEP_DELAY_MS)) {
        enterDeepSleep();
      }
      break;

    case TRACKING:
      if (now - lastLogTime >= GPS_LOG_INTERVAL_MS) {
        logPoint();
        lastLogTime = now;
      }
      if (now - lastMotionTime > STOP_TIMEOUT_MS) {
        state = IDLE;
        stopTime = now;
        idleStartTime = now;
        pStateChar->setValue("IDLE");
        savePoints();
        Serial.print(">>> Stopped — ");
        Serial.print(pointCount);
        Serial.println(" points recorded and saved.");
      }
      break;
  }
}
