#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
// NimBLE replaces Bluedroid: its deinit() frees the service/characteristic objects, which
// makes it safe to tear the BLE stack down during TRACKING — worth ~25 mA, measured. The
// Bluedroid library leaks ~2-4 KB per cycle and would exhaust the heap within weeks.
#include <NimBLEDevice.h>
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
// D2 — gate of the NDS0610 P-MOSFET high-side switch on the GPS supply.
// LOW = GPS on. HIGH, or HiZ in deep sleep, = off via the 10.2 kOhm gate pull-up.
// Switch the HIGH side, never GND: the GPS UART would otherwise backfeed through the
// ESP32 GPIO and keep the module partly alive.
#define GPS_POWER_PIN 4
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
// 4000 points = 48 KB RAM, ~5.5 h of tracking at a 5 s interval. A single 45-min ride
// logged 499 points, so 1000 was only ~83 min of total tracking.
#define MAX_POINTS           4000
#define SAVE_EVERY           10
#define BATCH_SIZE           10

// --- Power optimisation (all values measured on hardware) ---
#define CPU_FREQ_MHZ         80      // ~halves CPU current; 80 is the floor with BLE active
#define LOOP_DELAY_MS        10      // yield so the idle task can halt the core (WFI).
                                     // Keep well under ~250 ms: at 9600 baud the UART ring
                                     // buffer holds ~266 ms of NMEA.
#define MOTION_POLL_MS       20      // 50 Hz = 2x the ADXL345 ODR. Polling AT the ODR can
                                     // read one sample twice and skip the next as the
                                     // clocks drift, halving sensitivity to brief peaks.
                                     // Was one full I2C transaction per loop iteration.
#define ADV_INTERVAL_MIN     0x320   // 800 x 0.625 ms = 500 ms
#define ADV_INTERVAL_MAX     0x640   // 1600 x 0.625 ms = 1000 ms. Default is ~20-40 ms,
                                     // tuned for fast discovery and costly: slowing it
                                     // measured ~6.5 mA.
#define GPS_RX_BUFFER        1024

// Periodic BLE sync window while riding. 30 s every 10 min is 5% duty, ~1.25 mA on top of
// 48 — so it keeps nearly all of the 24 mA saving while removing the single point of
// failure in the sync path. The post-ride window is only SLEEP_DELAY_MS long and Android
// background BLE scanning is unreliable (doze, app kills, scan throttling); miss it and the
// device sleeps until the next motion.
#define BLE_WINDOW_INTERVAL_MS  600000UL
#define BLE_WINDOW_DURATION_MS  30000UL

// A connected client BLOCKS deep sleep, so an app that holds the link open — or crashes
// with it open — pins the device at ~73 mA and flattens the pack in ~1.5 days. Stop
// honouring the block after this long rather than trusting the client.
#define MAX_CONN_SECONDS        600UL
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
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pBatchChar = NULL;
NimBLECharacteristic* pCountChar = NULL;
NimBLECharacteristic* pStatusChar = NULL;
NimBLECharacteristic* pStateChar = NULL;
bool deviceConnected = false;
bool accelOk = false;            // false => ADXL345 missing; motion wake unavailable
NimBLEAdvertising* pAdv = NULL;
bool bleUp = false;              // is the BLE stack currently initialised?

// Keep BLE up for the whole ride so the app can show live state. Costs the full ~24 mA
// (tracking 48 -> 73 mA), so OFF by default and treated as a user preference. Persisted in
// NVS key "live" so the app can enable it later without a firmware change.
bool liveTracking = false;
unsigned long bleWindowStart  = 0;
unsigned long nextBleWindowAt = 0;
unsigned long bleConnectMs    = 0;  // millis() when the current connection opened

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

// Factory reset: hold BOOT for RESET_HOLD_MS during normal operation.
//
// This used to run only in setup(), which made it IMPOSSIBLE to trigger: GPIO9 is the
// ESP32-C3 bootloader strap, so holding it across a reset puts the chip into ROM download
// mode and the sketch never runs at all. The only window was pressing BOOT in the few
// milliseconds after a reset but before setup() read it — not a procedure anyone can
// perform reliably.
//
// Polling in loop() avoids the strap entirely: GPIO9 is only special at reset, and reading
// it during normal operation is fine. It also works on a sealed device, where RESET may not
// be reachable.
void pollFactoryReset() {
  static unsigned long heldSince = 0;
  if (digitalRead(BOOT_BUTTON) == LOW) {
    if (heldSince == 0) {
      heldSince = millis();
      Serial.println("BOOT held — keep holding for factory reset...");
    } else if (millis() - heldSince > RESET_HOLD_MS) {
      factoryReset();
    }
  } else {
    heldSince = 0;
  }
}

// --- Deep sleep (ADXL345 interrupt) ---
void enterDeepSleep() {
  savePoints();
  gpsPower(false);   // cut the GPS rail: this is the 31 mA -> 0.29 mA win
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
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    bleConnectMs = millis();
    idleStartTime = millis();
    sendIndex = 0;
    Serial.println("BLE: client connected");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    bleConnectMs = 0;
    idleStartTime = millis();
    Serial.println("BLE: client disconnected");
    // Guarded: stopBle() tears the stack down while a client may still be attached, which
    // fires this callback mid-deinit. Touching pServer then would use freed memory.
    if (bleUp && state == IDLE) pServer->startAdvertising();
  }
};

class PairCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    if (isPaired) {
      Serial.println("BLE: already paired, ignoring");
      return;
    }
    // Read as raw bytes, NOT into an Arduino String. A String is null-terminated, so a
    // random 16-byte key containing a 0x00 byte would be truncated and rejected as "invalid
    // key length" — ~6% of keys (1 - (255/256)^16), failing intermittently and
    // inexplicably.
    NimBLEAttValue data = pChar->getValue();
    if (data.length() == 16) {
      memcpy(encKey, data.data(), 16);
      prefs.putBytes("key", encKey, 16);
      isPaired = true;
      pStatusChar->setValue("PAIRED");
      Serial.println("BLE: paired successfully");
    } else {
      Serial.println("BLE: invalid key length");
    }
  }
};

class ClearCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
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

class SeekCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
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
class BatchReadCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
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

class CountReadCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    if (!isPaired) {
      pChar->setValue("LOCKED");
      return;
    }
    char countStr[8];
    snprintf(countStr, sizeof(countStr), "%d", pointCount);
    pChar->setValue(countStr);
  }
};

// GPS supply control via the D2 P-MOSFET. Cutting the rail in deep sleep took measured
// sleep current from 31 mA to 0.29 mA — a ~107x reduction, and by far the largest single
// power win in the design. The always-on GPS was the entire sleep floor before this.
void gpsPower(bool on) {
  pinMode(GPS_POWER_PIN, OUTPUT);
  digitalWrite(GPS_POWER_PIN, on ? LOW : HIGH);
}

// GPS power is a pure function of state, driven from here so it cannot be missed on a
// transition: IDLE is the only state where we are not trying to hold a fix.
void setState(TrackerState newState) {
  state = newState;
  switch (state) {
    case IDLE:
      gpsPower(false);
      startBle();                  // back up so the app can sync after the ride
      if (pStateChar) pStateChar->setValue("IDLE");
      break;
    case ACQUIRING:
      gpsPower(true);
      if (!liveTracking) {
        stopBle();                 // recovers the controller's ~24 mA for the whole ride
        nextBleWindowAt = millis() + BLE_WINDOW_INTERVAL_MS;
      }
      if (pStateChar) pStateChar->setValue("ACQUIRING");
      break;
    case TRACKING:
      gpsPower(true);   // already on via ACQUIRING; idempotent and cheap
      if (pStateChar) pStateChar->setValue("TRACKING");
      break;
    case PAUSED:
      // Stay powered: PAUSED is a stop within a ride, and re-acquiring from cold would
      // cost 5-41 s of TTFF producing nothing.
      gpsPower(true);
      if (pStateChar) pStateChar->setValue("PAUSED");
      break;
  }
}

float getMotion() {
  // Without this guard a missing/failed ADXL345 reads as zeros, so |0 - 9.8| = 9.8 clears
  // both WAKE_THRESHOLD and RIDE_THRESHOLD — the device believes it is permanently moving,
  // never returns to IDLE, and so never brings BLE back up. It then sits awake and
  // unreachable until the battery dies. Reporting "no motion" instead keeps it idle,
  // advertising and syncable, which is the right failure mode.
  if (!accelOk) return 0.0f;
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

// Static callback instances. NimBLECharacteristic::setCallbacks() does NOT take ownership,
// so heap-allocating these would leak on every BLE rebuild once the stack is cycled during
// TRACKING. NimBLEServer::setCallbacks() DOES take ownership by default — hence the explicit
// false below, or it would call delete on a static and corrupt the heap.
static ServerCallbacks     cbServer;
static BatchReadCallbacks  cbBatch;
static CountReadCallbacks  cbCount;
static ClearCallbacks      cbClear;
static PairCallbacks       cbPair;
static SeekCallbacks       cbSeek;

// --- BLE bring-up / tear-down ---
// The BLE controller costs ~24 mA of the 73 mA tracking total — measured by subtraction
// against a build with BLE never initialised. Neither light sleep nor stopping advertising
// recovers any of it (both tested: the meter did not move), because the cost is the
// controller sitting radio-ready, not the advertising packets. Only a full teardown works.
//
// Safe to cycle only on NimBLE: Bluedroid's deinit() left services, characteristics and
// callbacks allocated, leaking ~2-4 KB per ride.
void startBle() {
  if (bleUp) return;

  const char* bleName = isPaired ? "BikeTracker" : "BikeTracker (Setup)";
  NimBLEDevice::init(bleName);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&cbServer, false);   // false: do not delete a static

  NimBLEService* pService = pServer->createService(NimBLEUUID(SERVICE_UUID));

  pBatchChar = pService->createCharacteristic(CHAR_BATCH_UUID, NIMBLE_PROPERTY::READ);
  pBatchChar->setCallbacks(&cbBatch);

  pCountChar = pService->createCharacteristic(CHAR_COUNT_UUID, NIMBLE_PROPERTY::READ);
  pCountChar->setCallbacks(&cbCount);

  NimBLECharacteristic* pClearChar = pService->createCharacteristic(CHAR_CLEAR_UUID, NIMBLE_PROPERTY::WRITE);
  pClearChar->setCallbacks(&cbClear);

  NimBLECharacteristic* pPairChar = pService->createCharacteristic(CHAR_PAIR_UUID, NIMBLE_PROPERTY::WRITE);
  pPairChar->setCallbacks(&cbPair);

  NimBLECharacteristic* pSeekChar = pService->createCharacteristic(CHAR_SEEK_UUID, NIMBLE_PROPERTY::WRITE);
  pSeekChar->setCallbacks(&cbSeek);

  pStatusChar = pService->createCharacteristic(CHAR_STATUS_UUID, NIMBLE_PROPERTY::READ);
  pStatusChar->setValue(isPaired ? "PAIRED" : "SETUP");

  pStateChar = pService->createCharacteristic(CHAR_STATE_UUID, NIMBLE_PROPERTY::READ);
  switch (state) {
    case IDLE:      pStateChar->setValue("IDLE");      break;
    case ACQUIRING: pStateChar->setValue("ACQUIRING"); break;
    case TRACKING:  pStateChar->setValue("TRACKING");  break;
    case PAUSED:    pStateChar->setValue("PAUSED");    break;
  }

  pService->start();
  pAdv = pServer->getAdvertising();
  pAdv->setMinInterval(ADV_INTERVAL_MIN);
  pAdv->setMaxInterval(ADV_INTERVAL_MAX);
  // Set the name explicitly: the app discovers BY NAME, so omitting it from the
  // advertisement would be a silent, total break.
  pAdv->setName(bleName);
  pAdv->start();

  bleUp = true;
  Serial.print("BLE up as '");
  Serial.print(bleName);
  Serial.print("' heap=");
  Serial.println(ESP.getFreeHeap());
}

void stopBle() {
  if (!bleUp) return;
  NimBLEDevice::deinit(true);   // also deletes the server, services and characteristics
  pServer = NULL;  pAdv = NULL;
  pBatchChar = pCountChar = pStatusChar = pStateChar = NULL;
  deviceConnected = false;
  bleUp = false;
  Serial.print("BLE down, heap=");
  Serial.println(ESP.getFreeHeap());
}

void setup() {
  // Before any peripheral init — changing the clock later would corrupt UART baud
  // dividers. Roughly halves CPU current. Do NOT go below 80 MHz: the BLE controller
  // needs the APB at 80 MHz.
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();

  // Power the GPS rail FIRST, before Serial/LittleFS/NVS/BLE. Acquisition takes tens of
  // seconds, so overlapping it with the rest of boot is time the ride is captured rather
  // than missed.
  gpsPower(true);

  Serial.begin(115200);
  // Wait for a USB-CDC host only on a COLD boot, and briefly. On battery there is no host,
  // so a blanket wait burns its full timeout on EVERY motion wake — delaying GPS power-on
  // for a device whose job is catching the start of a ride.
  if (wakeCause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    while (!Serial && millis() < 600) delay(10);
  }

  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  if (wakeCause == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Woke from motion interrupt");
  }

  gpsSerial.setRxBufferSize(GPS_RX_BUFFER);  // must precede begin(); headroom for delay()
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
  liveTracking = prefs.getBool("live", false);   // user preference, default off
  isPaired = prefs.getBytesLength("key") == 16;
  if (isPaired) {
    prefs.getBytes("key", encKey, 16);
    Serial.println("Device is paired.");
  } else {
    Serial.println("Device is NOT paired — waiting for setup.");
  }

  // Retry: begin() can miss on the first try straight after a hard reset while the
  // ADXL345 is still coming up — observed repeatedly on real hardware.
  for (int attempt = 1; attempt <= 3 && !accelOk; attempt++) {
    accelOk = accel.begin();
    if (!accelOk) {
      Serial.print("ADXL345 not responding (attempt ");
      Serial.print(attempt);
      Serial.println("/3)");
      delay(50);
    }
  }
  if (accelOk) {
    accel.setRange(ADXL345_RANGE_4_G);
  } else {
    // Previously `while(1)`, which hung BEFORE BLE init: no BLE, no sleep, ~30-80 mA until
    // the cell was flat, and indistinguishable from a dead device except that USB stayed
    // enumerated. Now the failure is diagnosable and the device still sleeps at the floor.
    Serial.println("No ADXL345 detected! Motion wake unavailable — timer wake only.");
  }

  startBle();
  Serial.println("Waiting for motion...");

  lastMotionTime = millis();
  idleStartTime = millis();
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  pollFactoryReset();

  unsigned long now = millis();

  // Rate-limit the accelerometer read. It was one full I2C transaction per loop iteration —
  // thousands per second against a sensor running at 25 Hz. Same test, sampled at twice the
  // sensor's own rate.
  static unsigned long lastMotionPoll = 0;
  static float motionCached = 0.0f;
  if (now - lastMotionPoll >= MOTION_POLL_MS) {
    motionCached = getMotion();
    lastMotionPoll = now;
  }
  float motion = motionCached;

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

  // Periodic BLE sync window while riding: repeated chances for the app to find the device
  // mid-ride, rather than depending solely on the post-ride window.
  if (!liveTracking && state != IDLE) {
    if (!bleUp && (long)(now - nextBleWindowAt) >= 0) {
      startBle();
      bleWindowStart = now;
      Serial.println("BLE: sync window open");
    } else if (bleUp && (now - bleWindowStart > BLE_WINDOW_DURATION_MS)) {
      // Never tear down mid-transfer: a connected client is syncing, and closing the stack
      // under it would abort the download. Defer until it disconnects.
      if (!deviceConnected) {
        stopBle();
        nextBleWindowAt = now + BLE_WINDOW_INTERVAL_MS;
        Serial.println("BLE: sync window closed");
      }
    }
  }

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
      // A connected client normally blocks sleep so a sync is never cut off. Stop honouring
      // that past MAX_CONN_SECONDS, or a client that never disconnects keeps the device
      // awake at ~73 mA until the pack is flat. Deep sleep drops the link.
      {
        bool connStale = deviceConnected && bleConnectMs &&
                         ((now - bleConnectMs) / 1000UL > MAX_CONN_SECONDS);
        if (connStale) Serial.println("BLE: connection held too long — sleeping anyway");
        if ((!deviceConnected || connStale) && (now - idleStartTime > SLEEP_DELAY_MS)) {
          enterDeepSleep();
        }
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
  
  // Yield to the FreeRTOS idle task, which halts the core until the next interrupt. The
  // loop previously spun flat out at full clock.
  delay(LOOP_DELAY_MS);
}
