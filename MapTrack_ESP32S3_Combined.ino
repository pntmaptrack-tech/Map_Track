/*
  ══════════════════════════════════════════════════════════════════
  MapTrack — Combined GPS Tracker + Camera Unit  (v2.2, cellular-only)
  Waveshare ESP32-S3-LCD-2 (OV5640 camera) + SIM7600G-H
  ══════════════════════════════════════════════════════════════════

  This replaces BOTH GSM_to_Dashboard.ino (LOLIN32/WiFi) and
  maptrack_esp32s3_camera_1.ino (ESP32-S3/WiFi) with a single sketch
  that runs entirely over the SIM7600's cellular data — no WiFi
  anywhere in this file.

  One physical device now does two jobs:
    1. GPS TRAIL   — every GPS_CYCLE_MS, acquire a GNSS fix from the
                     SIM7600 and POST it to your Apps Script /exec
                     URL (action:"append"), same as before. Builds
                     the position trail your dashboard already reads.
    2. PHOTO GALLERY — capture button behaves exactly like before
                     (1st press = freeze frame, 2nd press = send).
                     On send, the JPEG is POSTed to gallery_server.py
                     tagged with this device's name AND the most
                     recent GPS fix, so every photo is geotagged and
                     attributable to a specific unit.

  Both the GPS name and the image "device" tag use the same
  `deviceName` constant below, so the map (grouped by device) and
  the gallery (device badge per photo) stay consistent.

  ── Why cellular changes things ──
  The SIM7600 reaches the public internet through your carrier, NOT
  your home/college WiFi LAN. That means:
    - Apps Script (script.google.com) — reachable fine, it's public.
    - gallery_server.py — is NOT reachable at a private 192.168.x.x /
      10.x.x.x address anymore, because carrier data connections sit
      behind CGNAT (no public IP handed to your device). You must
      run gallery_server.py somewhere with a real public
      address — a small cloud VM, a PaaS deployment (Render,
      Railway, PythonAnywhere, etc.), or any other host reachable
      from the open internet. Fill that address into GALLERY_HOST
      below. Until it's genuinely public, image uploads will fail
      (GPS trail uploads to Apps Script are unaffected either way).

  ── Libraries required (Arduino Library Manager) ──
    - TinyGSM          by vshymanskyy   (GPS + cellular data + HTTPS client)
    - ArduinoHttpClient by Arduino      (used for the image upload POST)
    - ArduinoJson       by Bblanchon
    - GFX Library for Arduino (moononournation)
    ESP32 board package >= 2.0.14. Tools > PSRAM: OPI PSRAM ENABLED.

  ── Before uploading ──
    1. Wiring: SIM7600 TXD -> GPIO44 (MODEM_RX), SIM7600 RXD -> GPIO43
       (MODEM_TX), SIM7600 PWRKEY -> GPIO18 (MODEM_PWRKEY). Shared GND
       between ESP32 and modem is required.
    2. Set `apn` to your SIM's APN.
    3. Set `scriptUrl` to your DEPLOYED Apps Script /exec URL.
    4. Set `GALLERY_HOST` / `GALLERY_PORT` / `GALLERY_PATH` to your
       publicly-reachable gallery_server.py (see note above).
    5. Give this physical unit a unique `deviceName` if you run more
       than one.
    6. GNSS needs open sky view; first fix can take 30-90s.
  ══════════════════════════════════════════════════════════════════
*/

#define TINY_GSM_MODEM_SIM7600
// Uncomment for verbose modem AT traffic on Serial while debugging:
// #define TINY_GSM_DEBUG Serial

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include "img_converters.h"
#include "esp_camera.h"
#include <Arduino_GFX_Library.h>

// ══════════════════════════════════════════
//  CAMERA PIN MAP (Waveshare ESP32-S3-LCD-2 official — unchanged)
// ══════════════════════════════════════════
#define CAM_PIN_PWDN    17
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    8
#define CAM_PIN_SIOD    21
#define CAM_PIN_SIOC    16

#define CAM_PIN_D7      2
#define CAM_PIN_D6      7
#define CAM_PIN_D5      10
#define CAM_PIN_D4      14
#define CAM_PIN_D3      11
#define CAM_PIN_D2      15
#define CAM_PIN_D1      13
#define CAM_PIN_D0      12
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    4
#define CAM_PIN_PCLK    9

#define LCD_SCLK 39
#define LCD_MOSI 38
#define LCD_MISO 40
#define LCD_DC   42
#define LCD_CS   45
#define LCD_RST  -1
#define LCD_BL   1

#define BTN_CAPTURE 47
#define BTN_DELETE 48

// ══════════════════════════════════════════
//  SIM7600 UART — free GPIOs, don't clash with camera/LCD/buttons above
// ══════════════════════════════════════════
#define MODEM_RX      44
#define MODEM_TX      43
#define MODEM_PWRKEY  18

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClientSecure gpsClient(modem, 0);   // HTTPS to Apps Script
TinyGsmClientSecure imgClient(modem, 1);   // HTTPS to gallery server (separate mux channel)

// ══════════════════════════════════════════
//  CELLULAR — SIM APN
// ══════════════════════════════════════════
const char* apn       = "airtelgprs.com";
const char* apnUser   = "";
const char* apnPass   = "";

// ══════════════════════════════════════════
//  MAPTRACK APPS SCRIPT — deployed /exec URL
// ══════════════════════════════════════════
const char* scriptHost = "script.google.com";
const char* scriptPath = "/macros/s/AKfycbxic2631YfHzzQUv7s7dYI11R1oJFCG9ZUDdkugzzyRJ68dQ2B7gepZGFpWYLMeIltuig/exec";

// ══════════════════════════════════════════
//  GALLERY SERVER
// ══════════════════════════════════════════
const char* GALLERY_HOST = "ets-full.onrender.com";
const int   GALLERY_PORT = 443;
const char* GALLERY_PATH = "/upload";

// ══════════════════════════════════════════
//  DEVICE IDENTITY
// ══════════════════════════════════════════
const char* deviceName     = "Field Unit 1";
const char* deviceIdPrefix = "esp32-gps-01";
const char* deviceCategory = "Live GPS";

// ══════════════════════════════════════════
//  TIMING
// ══════════════════════════════════════════
const unsigned long GPS_TIMEOUT_MS = 60000;
const unsigned long GPS_CYCLE_MS   = 120000;
const unsigned long GPS_FIX_MAX_AGE_MS = 10UL * 60UL * 1000UL;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, LCD_MISO);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, 240, 320);

uint8_t *capturedBuffer = NULL;
size_t capturedLength = 0;
int capturedWidth = 0, capturedHeight = 0;

bool imageStored = false;
bool previousCaptureState = HIGH;
bool previousDeleteState  = HIGH;
bool showingCaptured = false;
unsigned long captureTime = 0;

float lastLat = 0, lastLng = 0;
bool  hasFix = false;
unsigned long lastFixMillis = 0;
unsigned long lastGpsCycle = 0;

// ══════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MapTrack Combined (cellular-only) booting ===");

  // ---- Hardware Modem Power-On Sequence ----
  // SIM7600 PWRKEY is ACTIVE-LOW: idle HIGH, pull LOW for ~1-2s to
  // trigger power-on, then release back to HIGH.
  Serial.println("Pulsing PWRKEY to power on SIM7600...");
  pinMode(MODEM_PWRKEY, OUTPUT);

  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1200);
  digitalWrite(MODEM_PWRKEY, HIGH);

  Serial.println("Waiting for modem to boot internal firmware...");
  delay(4000);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);

  Serial.println("Initializing SIM7600 connection...");
  if (!modem.restart()) {
    Serial.println("Modem restart failed — check wiring/power, will retry in loop.");
  }
  Serial.print("Modem info: ");
  Serial.println(modem.getModemInfo());

  ensureCellularConnected();

  // ---- Display ----
  gfx->begin(40000000);
  gfx->fillScreen(0x0000);
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  Serial.println("LCD OK");

  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Total heap: %d, Free heap: %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d, Free PSRAM: %d\n", ESP.getPsramSize(), ESP.getFreePsram());

  // ---- Camera ----
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = 16000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  Serial.println("Initializing camera...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED, esp_err = 0x%x (%d)\n", err, err);
  } else {
    Serial.println("Camera init SUCCESS");
    sensor_t *s = esp_camera_sensor_get();
    if (s) { s->set_vflip(s, 1); Serial.printf("Sensor PID: 0x%x\n", s->id.PID); }
  }

  pinMode(BTN_CAPTURE, INPUT_PULLUP);
  pinMode(BTN_DELETE, INPUT_PULLUP);

  lastGpsCycle = millis() - GPS_CYCLE_MS + 5000;
}

// ══════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════
void loop() {
  bool captureState = digitalRead(BTN_CAPTURE);
  bool deleteState  = digitalRead(BTN_DELETE);

  if (!showingCaptured) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      gfx->draw16bitBeRGBBitmap(0, 40, (uint16_t *)fb->buf, fb->width, fb->height);
      esp_camera_fb_return(fb);
    }
  }

  if (previousCaptureState == HIGH && captureState == LOW) {
    if (!imageStored) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Capture FAILED");
      } else {
        capturedLength = fb->len;
        capturedWidth  = fb->width;
        capturedHeight = fb->height;

        if (capturedBuffer != NULL) { free(capturedBuffer); capturedBuffer = NULL; }
        capturedBuffer = (uint8_t *)malloc(capturedLength);

        if (capturedBuffer == NULL) {
          Serial.println("Memory Allocation Failed");
        } else {
          memcpy(capturedBuffer, fb->buf, capturedLength);
          gfx->draw16bitBeRGBBitmap(0, 40, (uint16_t *)capturedBuffer, capturedWidth, capturedHeight);
          imageStored = true;
          showingCaptured = true;
          captureTime = millis();
          Serial.println("Image Captured");
        }
        esp_camera_fb_return(fb);
      }
    } else {
      Serial.println("Sending Image...");
      uint8_t *jpg_buf = NULL;
      size_t jpg_len = 0;

      if (fmt2jpg(capturedBuffer, capturedLength, capturedWidth, capturedHeight,
                  PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_len)) {

        bool haveRecentFix = hasFix && (millis() - lastFixMillis) < GPS_FIX_MAX_AGE_MS;
        bool ok = sendImageToGallery(jpg_buf, jpg_len,
                                      haveRecentFix, lastLat, lastLng);
        Serial.println(ok ? "Image uploaded" : "Image upload FAILED");
        free(jpg_buf);
      } else {
        Serial.println("JPEG conversion failed");
      }

      free(capturedBuffer);
      capturedBuffer = NULL;
      imageStored = false;
      showingCaptured = false;
      Serial.println("Upload Complete");
    }
  }

  if (previousDeleteState == HIGH && deleteState == LOW) {
    if (imageStored) {
      free(capturedBuffer);
      capturedBuffer = NULL;
      imageStored = false;
      showingCaptured = false;
      Serial.println("Image Deleted");
    }
  }

  if (showingCaptured && (millis() - captureTime >= 4000)) {
    showingCaptured = false;
  }

  previousCaptureState = captureState;
  previousDeleteState  = deleteState;

  if (millis() - lastGpsCycle >= GPS_CYCLE_MS) {
    lastGpsCycle = millis();
    float lat, lon;
    if (acquireGPSFix(lat, lon)) {
      Serial.printf("Fix acquired: %.6f, %.6f\n", lat, lon);
      lastLat = lat; lastLng = lon; hasFix = true; lastFixMillis = millis();

      if (ensureCellularConnected()) {
        bool ok = pushToSheet(lat, lon);
        Serial.println(ok ? "Sheet updated" : "Sheet update failed");
      } else {
        Serial.println("No cellular data — skipping sheet push this cycle");
      }
    } else {
      Serial.println("No GPS fix within timeout — skipping this cycle");
    }
  }
}

// ══════════════════════════════════════════
//  CELLULAR CONNECTION
// ══════════════════════════════════════════
bool ensureCellularConnected() {
  if (modem.isNetworkConnected() && modem.isGprsConnected()) return true;

  Serial.println("Waiting for cellular network...");
  if (!modem.waitForNetwork(30000L)) {
    Serial.println("Network registration failed");
    return false;
  }
  Serial.print("Connecting APN \""); Serial.print(apn); Serial.println("\"...");
  if (!modem.gprsConnect(apn, apnUser, apnPass)) {
    Serial.println("GPRS/data connect failed");
    return false;
  }
  Serial.println("Cellular data connected");
  return true;
}

// ══════════════════════════════════════════
//  GPS
// ══════════════════════════════════════════
bool acquireGPSFix(float &latOut, float &lonOut) {
  Serial.println("Enabling GNSS engine...");
  modem.enableGPS();
  delay(2000);

  unsigned long start = millis();
  float speed, alt, accuracy;
  int   vsat, usat, year, month, day, hour, minute, second;

  while (millis() - start < GPS_TIMEOUT_MS) {
    if (modem.getGPS(&latOut, &lonOut, &speed, &alt, &vsat, &usat,
                      &accuracy, &year, &month, &day, &hour, &minute, &second)) {
      modem.disableGPS();
      delay(500);
      return true;
    }
    Serial.println("Waiting for GNSS fix... (needs open sky view)");
    delay(3000);
  }

  modem.disableGPS();
  return false;
}

// ══════════════════════════════════════════
//  PUSH TO SHEET
// ══════════════════════════════════════════
bool pushToSheet(float lat, float lon) {
  String rowId = String(deviceIdPrefix) + "-" + String(millis());

  StaticJsonDocument<512> doc;
  doc["action"]   = "append";
  doc["rowId"]    = rowId;
  doc["name"]     = deviceName;
  doc["lat"]      = lat;
  doc["lng"]      = lon;
  doc["datetime"] = getTimestamp();
  doc["category"] = deviceCategory;
  doc["notes"]    = "Auto-updated by ESP32-S3 + SIM7600G-H (cellular)";
  doc["images"]   = "";

  String payload;
  serializeJson(doc, payload);

  HttpClient http(gpsClient, scriptHost, 443);
  http.connectionKeepAlive();
  http.beginRequest();
  http.post(scriptPath);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", payload.length());
  http.beginBody();
  http.print(payload);
  http.endRequest();

  int status = http.responseStatusCode();
  String body = http.responseBody();
  Serial.printf("Sheet HTTP %d: %s\n", status, body.c_str());
  http.stop();

  return (status == 200 || status == 302);
}

// ══════════════════════════════════════════
//  IMAGE UPLOAD
// ══════════════════════════════════════════
bool sendImageToGallery(uint8_t *buf, size_t len, bool haveFix, float lat, float lng) {
  if (!ensureCellularConnected()) return false;

  String path = String(GALLERY_PATH) + "?device=" + urlEncode(deviceName);
  if (haveFix) {
    path += "&lat=" + String(lat, 6) + "&lng=" + String(lng, 6);
  }

  HttpClient http(imgClient, GALLERY_HOST, GALLERY_PORT);
  http.connectionKeepAlive();
  http.beginRequest();
  http.post(path.c_str());
  http.sendHeader("Content-Type", "image/jpeg");
  http.sendHeader("Content-Length", len);
  http.beginBody();
  http.write(buf, len);
  http.endRequest();

  int status = http.responseStatusCode();
  String body = http.responseBody();
  Serial.printf("Gallery HTTP %d: %s\n", status, body.c_str());
  http.stop();

  return (status == 200);
}

String urlEncode(const char *s) {
  String out;
  for (const char *p = s; *p; p++) {
    char c = *p;
    if (isalnum((unsigned char)c) || c=='-'||c=='_'||c=='.'||c=='~') out += c;
    else if (c==' ') out += "%20";
    else { char buf[4]; sprintf(buf, "%%%02X", (unsigned char)c); out += buf; }
  }
  return out;
}

String getTimestamp() {
  int year, month, day, hour, minute, second;
  float tzOffset;
  if (modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &tzOffset)) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d", year, month, day, hour, minute);
    return String(buf);
  }
  return "device-uptime-" + String(millis() / 1000) + "s";
}
