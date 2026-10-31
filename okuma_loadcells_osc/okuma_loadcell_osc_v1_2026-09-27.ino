/*
 * ============================================================================
 *  okuma interactive — Load cell module
 *  ESP32 + HX711  ->  WiFi  ->  OSC (UDP)
 * ============================================================================
 *
 * Arduino IDE device: EPS32 Dev Module
 *
 *  Sends the raw HX711 reading (no tare, no calibration). Zeroing and
 *  scaling are done downstream (TouchOSC / TouchDesigner).
 *
 *  Libraries (Arduino Library Manager):
 *    - "HX711 Arduino Library"  by Bogdan Necula (bogde)
 *    - "OSC"                    by Adrian Freed / CNMAT
 *  Board: any ESP32 dev board (esp32 core by Espressif, 2.x or 3.x)
 *
 *  Wiring (defaults below):
 *    HX711 VCC  -> 3V3        (NOT 5V — keeps DOUT at 3.3V logic)
 *    HX711 GND  -> GND
 *    HX711 DT   -> GPIO 18    (same as the working POC)
 *    HX711 SCK  -> GPIO 19
 *
 *  OSC OUT:
 *    OSC_PATH   float   raw 24-bit reading (approx. -8388608 … 8388607),
 *                       optionally smoothed
 * ============================================================================
 */

// ============================================================================
//  CONFIGURATION — edit everything here
// ============================================================================

// ---- Module identity -------------------------------------------------------
#define MODULE_NAME        "okuma-cup1"      // also used as WiFi hostname

// ---- HX711 pins ------------------------------------------------------------
// Safe general-purpose GPIOs. Avoid 0, 2, 12, 15 (strapping) and 6–11 (flash).
#define HX711_DOUT_PIN     18                 // proven in POC
#define HX711_SCK_PIN      19
#define HX711_GAIN         128                // 128 or 64 = channel A, 32 = channel B

// ---- WiFi ------------------------------------------------------------------
#define WIFI_SSID          "WIFI_SSID"
#define WIFI_PASSWORD      "WIFI_PASSWORD"

// DHCP by default (as in POC). Set true for a fixed module IP
// (handy with several modules — check the address is free on your router).
#define USE_STATIC_IP      false
IPAddress LOCAL_IP        (192, 168, 1, 51);
IPAddress GATEWAY         (192, 168, 1, 1);
IPAddress SUBNET          (255, 255, 255, 0);

// ---- OSC target ------------------------------------------------------------
// Your TouchDesigner PC (ipconfig -> Wireless LAN adapter -> IPv4).
// Use 192.168.1.255 to broadcast to the whole subnet.
IPAddress OSC_TARGET_IP   (192, 168, 1, 70);
#define OSC_TARGET_PORT    10000               // matches POC / existing TD OSC In
#define OSC_PATH           "/okuma/cup1"     // one path per module

// ---- Output ----------------------------------------------------------------
#define SEND_RATE_HZ       10                 // max messages/sec (HX711 itself does 10 or 80 SPS)
#define SMOOTHING          0.3f               // 0..1 EMA factor; 1.0 = no smoothing
#define DEADBAND           150.0f               // skip sending if change < this (0 = always send)
#define SPIKE_FILTER       true               // median of last 3 readings: drops single corrupted reads

// ---- Debug -----------------------------------------------------------------
#define SERIAL_BAUD        115200
#define SERIAL_PRINT_VALUES false              // print raw / median / smoothed per reading
                                              // (use Tools > Serial Plotter to see 3 lines)

// ============================================================================
//  END CONFIGURATION
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "HX711.h"

HX711   scale;
WiFiUDP udp;

const unsigned long SEND_INTERVAL_MS = 1000UL / SEND_RATE_HZ;

float   value        = 0.0f;
long    hist[3]      = {0, 0, 0};             // last 3 raw readings for the spike filter
int     histCount    = 0;
bool    haveReading  = false;
float   lastSent     = 1e9f;
unsigned long lastSendMs      = 0;
unsigned long lastWifiCheckMs = 0;

// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.setHostname(MODULE_NAME);              // must come before mode()
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // lower latency
  if (USE_STATIC_IP) WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[wifi] connecting to %s", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[wifi] connected, IP %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[wifi] failed, will keep retrying in loop");
  }
}

void maintainWiFi() {
  if (millis() - lastWifiCheckMs < 5000) return;
  lastWifiCheckMs = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[wifi] lost, reconnecting");
    WiFi.disconnect();
    WiFi.reconnect();
  }
}

// ---------------------------------------------------------------------------
void readScale() {
  if (!scale.is_ready()) return;              // non-blocking
  long raw = scale.read();

  // Median of last 3: a single glitched reading (common on ESP32 with WiFi
  // active) never reaches the smoothing, so it can't smear into a big swing.
  hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = raw;
  if (histCount < 3) histCount++;
  long filtered = raw;
  if (SPIKE_FILTER && histCount == 3) {
    long a = hist[0], b = hist[1], c = hist[2];
    filtered = max(min(a, b), min(max(a, b), c));
  }

  value = haveReading ? value + SMOOTHING * ((float)filtered - value) : (float)filtered;
  haveReading = true;

  if (SERIAL_PRINT_VALUES) Serial.printf("raw:%ld\tmedian:%ld\tvalue:%.0f\n", raw, filtered, value);
}

void sendOsc() {
  if (!haveReading || WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastSendMs < SEND_INTERVAL_MS) return;
  if (DEADBAND > 0.0f && fabsf(value - lastSent) < DEADBAND) return;
  lastSendMs = millis();
  lastSent = value;

  OSCMessage msg(OSC_PATH);
  msg.add(value);
  udp.beginPacket(OSC_TARGET_IP, OSC_TARGET_PORT);
  msg.send(udp);
  udp.endPacket();
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.printf("\n=== %s ===\n", MODULE_NAME);

  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN, HX711_GAIN);
  if (!scale.wait_ready_timeout(2000)) {
    Serial.println("[hx711] NOT FOUND — check wiring. Continuing anyway.");
  }

  connectWiFi();                              // send-only: no udp.begin() / no listening port

  Serial.printf("[osc] sending %s -> %s:%d\n", OSC_PATH,
                OSC_TARGET_IP.toString().c_str(), OSC_TARGET_PORT);
}

void loop() {
  maintainWiFi();
  readScale();
  sendOsc();
}
