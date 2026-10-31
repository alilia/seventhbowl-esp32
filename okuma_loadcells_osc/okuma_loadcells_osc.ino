/*
 * ============================================================================
 *  okuma interactive — load cells module
 *  ESP32 + several HX711  ->  WiFi  ->  OSC (UDP), one OSC path per cell
 * ============================================================================
 *
 *  Arduino IDE device: ESP32 Dev Module
 *
 *  ALL SETTINGS ARE IN THE config.h TAB — including the list of cells.
 *  Adding a cell = one line there; this file never needs editing.
 *
 *  Each cell sends its raw HX711 reading (no tare, no calibration) as a float
 *  to its own path, e.g. /okuma/cup1. Zeroing and scaling happen downstream
 *  (TouchOSC bridge).
 *
 *  Cells that share an SCK pin form a group and are read in one pass: when
 *  every HX711 in the group has a new reading, the clock is pulsed once and
 *  all DT pins are read bit by bit in parallel (the HX711 library can't do
 *  this — it reads one chip at a time, which would corrupt the others).
 *
 *  Per cell: median-of-3 spike filter -> EMA smoothing -> deadband -> OSC.
 *  A cell that delivers no data, or only 0 / -1 (data line stuck — seen with
 *  a GND on the wrong pin and with a dead HX711 board), stops sending and
 *  says so on Serial; it picks up again by itself once it reads normally.
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "config.h"

#define NUM_CELLS (sizeof(CELLS) / sizeof(CELLS[0]))

enum Health { H_WAITING, H_OK, H_NO_DATA, H_STUCK, H_DISABLED };

struct Cell {
  const char *label;                          // last part of the path, e.g. "cup1"
  float   smoothing;
  float   deadband;
  int     group;                              // index into groups[], -1 if disabled
  long    hist[3];
  int     histCount;
  long    lastRaw;
  float   value;
  bool    haveReading;
  float   lastSent;
  unsigned long lastSendMs;
  unsigned long lastDataMs;
  int     stuckCount;
  Health  health;
};

struct Group {                                // all cells on one SCK pin
  int     sck;
  int     members[NUM_CELLS];
  int     count;
  bool    waiting;                            // some members ready, others not yet
  unsigned long firstReadyMs;
};

Cell    cells[NUM_CELLS];
Group   groups[NUM_CELLS];
int     numGroups = 0;
WiFiUDP udp;
bool    booting = true;                       // quiet health messages until the boot report

portMUX_TYPE hxMux = portMUX_INITIALIZER_UNLOCKED;
const int GAIN_PULSES = (HX711_GAIN == 128) ? 1 : (HX711_GAIN == 64) ? 3 : 2;
const unsigned long SEND_INTERVAL_MS = 1000UL / SEND_RATE_HZ;
unsigned long lastWifiCheckMs  = 0;
unsigned long lastPlotMs       = 0;

// ---------------------------------------------------------------------------
//  Pin checks (classic ESP32)
// ---------------------------------------------------------------------------
bool pinExists(int p)   { return p >= 0 && p <= 39 && p != 20 && p != 24 && !(p >= 28 && p <= 31); }
bool pinIsFlash(int p)  { return p >= 6 && p <= 11; }
bool pinIsSerial(int p) { return p == 1 || p == 3; }
bool pinInputOnly(int p){ return p >= 34 && p <= 39; }
bool pinIsBoot(int p)   { return p == 0 || p == 2 || p == 12 || p == 15; }

// Returns an error text, or nullptr if the cell's pins are usable.
// A shared SCK is fine; DT pins must be unique and never anyone's SCK.
const char *checkPins(size_t i) {
  int dt = CELLS[i].dt, sck = CELLS[i].sck;
  if (!pinExists(dt) || !pinExists(sck))         return "pin does not exist on ESP32";
  if (pinIsFlash(dt) || pinIsFlash(sck))         return "pins 6-11 are the flash (incl. CMD) - unusable";
  if (pinIsSerial(dt) || pinIsSerial(sck))       return "pins 1/3 are USB serial - unusable";
  if (pinInputOnly(sck))                         return "SCK on an input-only pin (34-39) - use it for DT instead";
  if (dt == sck)                                 return "DT and SCK are the same pin";
  for (size_t j = 0; j < i; j++) {
    if (cells[j].health == H_DISABLED) continue;
    int odt = CELLS[j].dt, osck = CELLS[j].sck;
    if (dt == odt)                               return "DT pin already used by an earlier cell";
    if (dt == osck)                              return "DT pin is an earlier cell's SCK";
    if (sck == odt)                              return "SCK pin is an earlier cell's DT";
    if (strcmp(CELLS[i].path, CELLS[j].path) == 0) return "same OSC path as an earlier cell";
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
//  WiFi
// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.setHostname(MODULE_NAME);              // must come before mode()
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // lower latency
  if (USE_STATIC_IP) WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WIFI_TX_POWER_LOW) WiFi.setTxPower(WIFI_POWER_8_5dBm);

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
//  Cell health
// ---------------------------------------------------------------------------
void setHealth(int i, Health h) {
  Cell &c = cells[i];
  if (c.health == h) return;
  c.health = h;
  if (booting) return;                        // the boot report shows it instead
  switch (h) {
    case H_OK:
      Serial.printf("[%s] ok, raw %ld\n", c.label, c.lastRaw);
      break;
    case H_NO_DATA:
      Serial.printf("[%s] NO DATA for %d ms - HX711 not connected or not powered? (DT %d, SCK %d)\n",
                    c.label, NO_DATA_MS, CELLS[i].dt, CELLS[i].sck);
      break;
    case H_STUCK:
      Serial.printf("[%s] STUCK at 0/-1 - data line not driven: check GND (real GND, not CMD), "
                    "VCC, DT/SCK wires, or a faulty HX711 board\n", c.label);
      break;
    default: break;
  }
}

void checkTimeout(int i) {
  if (millis() - cells[i].lastDataMs > NO_DATA_MS) setHealth(i, H_NO_DATA);
}

// ---------------------------------------------------------------------------
//  Reading
// ---------------------------------------------------------------------------
// Clock 24 bits out of every HX711 on this SCK at once, plus the gain pulses.
// Interrupts are off for these ~80 µs so WiFi can't stretch a clock pulse
// (an SCK held high > 60 µs would power the HX711s down).
void clockGroup(const Group &g, long out[]) {
  int32_t v[NUM_CELLS];
  for (int m = 0; m < g.count; m++) v[m] = 0;

  portENTER_CRITICAL(&hxMux);
  for (int bit = 0; bit < 24; bit++) {
    digitalWrite(g.sck, HIGH);
    delayMicroseconds(1);
    for (int m = 0; m < g.count; m++)
      v[m] = (v[m] << 1) | digitalRead(CELLS[g.members[m]].dt);
    digitalWrite(g.sck, LOW);
    delayMicroseconds(1);
  }
  for (int p = 0; p < GAIN_PULSES; p++) {
    digitalWrite(g.sck, HIGH);
    delayMicroseconds(1);
    digitalWrite(g.sck, LOW);
    delayMicroseconds(1);
  }
  portEXIT_CRITICAL(&hxMux);

  for (int m = 0; m < g.count; m++) {
    if (v[m] & 0x800000) v[m] |= (int32_t)0xFF000000;   // 24-bit two's complement
    out[m] = v[m];
  }
}

void processReading(int i, long raw) {
  Cell &c = cells[i];
  c.lastDataMs = millis();
  c.lastRaw = raw;

  // A connected load cell never sits exactly at 0 or -1: that is a dead data line.
  if (raw == 0 || raw == -1) {
    if (++c.stuckCount >= STUCK_READS) setHealth(i, H_STUCK);
    return;
  }
  c.stuckCount = 0;
  if (c.health != H_OK) {                     // (re)start filtering cleanly
    c.histCount = 0;
    c.haveReading = false;
    setHealth(i, H_OK);
  }

  // Median of last 3: a single glitched reading never reaches the smoothing.
  c.hist[0] = c.hist[1]; c.hist[1] = c.hist[2]; c.hist[2] = raw;
  if (c.histCount < 3) c.histCount++;
  long filtered = raw;
  if (SPIKE_FILTER && c.histCount == 3) {
    long a = c.hist[0], b = c.hist[1], d = c.hist[2];
    filtered = max(min(a, b), min(max(a, b), d));
  }

  c.value = c.haveReading ? c.value + c.smoothing * ((float)filtered - c.value) : (float)filtered;
  c.haveReading = true;

  if (SERIAL_DEBUG_CELL == i + 1)
    Serial.printf("raw:%ld\tmedian:%ld\tvalue:%.0f\n", raw, filtered, c.value);
}

// Read the group when all its working HX711s have a new value; if one lags
// (or is missing) for GROUP_WAIT_MS, read the ready ones without it.
// Cells already flagged NO DATA aren't waited for — they rejoin when ready.
void serviceGroup(Group &g) {
  bool ready[NUM_CELLS];
  int nReady = 0, nNeeded = 0;
  for (int m = 0; m < g.count; m++) {
    int i = g.members[m];
    ready[m] = digitalRead(CELLS[i].dt) == LOW;      // DT low = conversion ready
    if (ready[m]) nReady++;
    else if (cells[i].health != H_NO_DATA) nNeeded++;
  }

  if (nReady == 0) {
    g.waiting = false;
    for (int m = 0; m < g.count; m++) checkTimeout(g.members[m]);
    return;
  }
  if (nNeeded > 0) {                           // someone still converting
    if (!g.waiting) { g.waiting = true; g.firstReadyMs = millis(); }
    if (millis() - g.firstReadyMs < GROUP_WAIT_MS) return;
  }
  g.waiting = false;

  long vals[NUM_CELLS];
  clockGroup(g, vals);
  for (int m = 0; m < g.count; m++) {
    if (ready[m]) processReading(g.members[m], vals[m]);
    else checkTimeout(g.members[m]);
  }
}

// ---------------------------------------------------------------------------
//  Sending
// ---------------------------------------------------------------------------
void sendCell(int i) {
  Cell &c = cells[i];
  if (c.health != H_OK || !c.haveReading || WiFi.status() != WL_CONNECTED) return;
  if (millis() - c.lastSendMs < SEND_INTERVAL_MS) return;
  if (c.deadband > 0.0f && fabsf(c.value - c.lastSent) < c.deadband) return;
  c.lastSendMs = millis();
  c.lastSent = c.value;

  OSCMessage msg(CELLS[i].path);
  msg.add(c.value);
  udp.beginPacket(OSC_TARGET_IP, OSC_TARGET_PORT);
  msg.send(udp);
  udp.endPacket();
}

void plotAll() {
  if (!SERIAL_PRINT_VALUES || SERIAL_DEBUG_CELL > 0) return;
  if (millis() - lastPlotMs < SEND_INTERVAL_MS) return;
  lastPlotMs = millis();
  for (int i = 0; i < (int)NUM_CELLS; i++) {
    if (cells[i].health != H_OK) continue;
    Serial.printf("%s:%.0f\t", cells[i].label, cells[i].value);
  }
  Serial.println();
}

// ---------------------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------------------
void setupCells() {
  for (int i = 0; i < (int)NUM_CELLS; i++) {
    Cell &c = cells[i];
    const CellConfig &cfg = CELLS[i];
    const char *slash = strrchr(cfg.path, '/');
    c.label        = slash ? slash + 1 : cfg.path;
    c.smoothing    = cfg.smoothing > 0.0f ? cfg.smoothing : SMOOTHING;
    c.deadband     = cfg.deadband  > 0.0f ? cfg.deadband  : DEADBAND;
    c.group        = -1;
    c.histCount    = 0;
    c.haveReading  = false;
    c.lastSent     = 1e9f;
    c.lastSendMs   = 0;
    c.lastDataMs   = millis();
    c.stuckCount   = 0;
    c.health       = H_WAITING;

    const char *err = checkPins(i);
    if (err) {
      c.health = H_DISABLED;
      Serial.printf("[%s] DISABLED - %s (DT %d, SCK %d)\n", c.label, err, cfg.dt, cfg.sck);
      continue;
    }
    if (pinIsBoot(cfg.dt) || pinIsBoot(cfg.sck))
      Serial.printf("[%s] warning: uses a boot pin (0/2/12/15) - may block flashing or booting\n", c.label);

    // join (or start) the group for this SCK pin
    int g = 0;
    while (g < numGroups && groups[g].sck != cfg.sck) g++;
    if (g == numGroups) {
      groups[g].sck = cfg.sck;
      groups[g].count = 0;
      groups[g].waiting = false;
      numGroups++;
      pinMode(cfg.sck, OUTPUT);
      digitalWrite(cfg.sck, LOW);             // SCK low = HX711 powered up
    }
    groups[g].members[groups[g].count++] = i;
    c.group = g;
    pinMode(cfg.dt, pinInputOnly(cfg.dt) ? INPUT : INPUT_PULLUP);
  }

  // Read for up to 2 s so every cell gets a first value, then print the boot report.
  unsigned long start = millis();
  while (millis() - start < 2000) {
    for (int g = 0; g < numGroups; g++) serviceGroup(groups[g]);
    bool allSeen = true;
    for (int i = 0; i < (int)NUM_CELLS; i++)
      if (cells[i].health != H_DISABLED && cells[i].health != H_OK) allSeen = false;
    if (allSeen) break;
    delay(1);
  }
  booting = false;

  Serial.printf("\n  #  path                    DT  SCK         smooth  deadband  status\n");
  for (int i = 0; i < (int)NUM_CELLS; i++) {
    Cell &c = cells[i];
    const CellConfig &cfg = CELLS[i];
    char sckText[16];
    if (c.group >= 0 && groups[c.group].count > 1)
      snprintf(sckText, sizeof(sckText), "%d (x%d)", cfg.sck, groups[c.group].count);
    else
      snprintf(sckText, sizeof(sckText), "%d", cfg.sck);
    Serial.printf("%3d  %-22s %3d  %-10s   %5.2f  %8.0f  ", i + 1, cfg.path, cfg.dt, sckText,
                  c.smoothing, c.deadband);
    switch (c.health) {
      case H_OK:       Serial.printf("ok, raw %ld\n", c.lastRaw); break;
      case H_STUCK:    Serial.println("STUCK at 0/-1 - check GND/VCC/wires/board"); break;
      case H_DISABLED: Serial.println("DISABLED (see above)"); break;
      default:         Serial.println("NO DATA - check wiring / power"); break;
    }
  }
  Serial.println("  (x2, x3 ... = cells sharing that SCK)\n");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.printf("\n=== %s - %u load cell(s) ===\n", MODULE_NAME, (unsigned)NUM_CELLS);

  setupCells();
  connectWiFi();                              // send-only: no udp.begin() / no listening port

  Serial.printf("[osc] sending to %s:%d\n", OSC_TARGET_IP.toString().c_str(), OSC_TARGET_PORT);
}

void loop() {
  maintainWiFi();
  for (int g = 0; g < numGroups; g++) serviceGroup(groups[g]);
  for (int i = 0; i < (int)NUM_CELLS; i++) sendCell(i);
  plotAll();
}
