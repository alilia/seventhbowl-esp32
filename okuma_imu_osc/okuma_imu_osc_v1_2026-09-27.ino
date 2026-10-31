/*
 * ============================================================================
 *  okuma interactive — IMU / tilt module
 *  ESP32-C3 Super Mini + MPU-6050 (GY-521)  ->  WiFi  ->  OSC (UDP)
 * ============================================================================
 *
 * Arduino IDE device: ESP32C3 Dev Module  (core 3.x also lists "Nologo ESP32C3 Super Mini")
 *   Tools > USB CDC On Boot: Enabled   <- otherwise Serial Monitor stays empty
 *
 *  Measures how far the module is tilted from its zero position, in any
 *  direction, and which way it leans. Tilt is computed on the ESP from a
 *  gyro + accelerometer fusion (steady while tilting, not fooled by short
 *  bumps).
 *
 *  ZERO POSITION: by default the orientation at power-up is 0°. The module
 *  must be still for ~1 s after power-up (it also measures gyro offset then).
 *  Press the BOOT button any time later to re-zero to the current position.
 *  Set ZERO_AT_BOOT false to use "sensor lying flat" as 0° instead.
 *
 *  Libraries (Arduino Library Manager):
 *    - "OSC"  by Adrian Freed / CNMAT
 *  The MPU-6050 is driven directly over I2C (Wire) — no extra library needed.
 *
 *  Wiring (defaults below):
 *    MPU VCC  -> 3V3
 *    MPU GND  -> GND
 *    MPU SDA  -> GPIO 3
 *    MPU SCL  -> GPIO 4
 *    MPU AD0  -> GND or unconnected = address 0x68  (3V3 = 0x69)
 *    MPU INT, XDA, XCL      -> not connected
 *
 *  OSC OUT (one bundle per send; each value on its own address so that
 *  TouchDesigner's OSC In CHOP gives one clean channel per value):
 *    OSC_PATH/tilt       float  total tilt from zero, degrees (0 … 180), any direction
 *    OSC_PATH/dir        float  lean direction, degrees (0 … 360):
 *                               0 = +X edge down, 90 = +Y edge down (axes printed on GY-521)
 *    OSC_PATH/tiltx      float  signed tilt along X, degrees (-180 … 180)
 *    OSC_PATH/tilty      float  signed tilt along Y, degrees (-180 … 180)
 *    OSC_PATH/motion     float  shake/movement, g (0 = at rest)
 *    OSC_PATH/ax ay az   float  raw acceleration, g        (off by default)
 *    OSC_PATH/gx gy gz   float  rotation rate, deg/s       (off by default)
 * ============================================================================
 */

// ============================================================================
//  CONFIGURATION — edit everything here
// ============================================================================

// ---- Module identity -------------------------------------------------------
#define MODULE_NAME        "okuma-imu1"       // also used as WiFi hostname

// ---- MPU-6050 pins / I2C ---------------------------------------------------
// ESP32-C3: avoid 2, 8, 9 (strapping), 18/19 (USB), 20/21 (UART).
#define MPU_SDA_PIN        3
#define MPU_SCL_PIN        4
#define MPU_I2C_ADDR       0x68               // 0x68 (AD0 low) or 0x69 (AD0 high)
#define I2C_CLOCK_HZ       400000             // 400 kHz; drop to 100000 for long wires

// ---- Zero / re-zero --------------------------------------------------------
#define ZERO_AT_BOOT       true               // true = power-up orientation is 0°, false = lying flat is 0°
#define ZERO_BUTTON_PIN    9                  // BOOT button on the C3 Super Mini; -1 = disabled
                                              // (press after boot — holding it during power-up enters flash mode)

// ---- MPU-6050 sensor setup -------------------------------------------------
#define MPU_ACCEL_RANGE    0                  // 0 = ±2g (best for tilt), 1 = ±4g, 2 = ±8g, 3 = ±16g
#define MPU_GYRO_RANGE     1                  // 0 = ±250, 1 = ±500, 2 = ±1000, 3 = ±2000 deg/s
#define MPU_DLPF           3                  // on-chip low-pass: 0=260 1=184 2=94 3=44 4=21 5=10 6=5 Hz
#define CAL_SAMPLES        500                // boot calibration, ~1 s at 2 ms per sample
#define STILL_RANGE_DPS    6.0f               // gyro wobble allowed during calibration; retried if exceeded

// ---- WiFi ------------------------------------------------------------------
#define WIFI_SSID          "WIFI_SSID"
#define WIFI_PASSWORD      "WIFI_PASSWORD"
#define WIFI_TX_POWER_FIX  true               // C3 Super Mini: lower TX power fixes the common
                                              // "never connects" problem of these boards

// DHCP by default. Set true for a fixed module IP
// (handy with several modules — check the address is free on your router).
#define USE_STATIC_IP      false
IPAddress LOCAL_IP        (192, 168, 1, 52);  // cup1 = .51, imu1 = .52
IPAddress GATEWAY         (192, 168, 1, 1);
IPAddress SUBNET          (255, 255, 255, 0);

// ---- OSC target ------------------------------------------------------------
// Your TouchDesigner PC (ipconfig -> Wireless LAN adapter -> IPv4).
// Use 192.168.1.255 to broadcast to the whole subnet.
IPAddress OSC_TARGET_IP   (192, 168, 1, 70);
#define OSC_TARGET_PORT    10000              // same OSC In as the load cell module
#define OSC_PATH           "/okuma/imu1"      // one path per module; values go below it

// Pick what gets sent (each adds addresses to the bundle)
#define SEND_TILT          true               // /tilt /dir
#define SEND_TILT_XY       true               // /tiltx /tilty
#define SEND_MOTION        true               // /motion
#define SEND_ACCEL         false              // /ax /ay /az
#define SEND_GYRO          false              // /gx /gy /gz

// ---- Output ----------------------------------------------------------------
#define SENDS_PER_MINUTE   1800               // 3600 = 60/s, 1800 = 30/s (smooth), 600 = 10/s,
                                              // 60 = 1/s, 6 = every 10 s
#define READS_PER_SECOND   200                // sensor reads feeding the tilt filter — keep 100…500
                                              // (independent of the send rate; fewer = less accurate tilt)
#define TILT_FILTER        0.98f              // gyro trust: 0.95 snappier/noisier … 0.99 smoother/slower
#define ACCEL_REJECT_G     0.3f               // ignore accel for tilt while shaken harder than this
#define SMOOTHING          0.4f               // 0..1 EMA on motion / accel / gyro; 1.0 = no smoothing

// ---- Debug -----------------------------------------------------------------
#define SERIAL_BAUD        115200
#define SERIAL_PRINT_VALUES false             // print values each send (Tools > Serial Plotter)

// ============================================================================
//  END CONFIGURATION
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <OSCBundle.h>

WiFiUDP udp;

// MPU-6050 registers
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

const unsigned long READ_INTERVAL_US = 1000000UL / READS_PER_SECOND;
const unsigned long SEND_INTERVAL_MS = 60000UL / SENDS_PER_MINUTE;
const float ACCEL_LSB_PER_G   = 16384.0f / (1 << MPU_ACCEL_RANGE);
const float GYRO_LSB_PER_DPS  = 131.0f   / (1 << MPU_GYRO_RANGE);
const float RAD2DEG           = 57.2957795f;
const float DEG2RAD           = 0.0174532925f;

float gyroBias[3] = {0, 0, 0};                // deg/s
float up[3]       = {0, 0, 1};                // fused "up" direction in sensor frame (unit vector)
float ref[3]      = {0, 0, 1};                // zero orientation
float e1[3]       = {1, 0, 0};                // reference-plane axes (for direction / tiltx / tilty)
float e2[3]       = {0, 1, 0};

float tilt = 0, tiltDir = 0, tiltX = 0, tiltY = 0;   // degrees
float motion = 0;                                    // g
float accel[3] = {0, 0, 0}, gyro[3] = {0, 0, 0};     // smoothed, for optional output

bool  haveReading  = false;
bool  mpuOk        = false;
int   i2cErrors    = 0;
unsigned long lastReadUs      = 0;
unsigned long lastSendMs      = 0;
unsigned long lastWifiCheckMs = 0;
unsigned long lastButtonMs    = 0;

// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.setHostname(MODULE_NAME);              // must come before mode()
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // lower latency
  if (USE_STATIC_IP) WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WIFI_TX_POWER_FIX) WiFi.setTxPower(WIFI_POWER_8_5dBm);

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
//  Small vector helpers
float dot3(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

bool normalize3(float v[3]) {
  float m = sqrtf(dot3(v, v));
  if (m < 1e-6f) return false;
  v[0] /= m; v[1] /= m; v[2] /= m;
  return true;
}

// Set the zero orientation and build two axes in the plane around it,
// aligned with the sensor's X / Y as closely as possible.
void setReference(const float r[3]) {
  float n[3] = {r[0], r[1], r[2]};
  if (!normalize3(n)) return;
  for (int k = 0; k < 3; k++) ref[k] = n[k];

  float x[3] = {1, 0, 0};
  float d = dot3(x, ref);
  for (int k = 0; k < 3; k++) e1[k] = x[k] - d * ref[k];
  if (!normalize3(e1) || fabsf(d) > 0.95f) {  // X nearly vertical: use Y instead
    float y[3] = {0, 1, 0};
    d = dot3(y, ref);
    for (int k = 0; k < 3; k++) e1[k] = y[k] - d * ref[k];
    normalize3(e1);
  }
  e2[0] = ref[1]*e1[2] - ref[2]*e1[1];        // e2 = ref × e1
  e2[1] = ref[2]*e1[0] - ref[0]*e1[2];
  e2[2] = ref[0]*e1[1] - ref[1]*e1[0];
}

// ---------------------------------------------------------------------------
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_I2C_ADDR, 1) != 1) return -1;
  return Wire.read();
}

// raw[0..6] = ax, ay, az, temp, gx, gy, gz
bool readRaw(int16_t raw[7]) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_I2C_ADDR, 14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();                 // two statements: read order matters
    uint8_t lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

bool initMpu() {
  int who = readReg(REG_WHO_AM_I);
  if (who < 0) {
    Serial.printf("[mpu] no answer at 0x%02X — check wiring / AD0\n", MPU_I2C_ADDR);
    return false;
  }
  // Genuine MPU-6050 says 0x68; many clone boards report 0x70/0x72/0x98 and still work.
  if (who != 0x68) Serial.printf("[mpu] WHO_AM_I = 0x%02X (clone?) — continuing\n", who);

  writeReg(REG_PWR_MGMT_1, 0x80);             // reset
  delay(100);
  bool ok = true;
  ok &= writeReg(REG_PWR_MGMT_1,   0x01);     // wake, clock = gyro X PLL (more stable)
  ok &= writeReg(REG_SMPLRT_DIV,   0x00);     // 1 kHz internal rate (with DLPF on)
  ok &= writeReg(REG_CONFIG,       MPU_DLPF & 0x07);
  ok &= writeReg(REG_GYRO_CONFIG,  (MPU_GYRO_RANGE  & 0x03) << 3);
  ok &= writeReg(REG_ACCEL_CONFIG, (MPU_ACCEL_RANGE & 0x03) << 3);
  delay(50);
  if (ok) Serial.println("[mpu] ready");
  return ok;
}

// Measures gyro offset and the resting "up" direction. Retries (up to 3x)
// if the module is being moved, then accepts whatever it got.
void calibrate() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.println("[mpu] calibrating — keep still");
    double gSum[3] = {0, 0, 0}, aSum[3] = {0, 0, 0};
    float gMin[3] = { 1e9,  1e9,  1e9}, gMax[3] = {-1e9, -1e9, -1e9};
    int n = 0;
    int16_t raw[7];
    for (int i = 0; i < CAL_SAMPLES; i++) {
      if (readRaw(raw)) {
        for (int k = 0; k < 3; k++) {
          float g = raw[4 + k] / GYRO_LSB_PER_DPS;
          gSum[k] += g;
          aSum[k] += raw[k] / ACCEL_LSB_PER_G;
          gMin[k] = min(gMin[k], g);
          gMax[k] = max(gMax[k], g);
        }
        n++;
      }
      delay(2);
    }
    if (n == 0) return;

    float wobble = 0;
    for (int k = 0; k < 3; k++) wobble = max(wobble, gMax[k] - gMin[k]);
    if (wobble > STILL_RANGE_DPS && attempt < 3) {
      Serial.printf("[mpu] moving (%.1f deg/s wobble), retrying\n", wobble);
      continue;
    }

    float a[3];
    for (int k = 0; k < 3; k++) { gyroBias[k] = gSum[k] / n; a[k] = aSum[k] / n; }
    for (int k = 0; k < 3; k++) up[k] = a[k];
    normalize3(up);
    if (ZERO_AT_BOOT) setReference(up);
    else { float flat[3] = {0, 0, 1}; setReference(flat); }

    Serial.printf("[mpu] gyro bias  x %.2f  y %.2f  z %.2f deg/s\n", gyroBias[0], gyroBias[1], gyroBias[2]);
    Serial.printf("[mpu] zero = %s\n", ZERO_AT_BOOT ? "power-up position" : "lying flat");
    haveReading = true;
    return;
  }
}

// ---------------------------------------------------------------------------
void readImu() {
  unsigned long now = micros();
  if (now - lastReadUs < READ_INTERVAL_US) return;   // non-blocking
  float dt = (now - lastReadUs) / 1e6f;
  lastReadUs = now;
  if (dt > 0.1f) dt = 0.1f;                   // after a stall, don't integrate a huge step

  if (!mpuOk) {                                // try to bring a missing sensor back
    static unsigned long lastRetryMs = 0;
    if (millis() - lastRetryMs > 2000) {
      lastRetryMs = millis();
      mpuOk = initMpu();
      if (mpuOk) { calibrate(); lastReadUs = micros(); }
    }
    return;
  }

  int16_t raw[7];
  if (!readRaw(raw)) {
    if (++i2cErrors > 20) {                   // ~0.1 s of failures -> re-init
      Serial.println("[mpu] I2C errors, re-initialising");
      mpuOk = false;
      i2cErrors = 0;
    }
    return;
  }
  i2cErrors = 0;

  float a[3], g[3];
  for (int k = 0; k < 3; k++) {
    a[k] = raw[k]     / ACCEL_LSB_PER_G;
    g[k] = raw[4 + k] / GYRO_LSB_PER_DPS - gyroBias[k];
  }
  float aMag = sqrtf(dot3(a, a));

  // 1) Rotate "up" by the gyro:  up -= (w × up) * dt
  float w[3] = {g[0] * DEG2RAD, g[1] * DEG2RAD, g[2] * DEG2RAD};
  float c[3] = { w[1]*up[2] - w[2]*up[1],
                 w[2]*up[0] - w[0]*up[2],
                 w[0]*up[1] - w[1]*up[0] };
  for (int k = 0; k < 3; k++) up[k] -= c[k] * dt;

  // 2) Pull gently toward gravity from the accelerometer — unless being shaken
  if (aMag > 0.1f && fabsf(aMag - 1.0f) < ACCEL_REJECT_G) {
    for (int k = 0; k < 3; k++)
      up[k] = TILT_FILTER * up[k] + (1.0f - TILT_FILTER) * (a[k] / aMag);
  }
  normalize3(up);

  // 3) Tilt relative to the zero orientation
  float d  = constrain(dot3(up, ref), -1.0f, 1.0f);
  float lx = -dot3(up, e1);                   // lean toward +X edge
  float ly = -dot3(up, e2);                   // lean toward +Y edge
  tilt  = acosf(d) * RAD2DEG;
  tiltX = atan2f(lx, d) * RAD2DEG;
  tiltY = atan2f(ly, d) * RAD2DEG;
  if (tilt > 2.0f) {                          // direction is meaningless near 0°: hold last value
    tiltDir = atan2f(ly, lx) * RAD2DEG;
    if (tiltDir < 0) tiltDir += 360.0f;
  }

  // 4) Motion + optional raw values
  motion += SMOOTHING * (fabsf(aMag - 1.0f) - motion);
  for (int k = 0; k < 3; k++) {
    accel[k] += SMOOTHING * (a[k] - accel[k]);
    gyro[k]  += SMOOTHING * (g[k] - gyro[k]);
  }
}

void checkZeroButton() {
  if (ZERO_BUTTON_PIN < 0 || !haveReading) return;
  if (digitalRead(ZERO_BUTTON_PIN) == LOW && millis() - lastButtonMs > 500) {
    lastButtonMs = millis();
    setReference(up);
    Serial.println("[zero] current position is now 0°");
  }
}

// ---------------------------------------------------------------------------
void addValue(OSCBundle &b, const char *name, float v) {
  char addr[64];
  snprintf(addr, sizeof(addr), "%s/%s", OSC_PATH, name);
  b.add(addr).add(v);
}

void sendOsc() {
  if (!haveReading || WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = millis();

  OSCBundle b;
  if (SEND_TILT)    { addValue(b, "tilt", tilt);   addValue(b, "dir", tiltDir); }
  if (SEND_TILT_XY) { addValue(b, "tiltx", tiltX); addValue(b, "tilty", tiltY); }
  if (SEND_MOTION)  { addValue(b, "motion", motion); }
  if (SEND_ACCEL)   { addValue(b, "ax", accel[0]); addValue(b, "ay", accel[1]); addValue(b, "az", accel[2]); }
  if (SEND_GYRO)    { addValue(b, "gx", gyro[0]);  addValue(b, "gy", gyro[1]);  addValue(b, "gz", gyro[2]); }

  udp.beginPacket(OSC_TARGET_IP, OSC_TARGET_PORT);
  b.send(udp);
  udp.endPacket();

  if (SERIAL_PRINT_VALUES)
    Serial.printf("tilt:%.1f\tdir:%.0f\ttiltx:%.1f\ttilty:%.1f\tmotion:%.3f\n",
                  tilt, tiltDir, tiltX, tiltY, motion);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);                                 // C3 USB serial needs a moment
  Serial.printf("\n=== %s ===\n", MODULE_NAME);

  if (ZERO_BUTTON_PIN >= 0) pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  mpuOk = initMpu();
  if (mpuOk) calibrate();
  else Serial.println("[mpu] NOT FOUND — will keep retrying. Continuing anyway.");

  connectWiFi();                              // send-only: no udp.begin() / no listening port

  Serial.printf("[osc] sending %s/* -> %s:%d, %d per minute\n", OSC_PATH,
                OSC_TARGET_IP.toString().c_str(), OSC_TARGET_PORT, SENDS_PER_MINUTE);
  lastReadUs = micros();
}

void loop() {
  maintainWiFi();
  readImu();
  checkZeroButton();
  sendOsc();
}
