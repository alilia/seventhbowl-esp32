/*
 * okuma interactive — I2C scanner (diagnostic)
 * Scans the bus every 3 s with SDA=3/SCL=4, then with the pins swapped.
 * MPU-6050 should show up at 0x68 (or 0x69 if AD0 is tied to 3V3).
 * Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled
 */
#include <Wire.h>

// Arduino IDE tends to switch this setting off when the board or folder changes;
// without it the Serial Monitor stays empty. Stop the compile instead.
#if defined(CONFIG_IDF_TARGET_ESP32C3) && (!defined(ARDUINO_USB_CDC_ON_BOOT) || ARDUINO_USB_CDC_ON_BOOT == 0)
#error "Tools > USB CDC On Boot must be Enabled for the ESP32-C3 Super Mini (otherwise Serial Monitor stays empty)"
#endif

#define PIN_A 3
#define PIN_B 4

void scan(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  Serial.printf("SDA=%d SCL=%d : ", sda, scl);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf("0x%02X ", a); found++; }
  }
  Serial.println(found ? "" : "nothing");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== I2C scan ===");
}

void loop() {
  scan(PIN_A, PIN_B);
  scan(PIN_B, PIN_A);
  Serial.println();
  delay(3000);
}
