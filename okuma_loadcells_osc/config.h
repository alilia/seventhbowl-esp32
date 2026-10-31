/*
 * ============================================================================
 *  Seventh Bowl — load cells module — CONFIGURATION — v3 (2026-10-28)
 *  This is the file you edit. The .ino tab holds the code.
 *  WiFi name/password, the network addresses and the OSC target are shared
 *  by all Seventh Bowl modules, in the SeventhBowlSecrets library (Arduino
 *  libraries folder, not in git). Setup: see README.
 *
 *  v3: five cells on the shared clock (SCK 19): gaiwan + cups 1–4.
 *      cup4 is the 50 kg half-bridge cell (+ 2 resistors), with the settings
 *      from the 50 kg test sketch: smoothing 0.15, deadband 50 (~1 g at
 *      ~43 counts/g). Its wiring notes are in okuma_loadcell50kg_osc/config.h.
 *      The .ino is unchanged from v2.
 * ============================================================================
 *
 *  ADDING A NEW LOAD CELL
 *    1. Wire the HX711: VCC -> 3V3, GND -> GND (the real GND pin, not CMD),
 *       SCK -> the shared clock pin (19), DT -> its own free pin from the
 *       PIN PLAN below.
 *       Load cell -> HX711: red E+, black E-, green A+, white A-.
 *    2. Add one line to CELLS (bottom of this file):  { "/okuma/cup2", 16, 19 },
 *    3. Upload. The boot report on Serial (115200) lists every cell with
 *       "ok" or what is wrong with it.
 *    4. Optional: set SERIAL_DEBUG_CELL to the new line's number to watch its
 *       raw / median / value in Tools > Serial Plotter, then set it back to -1.
 *    5. In TouchOSC: capture none / empty / tea for the new cell.
 *
 *  SHARED CLOCK
 *    Cells with the same SCK pin are read together in one pass: the sketch
 *    waits until every HX711 on that clock has a new reading, then clocks
 *    them all at once and reads each DT. So any number of cells can share
 *    one SCK; each needs only its own DT pin. A cell can also have its own SCK.
 *    If one HX711 on a shared clock stops answering, the others carry on
 *    (after a short wait, GROUP_WAIT_MS).
 *    With many cells on one clock, keep the SCK wires short, or split them
 *    into two clock groups (e.g. SCK 19 and SCK 23).
 *
 *  PIN PLAN (ESP32 Dev Module / DevKitC, 38-pin)
 *    SCK (shared):  19
 *    DT, in suggested order:  18, 5, 17, 16, 4, 21, 22, 23, 32, 33, 25, 26, 27, 14, 13
 *    DT also fine on the input-only pins 34, 35, 36 (VP), 39 (VN) — never as SCK.
 *  Off-limits (the sketch refuses them): 6–11 (flash, incl. CMD), 1 / 3 (USB serial),
 *  20, 24, 28–31 (don't exist). Avoid 0, 2, 12, 15 (boot pins: the sketch warns).
 *  On 30-pin boards some of these pins are missing — check the board's labels.
 *
 *  Libraries (Arduino Library Manager):
 *    - "OSC"  by Adrian Freed / CNMAT
 *  (The HX711s are read directly by this sketch; the HX711 library is no
 *  longer needed — leaving it installed does no harm.)
 *  Board: ESP32 Dev Module (esp32 core by Espressif, 2.x or 3.x)
 * ============================================================================
 */
#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// ---- Module identity -------------------------------------------------------
#define MODULE_NAME        "okuma-scales"     // also used as WiFi hostname

// ---- WiFi + network addresses ----------------------------------------------
// WiFi name/password, network prefix, gateway, subnet and OSC target come from
// the shared SeventhBowlSecrets library. Only this module's own address is here:
#include <SeventhBowlSecrets.h>
#define USE_STATIC_IP      false              // true = fixed IP below, false = DHCP
#define MODULE_IP_LAST     51                 // fixed IP = network prefix + this (load cells .51, imu1 .52)
IPAddress LOCAL_IP(NET_PREFIX, MODULE_IP_LAST);

#define WIFI_TX_POWER_LOW  true               // 8.5 dBm instead of ~19.5: smaller current spikes,
                                              // prevents "Brownout detector was triggered" resets
                                              // on weak USB supplies. Plenty of range for a room.

// ---- OSC target ------------------------------------------------------------
// OSC_TARGET_IP is in SeventhBowlSecrets.h (shared by all modules).
#define OSC_TARGET_PORT    10000              // TouchOSC bridge receive port

// ---- Defaults for every cell (a cell line can override smoothing/deadband) --
#define HX711_GAIN         128                // 128 or 64 = channel A, 32 = channel B (same for all cells)
#define SEND_RATE_HZ       10                 // max messages/sec per cell (HX711 does 10 SPS)
#define SMOOTHING          0.3f               // 0..1 EMA factor; 1.0 = no smoothing
#define DEADBAND           150.0f             // skip sending if change < this many counts
#define SPIKE_FILTER       true               // median of last 3 readings: drops single bad reads

// ---- Health checks (a cell that fails them stops sending until it recovers) --
#define NO_DATA_MS         1000               // no reading for this long -> "no data"
#define STUCK_READS        10                 // this many 0 / -1 readings in a row -> "stuck"
#define GROUP_WAIT_MS      150                // shared clock: wait this long for a slow/missing
                                              // HX711 before reading the others without it
                                              // (10 SPS boards: 150; 80 SPS boards: 30)

// ---- Debug -----------------------------------------------------------------
#define SERIAL_BAUD        115200
#define SERIAL_PRINT_VALUES false             // every cell's value on one line (Serial Plotter)
#define SERIAL_DEBUG_CELL  -1                 // CELLS line number (1 = first) -> raw / median / value
                                              // of that one cell on every reading; -1 = off

// ============================================================================
//  LOAD CELLS — one line per HX711
//    { "osc path", DT pin, SCK pin }            <- same SCK = shared clock
//    { "osc path", DT pin, SCK pin, smoothing, deadband }   <- per-cell override
//  Leave out smoothing / deadband (or write 0) to use the defaults above.
//  Turn a cell off by putting // in front of its line.
// ============================================================================
struct CellConfig {                           // (format of a line — no need to edit)
  const char *path;
  int   dt;
  int   sck;
  float smoothing;
  float deadband;
  CellConfig(const char *p, int d, int s, float sm = 0, float db = 0)
    : path(p), dt(d), sck(s), smoothing(sm), deadband(db) {}
};

const CellConfig CELLS[] = {
  { "/okuma/gaiwan", 18, 19 },
  { "/okuma/cup1",    5, 19 },
  { "/okuma/cup2",   17, 19 },
  { "/okuma/cup3",   16, 19 },
  // 50 kg half-bridge cell: ~43 counts/g -> smoothing 0.15, deadband 50 (~1 g)
  { "/okuma/cup4",    4, 19, 0.15f, 50.0f },
};
