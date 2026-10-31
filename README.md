# Seventh Bowl — ESP32 firmware

ESP32 firmware and tools for the sensor modules of **Seventh Bowl**, an interactive tea-ceremony
installation. Each module reads its sensors and sends the values over WiFi as OSC to the laptop
running the TouchOSC bridge (OSC → MIDI). TouchOSC, Pure Data and TouchDesigner files are not
part of this repo.

> Internal names still use the project's working title *okuma* (`okuma_imu_osc`, OSC paths
> `/okuma/...`). They are kept as they are because the TouchOSC bridge listens on those paths.

## What's in here

| Folder | What it is |
|---|---|
| Folder | Board | What it is |
|---|---|---|
| `okuma_imu_osc/` | ESP32-C3 Super Mini | **Pitcher module**: MPU-6050 (GY-521). Tilt, lean direction, motion, battery → OSC |
| `okuma_loadcells_osc/` | ESP32 Dev Module (DevKitC, 38-pin) | **Load cells module**: several HX711 load cells (gaiwan + cups), one OSC path per cell |
| `okuma_i2c_scan/` | ESP32-C3 Super Mini | **Tool**: scans the I2C bus on GPIO 3/4 (both pin orders) — run it first when a sensor isn't found |
| `secrets-template/SeventhBowlSecrets/` | — | Template of the shared WiFi/network settings library (placeholders only) |
| `.github/workflows/build.yml` | — | Compiles every sketch on GitHub after each push (see *Build check*) |

Each folder is a separate Arduino sketch: open its `.ino` in Arduino IDE. Everything else in the
working folder is ignored by git (see `.gitignore`).

## First-time setup on a computer

1. **Arduino IDE 2** with the board package **esp32 by Espressif Systems** (3.3.11 tested) and
   the library **OSC** by Adrian Freed / CNMAT (Library Manager).
2. **Shared WiFi settings (once per computer):** copy the folder
   `secrets-template/SeventhBowlSecrets` into your Arduino libraries folder
   (`<sketchbook>\libraries\`, sketchbook = *File > Preferences > Sketchbook location*), then
   edit the copy there: WiFi name, password, network prefix and OSC target. Every module uses
   this one file, and it lives outside the repo, so the password never reaches GitHub. Without
   it the compile stops with `SeventhBowlSecrets.h: No such file or directory`.
   **Keep the placeholders in the repo's template** — the build check fails if the template's
   password is changed. New network = edit the libraries copy, re-upload every module.
3. **Board settings** (Tools menu) — they differ per module:
   - Pitcher and I2C scan: **ESP32C3 Dev Module** (*not* "Ozobot DRVkit" or anything S3/C6),
     USB CDC On Boot **Enabled** (otherwise the Serial Monitor stays empty), Flash Mode **DIO**
   - Load cells: **ESP32 Dev Module**
   - Port: the board's COM port

## Pitcher module (`okuma_imu_osc`)

**Wiring** (ESP32-C3 Super Mini)

| GY-521 | ESP32-C3 |
|---|---|
| VCC | 3V3 (5V also works — the GY-521 has its own regulator) |
| GND | G |
| SDA | GPIO 3 |
| SCL | GPIO 4 |
| AD0 | open or GND → address 0x68 |

Battery: `battery + → 100 kΩ → GPIO 0 → 100 kΩ → GND`. **Never connect battery + straight to a
pin** — a full LiPo is 4.2 V, the pin's limit is 3.3 V.

**OSC out** (to `OSC_TARGET_IP`:10000)

| Address | Value |
|---|---|
| `/okuma/imu1/tilt` | total tilt from flat, 0–180° |
| `/okuma/imu1/dir` | lean direction, 0–360° (0 = +X edge down, 90 = +Y edge down) |
| `/okuma/imu1/tiltx`, `/tilty` | signed tilt per axis, degrees |
| `/okuma/imu1/motion` | shaking, g (0 at rest) |
| `/okuma/imu1/battery` | 0–100 % (own message, every `BATTERY_SEND_S` seconds) |

All other settings (pins, rates, filters, what gets sent) are in the configuration block at the
top of the `.ino`.

**Startup:** keep the module still for ~1 s after power-up — it measures the gyro offset then.

## Load cells module (`okuma_loadcells_osc`)

Settings, the list of cells and the pin plan are in `config.h` (WiFi in the shared `SeventhBowlSecrets` library); the `.ino`
doesn't need editing. Adding a cell = wire its HX711 (VCC → 3V3, GND → GND, SCK → the shared
clock pin 19, DT → its own free pin) and add one line to `CELLS` at the bottom of `config.h`.
Full steps and the pin plan are in the header of `config.h`. Each cell sends its raw HX711
reading as a float to its own path (`/okuma/gaiwan`, `/okuma/cup1` …); zeroing and scaling
happen in the TouchOSC bridge. The boot report on Serial (115200) lists every cell as "ok" or
what is wrong with it.

## Build check (GitHub Actions)

`build.yml` compiles every sketch for its own board on every push, using the example settings
(no real WiFi details involved). A red ✗ on the commit means the sketch doesn't compile — open
the *Actions* tab to see the error. It uses the settings template, and fails if real WiFi details were committed by mistake.
It does not flash a board; uploading stays in Arduino IDE.

## Adding another module

1. Make it use the shared settings: `#include <SeventhBowlSecrets.h>`, no WiFi details in the
   sketch, its own `USE_STATIC_IP` / `MODULE_IP_LAST` and
   `IPAddress LOCAL_IP(NET_PREFIX, MODULE_IP_LAST);` — like the pitcher module.
2. Add `!/<folder>/` to `.gitignore` and the folder to the right board's `sketches` list in
   `build.yml` (or a new board entry, plus any libraries it needs).
3. Before committing, check that no WiFi password appears in the changed files.

## Troubleshooting

- **No Serial output:** the sketch only prints at startup. Open the Serial Monitor (115200),
  then press RESET on the board. Check USB CDC On Boot = Enabled.
- **No OSC:** the network must be **2.4 GHz** (the C3 can't join 5 GHz). `OSC_TARGET_IP` must
  be on the same network as the laptop: check its IPv4 with `ipconfig` (a `.255` broadcast
  address only works within the same first three numbers). On a new network Windows may mark
  it *Public* and the firewall can block TouchOSC — set it to *Private*.
- **Sensor not found** (`[mpu] no answer at 0x68`): run `okuma_i2c_scan`. If nothing answers
  and SDA/SCL read ~1 V instead of ~3.3 V at idle, the GY-521 is dead — swap it (happened once).
- **"This chip is ESP32-C3, not ESP32-S3"**: wrong board selected.
- **Changes vanish:** Arduino IDE auto-saves the open sketch over files changed outside it.
  Close the sketch before editing elsewhere — and with git, `git diff` shows exactly what changed.
