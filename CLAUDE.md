# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based remote control for LEGO Duplo Train using BLE. Built with PlatformIO (Arduino framework). Supports two board configurations: `nodemcu-32s` (default) and `wemosbat`.

## Build Commands

```bash
# Build (default: nodemcu-32s)
pio run -e nodemcu-32s

# Or for wemosbat board
pio run -e wemosbat

# Upload to device
pio run -t upload -e nodemcu-32s

# OTA upload (wemosbat_ota env has IP preconfigured)
pio run -t upload -e wemosbat_ota

# Serial monitor (115200 baud)
pio device monitor -b 115200
```

## Architecture

**Main entry point**: `src/main.cpp` — single-file project, all logic lives here.

**MyHub class** (defined at top of `main.cpp`): Custom subclass of `Lpf2Hub` that adds Duplo Train-specific methods missing from the base library. These send raw BLE byte commands:
- `activateBaseSpeaker()` / `playSound(byte)` — sound playback via raw BLE writes
- `activateRgbLight()` / `setLedColor(Color)` — LED control via raw BLE writes
- Source: adapted from [legoino issue #44](https://github.com/corneliusmunz/legoino/issues/44#issuecomment-985384328)

**BLE connection flow** (must follow this pattern):
```cpp
myHub.init();  // In setup(), starts BLE scan
// In loop():
if (myHub.isConnecting()) {
    myHub.connectHub();
    if (myHub.isConnected()) {
        myHub.setHubName("DavidTrainHub");
    }
}
```

**Vendored Legoino library**: `lib/legoino-master/`
- Upstream: https://github.com/Basseltan/legoino (fork with NimBLE 2.x patches)
- Provides base `Lpf2Hub` class for LEGO Powered UP hub communication
- Do not convert to remote dependency; referenced via `lib_extra_dirs` in platformio.ini
- **Patched for NimBLE 2.x compatibility** in `Lpf2Hub.cpp` (originally written for NimBLE 1.x):
  - BLE scan uses `getResults()` (blocking) instead of `start()` (non-blocking in 2.x)
  - Scan duration converted from seconds to milliseconds (NimBLE 2.x API change)
  - Scan callbacks use both `onDiscovered()` and `onResult()` (NimBLE 2.x split)
  - `_isConnecting` set before `stop()` to prevent race condition with main task
  - `clearResults()` called before each scan to avoid duplicate filtering
  - `onDisconnect` callback signature updated with `int reason` parameter (NimBLE 2.x)

**Auto-reconnect**: The `loop()` tracks connection state via `gWasConnected` flag. On disconnect, it automatically calls `myHub.init()` to restart BLE scanning.

**WiFi / Telnet / OTA** (optional, guarded by `#ifdef WIFI_ENABLED`):
- Enabled by creating `include/credentials.h` from `include/credentials.h.template` with WiFi SSID/password
- `__has_include("credentials.h")` at compile time sets `WIFI_ENABLED`; without it, WiFi code is excluded entirely
- `setupWiFi()`: non-blocking `WiFi.begin()` with `setAutoReconnect(true)`, ArduinoOTA + ESPTelnet setup
- `handleWiFi()`: called every loop iteration, handles OTA + Telnet when WiFi is connected
- ArduinoOTA `onStart` callback stops motor as safety measure before CPU-blocking update
- `debugLog(format, ...)`: printf-style function that outputs to Serial always, and to Telnet when connected

## Hardware Pin Mapping

| Pin | Function |
|-----|----------|
| 25 | BTN_MUSIC |
| 26 | BTN_LICHT |
| 27 | BTN_WASSER |
| 14 | BTN_STOP |
| 34 | PTI_SPEED (potentiometer) |
| 16 | LED_ONBOARD (not used) |
| 32 | BAT_VOLTAGE (battery monitoring via ADC) |
| 33 | LED_LOW_BAT (multi-purpose status/battery LED) |

**Status LED** (GPIO 33): Solid ON = disconnected, fast blink (100ms) = connecting, slow blink (500ms) = connected, very fast blink (50ms) = battery low (overrides connection status).

**Battery Monitoring** (GPIO 32): Voltage divider (2x 220kOhm), checked every 5 seconds, warning at 3.5V for 18650 Li-Ion (AMS1117-3.3 regulator needs headroom). ADC reference calibrated to 3.54V (chip-specific). Uses `movingAvg` over 50 readings (sampled every loop iteration) to filter ESP32 ADC outliers.

**Potentiometer** (GPIO 34): ADC mapped to speed -100..+100 with ±20 deadzone. Uses `movingAvg` over 10 readings for smoothing. Calibration constants `POT_MIN=1191` / `POT_MAX=2941` in `handlePoti()` — adjust if hardware gives different range.

## Code Structure

The `loop()` calls these handlers when connected:

**`handleButtons()`** — Processes button events using Bounce2 (50ms debounce):
- Music: plays `HORN` sound
- Light: cycles through LED colors (11 colors via `getNextColor()`)
- Water: plays `WATER_REFILL` sound
- Stop: stops motor, plays `BRAKE`, sets **stop latch** (`gStopLatch`) — poti must return to center (speed=0) before new speed is accepted

**`handlePoti()`** — Reads potentiometer and controls motor speed:
- Auto-plays `STATION_DEPARTURE` when transitioning from stop to forward
- Respects stop latch: ignores input until poti returns to center after STOP

**`handleStatusLed()`** / **`checkBatteryVoltage()`** — Run every loop iteration regardless of connection state.

**Command rate limiting**: `ensureCommandInterval()` enforces 150ms minimum between hub BLE commands to prevent overload. Uses `delay()` to pad if needed.

## Available Sound Effects

From `DuploTrainBaseSound` enum (in `Lpf2HubConst.h`):
- `BRAKE = 3`, `STATION_DEPARTURE = 5`, `WATER_REFILL = 7`, `HORN = 9`

Used via `myHub.playSound((byte)DuploTrainBaseSound::SOUND_NAME)`.

## Dependencies (from platformio.ini)

- `h2zero/NimBLE-Arduino@^2.3.7` — BLE library
- `thomasfredericks/Bounce2@^2.72` — Button debouncing (50ms interval)
- `jchristensen/movingAvg@^2.3.0` — Potentiometer smoothing
- `lennarthennigs/ESPTelnet@^2.2.3` — Telnet server for remote debug output

Note: `lib_ldf_mode = deep+` is used to detect conditional includes (`__has_include` for WiFi).

## Working with Motors

Use `myHub.setBasicMotorSpeed(port, speed)` for Duplo Train motor control. Speed range is -100 to 100. Port is hardcoded to `PoweredUpHubPort::A`.

## When Making Changes

- Run `pio run -e nodemcu-32s` to verify build
- BLE/motor changes require hardware testing; check serial logs for hub connect and motor commands
- Ask before upgrading NimBLE or Legoino library versions
- When adding new hub commands, wrap with `ensureCommandInterval()` to respect rate limiting
- Use `debugLog()` instead of `Serial.print`/`println` for debug output (mirrors to Telnet)
- WiFi credentials go in `include/credentials.h` (git-ignored); template at `include/credentials.h.template`

## Known Limitations

- **Action Brick Color Detection**: The Duplo Train color sensor produces very noisy readings on normal track, making reliable detection of colored action bricks difficult. Not implemented yet; experimental code is on the `feature/color-sensor-detection` branch.
