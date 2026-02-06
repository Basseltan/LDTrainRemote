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

# Serial monitor (115200 baud)
pio device monitor -b 115200
```

## Architecture

**Main entry point**: `src/main.cpp`
- Single-mode implementation: interactive control via buttons and potentiometer
- On connection, hub name is automatically set to "DavidTrainHub"

**BLE connection flow** (must follow this pattern):
```cpp
myHub.init();  // In setup(), starts BLE scan
// In loop():
if (myHub.isConnecting()) {
    myHub.connectHub();
    if (myHub.isConnected()) {
        // Hub name is set to "DavidTrainHub" here
        // Ready for commands
    }
}
```

**Vendored Legoino library**: `lib/legoino-master/`
- Provides `Lpf2Hub` class for LEGO Powered UP hub communication
- Key APIs: `setBasicMotorSpeed()`, `setLedColor()`, `playSound()`, `stopBasicMotor()`
- Uses NimBLE-Arduino for BLE connectivity
- Do not convert to remote dependency; referenced via `lib_extra_dirs` in platformio.ini

## Hardware Pin Mapping

| Pin | Function |
|-----|----------|
| 25 | BTN_MUSIC |
| 26 | BTN_LICHT |
| 27 | BTN_WASSER |
| 14 | BTN_STOP |
| 12 | PTI_SPEED (potentiometer) |
| 16 | LED_ONBOARD (status indicator) |
| 32 | BAT_VOLTAGE (battery monitoring via ADC) |
| 33 | LED_LOW_BAT (low battery warning) |

**Status LED**: GPIO 33 (LED_LOW_BAT - multi-purpose indicator)
- Solid ON: Disconnected
- Fast blink (100ms): Connecting to hub
- Slow blink (500ms): Connected to hub
- Very fast blink (50ms): **Battery low** (overrides connection status)

Note: Onboard LED (GPIO 16) is not used.

**Battery Monitoring**: GPIO 32 (ADC input with voltage divider)
- Voltage divider: 2x 220kOhm resistors (divides battery voltage by 2)
- Cutoff voltage: 3.0V (typical for 18650 Li-Ion)
- Check interval: 5 seconds
- Warning via LED_LOW_BAT (GPIO 33) very fast blink when voltage < 3.0V

Potentiometer maps ADC to speed (-100 to 100) with a deadzone of ±20 around center. ADC is configured for 12-bit resolution in `setup()`.

**Potentiometer Calibration**: Hardware calibration constants in `handlePoti()`:
- `POT_MIN = 1126` - ADC value at minimum position (maps to -100)
- `POT_MAX = 2969` - ADC value at maximum position (maps to +100)
- Adjust these values if you get a different speed range than ±100

## Code Structure

The code runs in interactive mode with four main handlers called from `loop()`:

**`handleStatusLed()`** - Multi-purpose LED status indicator (GPIO 33):
- Uses `millis()` for timing to avoid blocking
- Shows connection status: solid ON (disconnected), fast blink (connecting), slow blink (connected)
- **Battery warning has priority**: Very fast blink (50ms) when `gBatteryLow` is true
- Combines connection status and battery warning in one LED

**`checkBatteryVoltage()`** - Battery voltage monitoring:
- Reads ADC every 5 seconds
- Converts ADC value to actual battery voltage (accounting for voltage divider)
- Sets `gBatteryLow` flag when voltage < 3.0V
- Prints battery info to serial for debugging

**`handleButtons()`** - Processes button events using Bounce2:
- Music button: plays `HORN` sound
- Light button: cycles through LED colors (11 colors via `getNextColor()`)
- Water button: plays `WATER_REFILL` sound
- Stop button: stops motor (sets speed to 0) and plays `BRAKE` sound

**`handlePoti()`** - Reads potentiometer and controls motor speed:
- Applies ±20 deadzone around center to prevent jitter
- Auto-plays `STATION_DEPARTURE` sound when transitioning from stop to forward motion
- Updates global `gSpeed` variable tracked for stop button

**Auto-reconnect**: The `loop()` tracks connection state via `gWasConnected` flag. On disconnect, it automatically calls `myHub.init()` to restart BLE scanning.

## Available Sound Effects

From `DuploTrainBaseSound` enum (in `Lpf2HubConst.h`):
- `BRAKE = 3`
- `STATION_DEPARTURE = 5`
- `WATER_REFILL = 7`
- `HORN = 9`

Used via `myHub.playSound((byte)DuploTrainBaseSound::SOUND_NAME)`.

## Dependencies (from platformio.ini)

- `h2zero/NimBLE-Arduino@^1.4.2` - BLE library
- `thomasfredericks/Bounce2@^2.72` - Button debouncing (25ms interval)

Note: `lib_ldf_mode = chain+` is used; changes to library layout may affect linking.

## Working with Motors

Use `myHub.setBasicMotorSpeed(port, speed)` for Duplo Train motor control. Speed range is -100 to 100. The code currently hardcodes `port = (byte)PoweredUpHubPort::A`. For more advanced motors (Tacho/Absolute), see `lib/legoino-master/README.md`.

## When Making Changes

- Run `pio run -e nodemcu-32s` to verify build
- BLE/motor changes require hardware testing; check serial logs for hub connect and motor commands
- Ask before upgrading NimBLE or Legoino library versions
