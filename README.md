# LDTrainRemote

Remote control for LEGO Duplo Train based on ESP32 using BLE. Supports board configurations `nodemcu-32s` and `wemosbat` (ESP32 with 18650 Battery Holder).

## Features

- Connect to LEGO Duplo Train Hub via Bluetooth Low Energy
- Control train speed with potentiometer (-100 to +100)
- Play sounds (Horn, Water Refill, Brake, Station Departure)
- Change LED color on the train hub
- Emergency stop button (requires poti to return to center before resuming)
- Battery voltage monitoring with low battery warning LED
- Status LED indicating connection state and battery level
- Command rate limiting to prevent hub overload
- Moving average filter for smooth potentiometer readings
- Optional WiFi with Telnet debug output and OTA firmware updates

## Hardware

| GPIO Pin | Function |
|----------|----------|
| 25 | Music button (Horn) |
| 26 | Light button (LED color cycle) |
| 27 | Water button (Water Refill sound) |
| 14 | Stop button |
| 12 | Speed potentiometer |
| 32 | Battery voltage (ADC via voltage divider 2x 220kOhm) |
| 33 | Status / Low battery LED |

### Status LED (GPIO 33)

| State | LED Pattern |
|-------|-------------|
| Disconnected | Solid ON |
| Connecting | Fast blink (100ms) |
| Connected | Slow blink (500ms) |
| Battery low | Very fast blink (50ms) |

## Build & Upload

Requires [PlatformIO](https://platformio.org/).

```bash
# Build (default board)
pio run -e nodemcu-32s

# Or for wemosbat board
pio run -e wemosbat

# Upload
pio run -t upload -e nodemcu-32s

# Serial monitor
pio device monitor -b 115200
```

## Dependencies

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) - BLE library
- [Bounce2](https://github.com/thomasfredericks/Bounce2) - Button debouncing
- [movingAvg](https://github.com/JChristensen/movingAvg) - Potentiometer smoothing
- [ESPTelnet](https://github.com/LennartHennigs/ESPTelnet) - Telnet server for remote debug
- [Legoino](https://github.com/Basseltan/legoino) - LEGO Powered UP library (vendored in `lib/`, fork with NimBLE 2.x patches)

## WiFi / Telnet / OTA (optional)

WiFi is optional and runs in the background. Train control works without WiFi.

1. Copy `include/credentials.h.template` to `include/credentials.h`
2. Fill in your WiFi SSID and password
3. Build and upload — WiFi connects automatically

With WiFi connected:
- **Telnet** (port 23): All debug output is mirrored to Telnet clients (`telnet <ip>`)
- **OTA**: Upload firmware over WiFi with `pio run -t upload --upload-port <ip>`

Without `credentials.h`, WiFi/Telnet/OTA are excluded from the build entirely.

## Usage

1. Power on the ESP32 board (status LED lights up solid)
2. Power on the Duplo Train (green button until it blinks)
3. Wait for automatic BLE connection (LED changes to slow blink)
4. Use potentiometer for speed, buttons for sounds and LED

## Known Limitations

- **Action Brick Color Detection**: The Duplo Train color sensor produces very noisy readings on normal track (mostly green, blue, yellow), making reliable detection of colored action bricks difficult. This feature is currently not implemented but planned for a future version. Experimental code is available on the `feature/color-sensor-detection` branch.

## License

GPL-3.0
