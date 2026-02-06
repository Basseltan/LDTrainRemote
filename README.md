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
- [Legoino](https://github.com/corneliusmunz/legoino) - LEGO Powered UP library (vendored in `lib/`)

## Usage

1. Power on the ESP32 board (status LED lights up solid)
2. Power on the Duplo Train (green button until it blinks)
3. Wait for automatic BLE connection (LED changes to slow blink)
4. Use potentiometer for speed, buttons for sounds and LED

## License

GPL-3.0
