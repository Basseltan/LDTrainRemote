#include <Lpf2Hub.h>      //legoino
#include <Lpf2HubConst.h> //legoino
#include <Bounce2.h>      //bounce2
#include <movingAvg.h>    //movingAvg
#include <stdarg.h>       //for debugLog variadic function
#include <esp_sleep.h>    //deep sleep and wakeup

// WiFi/OTA/Telnet libraries (always available on ESP32, usage guarded by WIFI_ENABLED)
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPTelnet.h>

// Optional WiFi support: copy include/credentials.h.template to include/credentials.h
// and fill in your WiFi credentials. Without credentials.h, WiFi/Telnet/OTA are disabled.
#if __has_include("credentials.h")
  #include "credentials.h"
  #define WIFI_ENABLED
#endif

// new MyHub class extends Lpf2Hub with helper functions for our specific use case (color sensor, sounds, LED control)
// taken from https://github.com/corneliusmunz/legoino/issues/44#issuecomment-985384328
class MyHub
  : public Lpf2Hub
{
public:
  void activateBaseSpeaker()
  {
    byte setSoundMode[8] = { 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01 };
    WriteValue(setSoundMode, 8);
  }

  void playSound(byte sound)
  {
    byte playSound[6] = { 0x81, 0x01, 0x11, 0x51, 0x01, sound };
    WriteValue(playSound, 6);
  }

  void activateRgbLight()
  {
    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
    byte setColorMode[8] = { 0x41, port, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    WriteValue(setColorMode, 8);
  }

  void setLedColor(Color color)
  {
    byte port = getPortForDeviceType((byte)DeviceType::HUB_LED);
    byte setColor[6] = { 0x81, port, 0x11, 0x51, 0x00, color };
    Lpf2Hub::WriteValue(setColor, 6);
  }
};
// Lpf2Hub myHub;
MyHub myHub;  // Use our custom MyHub class with helper functions

byte port = (byte)PoweredUpHubPort::A;

// Pin declaration (hardware mapping)
// - BTN_*: digital buttons wired to the ESP32 (configured INPUT_PULLUP in setup()).
// - PTI_SPEED: analog potentiometer (used to select discrete speed steps).
// - LED_ONBOARD: status LED (fast blink = connecting, slow blink = connected)
//   GPIO 16 for ESP32 Board with 18650 Battery Holder
// - BAT_VOLTAGE: battery voltage monitoring via voltage divider (2x 220kOhm)
// - LED_LOW_BAT: low battery warning LED
// If you change pins here, update wiring/docs and any tests that read them.
#define BTN_MUSIC 25
#define BTN_LICHT 26
#define BTN_WASSER 27
#define BTN_STOP 14
#define PTI_SPEED 34
#define LED_ONBOARD 16
#define BAT_VOLTAGE 32
#define LED_LOW_BAT 33

Bounce pbMusic = Bounce();
Bounce pbLight = Bounce();
Bounce pbWater = Bounce();
Bounce pbStop = Bounce();
int gLastStatePtiSpeed = 0;
movingAvg avgPotiSpeed(10);  // Moving average over 10 readings
bool gWasConnected = false;  // Track connection state for reconnect

static short gColor = -1;  // Start at -1 so first increment gives 0
static int gSpeed = 0;
bool gStopLatch = false;   // After STOP, poti must return to 0 before new speed is accepted

// LED status indicator state (now using LED_LOW_BAT for both status and battery warning)
unsigned long gLedLastToggle = 0;
bool gLedState = false;

// Command rate limiting to prevent hub overload
unsigned long gLastCommandTime = 0;
const unsigned long COMMAND_MIN_INTERVAL = 150;  // Minimum 150ms between commands

// Battery monitoring (18650 Li-Ion with voltage divider 2x 220kOhm)
const float BAT_VOLTAGE_MIN = 3.5;        // Cutoff voltage: 3.0V
const float BAT_VOLTAGE_DIVIDER = 2.0;    // Voltage divider ratio
const float ADC_REFERENCE = 3.54;         // ESP32 ADC reference voltage (calibrated: 3.3 × 2.05/1.91)
const int ADC_MAX = 4095;                 // 12-bit ADC
const unsigned long BAT_CHECK_INTERVAL = 5000;  // Check every 5 seconds
unsigned long gLastBatCheck = 0;
bool gBatteryLow = false;
movingAvg avgBatVoltage(50);  // Moving average over 50 readings to filter ADC outliers

// Deep sleep configuration
const unsigned long INACTIVITY_TIMEOUT = 5UL * 60UL * 1000UL;  // 5 minutes in milliseconds
const uint64_t PERIODIC_WAKE_INTERVAL_US = 5ULL * 1000000ULL;  // 5 seconds in microseconds
unsigned long gLastActivityTime = 0;

// WiFi / Telnet / OTA
#ifdef WIFI_ENABLED
ESPTelnet telnet;
bool gWifiConnected = false;
#endif

// Ensure minimum time between hub commands to prevent overload
void ensureCommandInterval()
{
  unsigned long now = millis();
  unsigned long elapsed = now - gLastCommandTime;

  if (elapsed < COMMAND_MIN_INTERVAL) {
    delay(COMMAND_MIN_INTERVAL - elapsed);
  }

  gLastCommandTime = millis();
}

// Debug logging: always prints to Serial, also to Telnet if connected
void debugLog(const char* format, ...)
{
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  Serial.println(buf);
#ifdef WIFI_ENABLED
  if (telnet.isConnected()) {
    telnet.println(buf);
  }
#endif
}

#ifdef WIFI_ENABLED
void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setAutoReconnect(true);

  // ArduinoOTA setup
  ArduinoOTA.setHostname("DuploTrainRemote");

  ArduinoOTA.onStart([]() {
    // Emergency stop motor before OTA update blocks CPU
    gSpeed = 0;
    if (myHub.isConnected()) {
      myHub.setBasicMotorSpeed(port, 0);
    }
    Serial.println("OTA: update starting...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA: update complete");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA: error %u\r\n", error);
  });

  ArduinoOTA.begin();

  // Telnet setup
  telnet.onConnect([](String ip) {
    Serial.print("Telnet client connected from ");
    Serial.println(ip);
  });

  telnet.onDisconnect([](String ip) {
    Serial.print("Telnet client disconnected from ");
    Serial.println(ip);
  });

  telnet.begin();

  Serial.println("WiFi/OTA/Telnet initialized, connecting...");
}

void handleWiFi()
{
  if (WiFi.status() == WL_CONNECTED) {
    if (!gWifiConnected) {
      gWifiConnected = true;
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
    }
    ArduinoOTA.handle();
    telnet.loop();
  } else {
    if (gWifiConnected) {
      gWifiConnected = false;
      Serial.println("WiFi disconnected");
    }
  }
}
#endif

// Prepare hardware and enter deep sleep with timer + button wakeup
void enterDeepSleep()
{
  debugLog("Entering deep sleep (inactivity timeout)...");

  // Stop motor and shut down hub if connected (saves hub battery too)
  if (myHub.isConnected()) {
    gSpeed = 0;
    ensureCommandInterval();
    myHub.setBasicMotorSpeed(port, 0);
    ensureCommandInterval();
    myHub.shutDownHub();
    delay(200);
  }

  // Deinitialize BLE stack
  NimBLEDevice::deinit();

#ifdef WIFI_ENABLED
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

  // Turn off all outputs
  digitalWrite(LED_LOW_BAT, LOW);
  digitalWrite(LED_ONBOARD, LOW);

  // Configure wakeup sources: timer (5s heartbeat) + LICHT button (ext0, immediate)
  esp_sleep_enable_timer_wakeup(PERIODIC_WAKE_INTERVAL_US);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_LICHT, 0);  // Wake on LOW (button pressed)

  debugLog("Deep sleep now.");
  Serial.flush();

  esp_deep_sleep_start();
  // Never reaches here. On wake, ESP32 resets and setup() runs.
}

// Timer wakeup: blink LED briefly, then re-enter deep sleep immediately.
// Runs before any heavy initialization (no BLE, no WiFi, no Serial).
void handleTimerWakeup()
{
  pinMode(LED_LOW_BAT, OUTPUT);

  digitalWrite(LED_LOW_BAT, HIGH);
  delay(100);
  digitalWrite(LED_LOW_BAT, LOW);

  esp_sleep_enable_timer_wakeup(PERIODIC_WAKE_INTERVAL_US);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_LICHT, 0);
  esp_deep_sleep_start();
}

Color getNextColor()
{
    gColor++;
    if (gColor >= NUM_COLORS)
    {
        gColor = 0;
    }
    return (Color)gColor;
}

void handlePoti()
{
  int ptiSpeed = avgPotiSpeed.reading(analogRead(PTI_SPEED));
  gLastStatePtiSpeed = ptiSpeed;

  // Potentiometer calibration - adjust these values to match your hardware
  // Calculated for pot that currently gives -45 to +45 range (to extend to -100 to +100)
  const int POT_MIN = 1191;   // ADC value at minimum position (calibrated)
  const int POT_MAX = 2941;   // ADC value at maximum position (calibrated)

  // Map calibrated ADC range to -100..100
  long mapped = map(ptiSpeed, POT_MIN, POT_MAX, -100, 100);
  int speed = (int)constrain(mapped, -100, 100);

  // Apply deadzone to avoid jitter around center
  const int DEADZONE = 20;
  if (abs(speed) <= DEADZONE) speed = 0;

  // After STOP button, poti must return to 0 before accepting new speed
  if (gStopLatch) {
    if (speed == 0) {
      gStopLatch = false;
      debugLog("POT: returned to center - accepting speed again");
    }
    return;
  }

  if (speed != gSpeed)
  {
    gLastActivityTime = millis();
    debugLog("POT: speed change %d -> %d", gSpeed, speed);

    if (gSpeed == 0 && speed > 0)
    {
      debugLog("POT: transitioning 0->forward: play STATION_DEPARTURE");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::STATION_DEPARTURE);
    }

    gSpeed = speed;
    ensureCommandInterval();
    myHub.setBasicMotorSpeed(port, speed);
  }
}

void handleStatusLed()
{
  unsigned long now = millis();
  unsigned long blinkInterval;

  // Battery low has highest priority - very fast blink (50ms)
  if (gBatteryLow) {
    blinkInterval = 50;
  }
  // Connected - slow blink (500ms)
  else if (myHub.isConnected()) {
    blinkInterval = 500;
  }
  // Connecting - fast blink (100ms)
  else if (myHub.isConnecting()) {
    blinkInterval = 100;
  }
  // Disconnected - LED on (solid)
  else {
    digitalWrite(LED_LOW_BAT, HIGH);
    gLedState = true;
    return;
  }

  // Toggle LED at the determined interval
  if (now - gLedLastToggle >= blinkInterval) {
    gLedState = !gLedState;
    digitalWrite(LED_LOW_BAT, gLedState ? HIGH : LOW);
    gLedLastToggle = now;
  }
}

void checkBatteryVoltage()
{
  // Feed ADC reading into moving average every loop iteration
  int adcValue = avgBatVoltage.reading(analogRead(BAT_VOLTAGE));

  unsigned long now = millis();

  // Only evaluate and log periodically
  if (now - gLastBatCheck < BAT_CHECK_INTERVAL) {
    return;
  }

  gLastBatCheck = now;

  // Convert averaged ADC value to actual battery voltage
  float pinVoltage = (adcValue * ADC_REFERENCE) / ADC_MAX;
  float batteryVoltage = pinVoltage * BAT_VOLTAGE_DIVIDER;

  // Debug output
  debugLog("Battery: ADC=%d, Pin=%.2fV, Bat=%.2fV", adcValue, pinVoltage, batteryVoltage);

  // Check if battery is below cutoff voltage
  if (batteryVoltage < BAT_VOLTAGE_MIN) {
    if (!gBatteryLow) {
      debugLog("WARNING: Battery voltage low!");
      gBatteryLow = true;
    }
  } else {
    gBatteryLow = false;
  }
}

void handleButtons()
{
  if(pbMusic.update())
  {
    if(pbMusic.fell())
    {
      gLastActivityTime = millis();
      debugLog("Button MUSIC pressed - playing HORN");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::HORN);
    }
  }
  if(pbLight.update())
  {
    if(pbLight.fell())
    {
      gLastActivityTime = millis();
      Color c = getNextColor();
      debugLog("Button LIGHT pressed - setting LED color to %d", (int)c);
      ensureCommandInterval();
      myHub.setLedColor(c);
    }
  }
  if(pbWater.update())
  {
    if(pbWater.fell())
    {
      gLastActivityTime = millis();
      debugLog("Button WATER pressed - playing WATER_REFILL");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::WATER_REFILL);
    }
  }
  if(pbStop.update())
  {
    if(pbStop.fell())
    {
      gLastActivityTime = millis();
      debugLog("Button STOP pressed - playing BRAKE and stopping motor");
      gSpeed = 0;
      gStopLatch = true;  // Block poti until it returns to center
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::BRAKE);
      ensureCommandInterval();
      myHub.setBasicMotorSpeed(port, 0);
    }
  }
}

void setup() {
  // On deep sleep wakeup, check if a button is actually pressed.
  // If not, it's a timer wakeup -> blink LED and go back to sleep.
  // (esp_sleep_get_wakeup_cause() is unreliable with timer+ext1 combined)
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  if (wakeupCause == ESP_SLEEP_WAKEUP_TIMER || wakeupCause == ESP_SLEEP_WAKEUP_EXT1) {
    pinMode(BTN_MUSIC, INPUT_PULLUP);
    pinMode(BTN_LICHT, INPUT_PULLUP);
    pinMode(BTN_WASSER, INPUT_PULLUP);
    pinMode(BTN_STOP, INPUT_PULLUP);
    delay(5);  // Let pullups settle

    bool buttonPressed = (digitalRead(BTN_MUSIC) == LOW ||
                          digitalRead(BTN_LICHT) == LOW ||
                          digitalRead(BTN_WASSER) == LOW ||
                          digitalRead(BTN_STOP) == LOW);
    if (!buttonPressed) {
      handleTimerWakeup();  // Never returns
    }
  }

  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

  if (wakeupCause != ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("Woke from deep sleep: button press");
  } else {
    Serial.println("Normal boot (power-on or reset)");
  }

  // Set ADC resolution explicitly (ESP32 supports 9-12 bit)
  analogReadResolution(12);

  // Configure status LED (also used for low battery warning)
  pinMode(LED_LOW_BAT, OUTPUT);
  digitalWrite(LED_LOW_BAT, LOW);

  // Note: Onboard LED (GPIO 16) is not used
  // Battery voltage monitoring pin (GPIO 32) is ADC input, no pinMode needed

  // Attach buttons with INPUT_PULLUP and set debounce interval
  pbMusic.attach(BTN_MUSIC, INPUT_PULLUP);
  pbMusic.interval(50);
  pbLight.attach(BTN_LICHT, INPUT_PULLUP);
  pbLight.interval(50);
  pbWater.attach(BTN_WASSER, INPUT_PULLUP);
  pbWater.interval(50);
  pbStop.attach(BTN_STOP, INPUT_PULLUP);
  pbStop.interval(50);

  avgPotiSpeed.begin();
  avgBatVoltage.begin();

  myHub.init();
  debugLog("myHub.init() called - BLE scan started");

#ifdef WIFI_ENABLED
  setupWiFi();
#endif

  gLastActivityTime = millis();
}

void loop() {
#ifdef WIFI_ENABLED
  handleWiFi();
#endif

  // Check for inactivity timeout -> enter deep sleep
  if (millis() - gLastActivityTime >= INACTIVITY_TIMEOUT) {
    enterDeepSleep();  // Never returns
  }

  // Monitor battery voltage (affects LED status)
  checkBatteryVoltage();

  // Update status LED (shows connection state and battery warning)
  handleStatusLed();

  // Handle BLE connection and reconnection
  if (myHub.isConnecting()) {
    myHub.connectHub();
    if (myHub.isConnected()) {
      char hubName[] = "DavidTrainHub";
      debugLog("Setting hub name to DavidTrainHub");
      ensureCommandInterval();
      myHub.setHubName(hubName);
      debugLog("Connected to HUB");
      debugLog("Hub address: %s", myHub.getHubAddress().toString().c_str());
      debugLog("Hub name: %s", myHub.getHubName().c_str());
      // Activate speaker port for sound playback
      delay(200);
      ensureCommandInterval();
      myHub.activateBaseSpeaker();
      debugLog("Speaker activated");

      gWasConnected = true;
      gSpeed = 0;  // Reset speed on new connection
      gLastActivityTime = millis();  // Reset inactivity timer on new connection
    } else {
      debugLog("Failed to connect to HUB");
    }
  }

  // Restart BLE scan when not connected and scan has finished
  if (!myHub.isConnected() && !myHub.isConnecting()) {
      if (gWasConnected) {
        debugLog("Connection lost - restarting BLE scan...");
        gWasConnected = false;
        gSpeed = 0;
      }
      myHub.init();
  }

  if (myHub.isConnected()) {
      handleButtons();
      handlePoti();
  }

  // Track button activity when disconnected (for deep sleep timeout)
  if (!myHub.isConnected()) {
    pbMusic.update();
    pbLight.update();
    pbWater.update();
    pbStop.update();
    if (pbMusic.fell() || pbLight.fell() || pbWater.fell() || pbStop.fell()) {
      gLastActivityTime = millis();
    }
  }

  delay(20); //small delay to avoid busy looping
} 