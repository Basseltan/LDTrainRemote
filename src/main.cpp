#include <Lpf2Hub.h>      //legoino
#include <Lpf2HubConst.h> //legoino
#include <Bounce2.h>      //bounce2
#include <movingAvg.h>    //movingAvg
#include <stdarg.h>       //for debugLog variadic function

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
#define PTI_SPEED 12
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
const float ADC_REFERENCE = 3.3;          // ESP32 ADC reference voltage
const int ADC_MAX = 4095;                 // 12-bit ADC
const unsigned long BAT_CHECK_INTERVAL = 5000;  // Check every 5 seconds
unsigned long gLastBatCheck = 0;
bool gBatteryLow = false;

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
  unsigned long now = millis();

  // Check battery voltage periodically
  if (now - gLastBatCheck < BAT_CHECK_INTERVAL) {
    return;
  }

  gLastBatCheck = now;

  // Read ADC value and convert to actual battery voltage
  int adcValue = analogRead(BAT_VOLTAGE);
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
      debugLog("Button MUSIC pressed - playing HORN");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::HORN);
    }
  }
  if(pbLight.update())
  {
    if(pbLight.fell())
    {
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
      debugLog("Button WATER pressed - playing WATER_REFILL");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::WATER_REFILL);
    }
  }
  if(pbStop.update())
  {
    if(pbStop.fell())
    {
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
  Serial.begin(115200);
  while (!Serial) {}
  delay(200);

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

  myHub.init();
  debugLog("myHub.init() called - BLE scan started");

#ifdef WIFI_ENABLED
  setupWiFi();
#endif
}

void loop() {
#ifdef WIFI_ENABLED
  handleWiFi();
#endif

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
    } else {
      debugLog("Failed to connect to HUB");
    }
  }

  // Detect disconnection and trigger rescan
  if (gWasConnected && !myHub.isConnected() && !myHub.isConnecting()) {
      debugLog("Connection lost - restarting BLE scan...");
      gWasConnected = false;
      gSpeed = 0;
      myHub.init();
  }

  if (myHub.isConnected()) {
      handleButtons();
      handlePoti();
  }
  delay(20); //small delay to avoid busy looping
} 