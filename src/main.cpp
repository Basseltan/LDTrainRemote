#include <Lpf2Hub.h>      //legoino
#include <Lpf2HubConst.h> //legoino
#include <Bounce2.h>      //bounce2
#include <movingAvg.h>    //movingAvg

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
// new MyHub class extends Lpf2Hub with helper functions for our specific use case (color sensor, sounds, LED control)
// taken from https://github.com/corneliusmunz/legoino/issues/44#issuecomment-985384328
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

// Refueling state (blue action brick): stop, play sound, resume after 5 seconds
bool gRefueling = false;
unsigned long gRefuelStart = 0;
int gRefuelResumeSpeed = 0;
const unsigned long REFUEL_DURATION = 5000;  // 5 seconds

// Color sensor detection: sliding window with majority vote and per-color thresholds.
// The Duplo Train color sensor is very noisy — on normal track it reads green (~40%),
// blue (~34%), yellow (~23%) with occasional BLACK. RED and WHITE rarely appear in noise.
// Per-color thresholds prevent false triggers from noise while allowing distinctive colors.
const int COLOR_WINDOW_SIZE = 16;      // Circular buffer size (smaller = faster response)
int gColorWindow[COLOR_WINDOW_SIZE];   // Circular buffer of non-BLACK color readings
int gColorWindowIdx = 0;              // Next write position
int gColorWindowCount = 0;            // How many entries filled (0..COLOR_WINDOW_SIZE)
unsigned long gLastColorAction = 0;
const unsigned long COLOR_COOLDOWN = 3000;  // Ignore sensor for 3 seconds after action
int gBlackCount = 0;                   // Consecutive BLACK readings (for diagnostics)
unsigned long gStationarySince = 0;    // When speed last became 0 (0 = currently moving)
const unsigned long STATIONARY_SUPPRESS_DELAY = 5000;  // Suppress color after 5s stationary

// Per-color trigger thresholds (out of COLOR_WINDOW_SIZE readings in window)
// Noise while moving: green ~35%, blue ~33%, yellow ~30%, red ~0%, white ~1%
// Noise while stopped: green ~0%, blue ~60%, yellow ~40% (different profile!)
// Distinctive colors (RED/WHITE) are checked independently — they don't need to be dominant.
const int THRESHOLD_RED = 1;           // 6% — never appears in noise, may get only 1 reading at high speed
const int THRESHOLD_WHITE = 4;         // 25% — appears in noise ~5% near colored bricks, need higher bar
const int THRESHOLD_NOISE = 12;        // 75% — green/blue/yellow need strong dominance over noise

// LED status indicator state (now using LED_LOW_BAT for both status and battery warning)
unsigned long gLedLastToggle = 0;
bool gLedState = false;

// Command rate limiting to prevent hub overload
unsigned long gLastCommandTime = 0;
const unsigned long COMMAND_MIN_INTERVAL = 150;  // Minimum 150ms between commands

// Battery monitoring (18650 Li-Ion with voltage divider 2x 220kOhm)
const float BAT_VOLTAGE_MIN = 3.0;        // Cutoff voltage: 3.0V
const float BAT_VOLTAGE_DIVIDER = 2.0;    // Voltage divider ratio
const float ADC_REFERENCE = 3.3;          // ESP32 ADC reference voltage
const int ADC_MAX = 4095;                 // 12-bit ADC
const unsigned long BAT_CHECK_INTERVAL = 5000;  // Check every 5 seconds
unsigned long gLastBatCheck = 0;
bool gBatteryLow = false;

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

Color getNextColor()
{
    gColor++;
    if (gColor >= NUM_COLORS)
    {
        gColor = 0;
    }
    return (Color)gColor;
}

// Callback for color sensor notifications from the Duplo Train hub.
// Called automatically by the BLE notification handler when a color stone is detected.
void colorSensorCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData)
{
  Lpf2Hub *myHubRef = (Lpf2Hub *)hub;

  if (deviceType == DeviceType::DUPLO_TRAIN_BASE_COLOR_SENSOR)
  {
    int color = myHubRef->parseColor(pData);

    // Stationary noise suppression: when stopped, the noise profile shifts to ~60% blue
    // / ~40% yellow, which can cause false triggers. Clear window on transition to
    // speed 0 so noise can't build on old readings, then suppress after 5 seconds.
    if (gSpeed == 0)
    {
      if (gStationarySince == 0) {
        gStationarySince = millis();
        // Clear window on transition to stopped
        gColorWindowIdx = 0;
        gColorWindowCount = 0;
      }
      if (millis() - gStationarySince > STATIONARY_SUPPRESS_DELAY) {
        return;
      }
    } else {
      gStationarySince = 0;  // Reset when moving
    }

    // Don't process color sensor when train is intentionally stopped (RED brick
    // or STOP button) or during refueling — prevents re-triggers on same brick
    if (gStopLatch || gRefueling)
    {
      return;
    }

    // Track BLACK readings for diagnostics, don't add to window
    if (color == (byte)BLACK || color == (byte)NONE)
    {
      gBlackCount++;
      return;
    }
    gBlackCount = 0;  // Reset on any non-BLACK reading

    Serial.print("COLOR: ");
    Serial.print(COLOR_STRING[color]);

    // Cooldown: ignore sensor readings for 3 seconds after last action
    if (millis() - gLastColorAction < COLOR_COOLDOWN)
    {
      Serial.println(" (cooldown)");
      return;
    }

    // Add to sliding window (circular buffer)
    gColorWindow[gColorWindowIdx] = color;
    gColorWindowIdx = (gColorWindowIdx + 1) % COLOR_WINDOW_SIZE;
    if (gColorWindowCount < COLOR_WINDOW_SIZE) gColorWindowCount++;

    // Count occurrences of each color in the window
    int counts[NUM_COLORS] = {0};
    for (int i = 0; i < gColorWindowCount; i++) {
      int c = gColorWindow[i];
      if (c >= 0 && c < NUM_COLORS) counts[c]++;
    }

    // Find the dominant color (for noise color check)
    int bestColor = -1;
    int bestCount = 0;
    for (int i = 1; i < NUM_COLORS; i++) {  // Skip BLACK (0)
      if (counts[i] > bestCount) {
        bestCount = counts[i];
        bestColor = i;
      }
    }

    // Check each color against its individual threshold.
    // Distinctive colors (RED/WHITE) are checked first — they don't need to be
    // the dominant color to trigger, since they never appear in noise.
    // Noise colors (GREEN/BLUE/YELLOW) only trigger if dominant at high threshold.
    int triggeredColor = -1;
    int triggeredCount = 0;
    int triggeredThreshold = 0;

    if (counts[(byte)RED] >= THRESHOLD_RED) {
      triggeredColor = (byte)RED;
      triggeredCount = counts[(byte)RED];
      triggeredThreshold = THRESHOLD_RED;
    } else if (counts[(byte)WHITE] >= THRESHOLD_WHITE) {
      triggeredColor = (byte)WHITE;
      triggeredCount = counts[(byte)WHITE];
      triggeredThreshold = THRESHOLD_WHITE;
    } else if (bestCount >= THRESHOLD_NOISE) {
      triggeredColor = bestColor;
      triggeredCount = bestCount;
      triggeredThreshold = THRESHOLD_NOISE;
    }

    Serial.print(" window=[");
    for (int i = 1; i < NUM_COLORS; i++) {
      if (counts[i] > 0) {
        Serial.print(COLOR_STRING[i]);
        Serial.print(":");
        Serial.print(counts[i]);
        Serial.print(" ");
      }
    }
    Serial.print("] best=");
    Serial.print(bestColor >= 0 ? COLOR_STRING[bestColor] : "?");
    Serial.print("(");
    Serial.print(bestCount);
    Serial.print("/");
    Serial.print(THRESHOLD_NOISE);
    Serial.print(")");
    // Show distinctive color counts if present
    if (counts[(byte)RED] > 0) {
      Serial.print(" R:");
      Serial.print(counts[(byte)RED]);
      Serial.print("/");
      Serial.print(THRESHOLD_RED);
    }
    if (counts[(byte)WHITE] > 0) {
      Serial.print(" W:");
      Serial.print(counts[(byte)WHITE]);
      Serial.print("/");
      Serial.print(THRESHOLD_WHITE);
    }
    Serial.println();

    // No color reached its threshold
    if (triggeredColor < 0)
    {
      return;
    }

    // Triggered! Clear window and start cooldown
    gColorWindowIdx = 0;
    gColorWindowCount = 0;
    gLastColorAction = millis();
    color = triggeredColor;  // Use the triggered color, not the last reading

    Serial.print(">>> Action brick triggered: ");
    Serial.println(COLOR_STRING[color]);

    // Set hub LED to detected color
    ensureCommandInterval();
    myHubRef->setLedColor((Color)color);

    // Execute action based on action brick color
    if (color == (byte)RED)
    {
      // Red: Stop - train stops completely
      gSpeed = 0;
      gStopLatch = true;
      ensureCommandInterval();
      myHubRef->setBasicMotorSpeed(port, 0);
      ensureCommandInterval();
      myHubRef->playSound((byte)DuploTrainBaseSound::BRAKE);
    }
    else if (color == (byte)BLUE)
    {
      // Blue: Refueling - stop, tank sound, resume after 5 seconds
      gRefuelResumeSpeed = gSpeed;
      gSpeed = 0;
      ensureCommandInterval();
      myHubRef->setBasicMotorSpeed(port, 0);
      ensureCommandInterval();
      myHubRef->playSound((byte)DuploTrainBaseSound::WATER_REFILL);
      gRefueling = true;
      gRefuelStart = millis();
    }
    else if (color == (byte)YELLOW)
    {
      // Yellow: Signal - horn sound only
      ensureCommandInterval();
      myHubRef->playSound((byte)DuploTrainBaseSound::HORN);
    }
    else if (color == (byte)GREEN)
    {
      // Green: Direction change - stop and reverse
      gSpeed = -gSpeed;
      ensureCommandInterval();
      myHubRef->setBasicMotorSpeed(port, 0);
      ensureCommandInterval();
      myHubRef->playSound((byte)DuploTrainBaseSound::STATION_DEPARTURE);
      ensureCommandInterval();
      myHubRef->setBasicMotorSpeed(port, gSpeed);
    }
    else if (color == (byte)WHITE)
    {
      // White: Light change - cycle hub LED color
      Color c = getNextColor();
      ensureCommandInterval();
      myHubRef->setLedColor(c);
    }
  }
}

void handlePoti()
{
  int ptiSpeed = avgPotiSpeed.reading(analogRead(PTI_SPEED));
  gLastStatePtiSpeed = ptiSpeed;

  // Potentiometer calibration - adjust these values to match your hardware
  // Calculated for pot that currently gives -45 to +45 range (to extend to -100 to +100)
  const int POT_MIN = 1126;   // ADC value at minimum position (~27% of 4095)
  const int POT_MAX = 2969;   // ADC value at maximum position (~73% of 4095)

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
      Serial.println("POT: returned to center - accepting speed again");
    }
    return;
  }

  if (speed != gSpeed)
  {
    Serial.print("POT: speed change ");
    Serial.print(gSpeed);
    Serial.print(" -> ");
    Serial.println(speed);

    if (gSpeed == 0 && speed > 0)
    {
      Serial.println("POT: transitioning 0->forward: play STATION_DEPARTURE");
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

  // Debug output (commented out to reduce serial noise during color sensor debugging)
  // Serial.print("Battery: ADC=");
  // Serial.print(adcValue);
  // Serial.print(", Pin=");
  // Serial.print(pinVoltage, 2);
  // Serial.print("V, Bat=");
  // Serial.print(batteryVoltage, 2);
  // Serial.println("V");

  // Check if battery is below cutoff voltage
  if (batteryVoltage < BAT_VOLTAGE_MIN) {
    if (!gBatteryLow) {
      Serial.println("WARNING: Battery voltage low!");
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
      Serial.println("Button MUSIC pressed - playing HORN");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::HORN);
    }
  }
  if(pbLight.update())
  {
    if(pbLight.fell())
    {
      Color c = getNextColor();
      Serial.print("Button LIGHT pressed - setting LED color to "); Serial.println((int)c);
      ensureCommandInterval();
      myHub.setLedColor(c);
    }
  }
  if(pbWater.update())
  {
    if(pbWater.fell())
    {
      Serial.println("Button WATER pressed - playing WATER_REFILL");
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::WATER_REFILL);
    }
  }
  if(pbStop.update())
  {
    if(pbStop.fell())
    {
      Serial.println("Button STOP pressed - stopping motor and playing BRAKE");
      gSpeed = 0;
      gStopLatch = true;  // Block poti until it returns to center
      ensureCommandInterval();
      myHub.setBasicMotorSpeed(port, 0);
      ensureCommandInterval();
      myHub.playSound((byte)DuploTrainBaseSound::BRAKE);
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
  Serial.println("myHub.init() called - BLE scan started");
}

void loop() {
  // Monitor battery voltage (affects LED status)
  checkBatteryVoltage();

  // Update status LED (shows connection state and battery warning)
  handleStatusLed();

  // Handle BLE connection and reconnection
  if (myHub.isConnecting()) {
    myHub.connectHub();
    if (myHub.isConnected()) {
      char hubName[] = "DavidTrainHub";
      Serial.println("Setting hub name to DavidTrainHub");
      ensureCommandInterval();
      myHub.setHubName(hubName);
      Serial.println("Connected to HUB");
      Serial.print("Hub address: ");
      Serial.println(myHub.getHubAddress().toString().c_str());
      Serial.print("Hub name: ");
      Serial.println(myHub.getHubName().c_str());
      // Activate color sensor to detect action bricks on the track
      delay(200);  // Allow hub to initialize
      myHub.activatePortDevice((byte)DuploTrainHubPort::COLOR, colorSensorCallback);
      Serial.println("Color sensor activated");

      gWasConnected = true;
      gSpeed = 0;  // Reset speed on new connection
    } else {
      Serial.println("Failed to connect to HUB");
    }
  }

  // Detect disconnection and trigger rescan
  if (gWasConnected && !myHub.isConnected() && !myHub.isConnecting()) {
      Serial.println("Connection lost - restarting BLE scan...");
      gWasConnected = false;
      gSpeed = 0;
      myHub.init();
  }

  if (myHub.isConnected()) {
      // Handle refueling timer (blue action brick): resume after 5 seconds
      if (gRefueling && (millis() - gRefuelStart >= REFUEL_DURATION)) {
        gRefueling = false;
        gSpeed = gRefuelResumeSpeed;
        Serial.print("Refueling complete - resuming speed: ");
        Serial.println(gSpeed);
        ensureCommandInterval();
        myHub.playSound((byte)DuploTrainBaseSound::STATION_DEPARTURE);
        ensureCommandInterval();
        myHub.setBasicMotorSpeed(port, gSpeed);
      }

      handleButtons();
      if (!gRefueling) {
        handlePoti();
      }
  }
  delay(20); //small delay to avoid busy looping
} 