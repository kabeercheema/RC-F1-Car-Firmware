
---

# Arduino Uno Firmware (`RC_F1_Firmware.ino`)

/*
  RC_F1_Firmware.ino
  ----------------------------------------------------------------------
  1/10-scale F1 car control firmware (Arduino Uno)

  Hardware
  - Arduino Uno (ATmega328P @ 16 MHz)
  - RC receiver PWM: CH2 Throttle -> D2 (INT0), CH1 Steering -> D3 (INT1)
  - Outputs: ESC -> D10, Steering servo -> D9  (Servo lib @ ~50 Hz)
  - Arming button -> D7 (to GND, INPUT_PULLUP)
  - Status LED -> D13 (built-in)
  - Optional VBAT divider -> A0

  Features
  - RC PWM capture on hardware interrupts
  - Calibration (min/mid/max) to EEPROM
  - Deadband, expo, EMA smoothing, slew-rate limiting
  - DISARMED / ARMED / FAULT states
  - ESC neutral hold on boot for arming
  - Serial diagnostics (115200)
*/

#include <Servo.h>
#include <EEPROM.h>
#include <math.h>

// Compile-time configuration

// Pins (Arduino Uno)
#define PIN_IN_THROTTLE    2    // INT0
#define PIN_IN_STEERING    3    // INT1
#define PIN_OUT_ESC        10   // Servo-capable
#define PIN_OUT_STEER      9    // Servo-capable
#define PIN_BTN_ARM        7    // Active-low pushbutton to GND
#define PIN_STATUS_LED     13   // Built-in LED
#define PIN_VBAT           A0   // Optional battery monitor

// RC pulse expectations
#define RC_PULSE_MIN_US        1000
#define RC_PULSE_MAX_US        2000
#define RC_PULSE_NEUTRAL_US    1500
#define RC_SIGNAL_TIMEOUT_US   100000UL  // 100 ms -> FAULT on loss

// Output pulse bounds
#define ESC_MIN_US          1000
#define ESC_MAX_US          2000
#define ESC_NEUTRAL_US      1500
#define STEER_MIN_US        1100
#define STEER_MAX_US        1900
#define STEER_CENTER_US     1500

// Filtering / shaping
#define EMA_ALPHA           0.25f   // Exponential moving average weight
#define DEADBAND_NORM       0.04f   // deadband in normalized units (-1..1)
#define EXPO                0.25f   // 0..1 (higher = more center softness)
#define SLEW_PER_SEC        3.0f    // max change per second in normalized units

// Battery (optional)
#define VBAT_R1             100000.0f  // top resistor (ohms)
#define VBAT_R2             10000.0f   // bottom resistor (ohms)
#define VBAT_ADC_REF        5.0f       // Uno default analog reference
#define VBAT_ADC_BITS       1023.0f
#define VBAT_LOW_WARN       6.6f       // 2S @ 3.3 V/cell
#define VBAT_SAMPLE_MS      250

// Arming /timing
#define ESC_NEUTRAL_ON_BOOT_MS  2000
#define BTN_DEBOUNCE_MS           40
#define LONG_PRESS_MS           1500   // hold on boot -> calibration
#define LOOP_DT_TARGET_MS         10   // 100 Hz loop

// EEPROM layout
#define EEPROM_ADDR_CAL     0
#define CAL_VERSION         1

// Types 
enum RunState : uint8_t { DISARMED = 0, ARMED = 1, FAULT = 2 };

struct CalData {
  uint16_t thrMin, thrMid, thrMax;
  uint16_t strMin, strMid, strMax;
  uint8_t  version;
  uint8_t  checksum; // simple 8-bit sum (not included when computed)
};

// Globals
Servo esc, steer;

// RC input capture (ISR-updated)
volatile uint32_t thrRiseUs = 0, strRiseUs = 0;
volatile uint16_t thrPulseUs = RC_PULSE_NEUTRAL_US;
volatile uint16_t strPulseUs = RC_PULSE_NEUTRAL_US;
volatile uint32_t thrLastUpdateUs = 0;
volatile uint32_t strLastUpdateUs = 0;

CalData g_cal;
RunState g_state = DISARMED;

bool g_btnPrev = true; // pullup: true=not pressed
uint32_t g_btnLastMs = 0;

float g_thrOut = 0.0f;  // final shaped/limited normalized outputs
float g_strOut = 0.0f;

uint32_t g_lastLoopUs = 0;
uint32_t g_lastVBatMs = 0;

// Utilty
uint8_t calcChecksum(const CalData &c) {
  const uint8_t *p = (const uint8_t*)&c;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(CalData) - 1; ++i) sum += p[i];
  return sum;
}
void saveCal(const CalData &c) { EEPROM.put(EEPROM_ADDR_CAL, c); }
bool loadCal(CalData &out) {
  EEPROM.get(EEPROM_ADDR_CAL, out);
  if (out.version != CAL_VERSION) return false;
  if (calcChecksum(out) != out.checksum) return false;
  if (out.thrMin < 800 || out.thrMax > 2200) return false;
  if (out.strMin < 800 || out.strMax > 2200) return false;
  return true;
}
void setDefaultCal(CalData &c) {
  c.thrMin = 1000; c.thrMid = 1500; c.thrMax = 2000;
  c.strMin = 1100; c.strMid = 1500; c.strMax = 1900;
  c.version = CAL_VERSION;
  c.checksum = calcChecksum(c);
}

float mapPulseToNorm(uint16_t us, uint16_t minUs, uint16_t midUs, uint16_t maxUs) {
  if (us >= midUs) {
    float span = (float)(maxUs - midUs); if (span < 1) span = 1;
    return (float)(us - midUs) / span; // 0..1
  } else {
    float span = (float)(midUs - minUs); if (span < 1) span = 1;
    return - (float)(midUs - us) / span; // -1..0
  }
}
uint16_t mapNormToPulse(float x, uint16_t minUs, uint16_t midUs, uint16_t maxUs) {
  x = constrain(x, -1.0f, 1.0f);
  if (x >= 0.0f) return (uint16_t)(midUs + x * (float)(maxUs - midUs));
  else           return (uint16_t)(midUs + x * (float)(midUs - minUs)); // x negative
}
float applyDeadband(float x, float db) {
  if (fabs(x) <= db) return 0.0f;
  float sign = (x > 0) ? 1.0f : -1.0f;
  float mag = (fabs(x) - db) / (1.0f - db);
  return sign * constrain(mag, 0.0f, 1.0f);
}
float applyExpo(float x, float expo) { return x * (1.0f - expo) + (x * x * x) * expo; }
float slewLimit(float target, float current, float maxPerSec, float dtSec) {
  float maxStep = maxPerSec * dtSec;
  float delta = target - current;
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;
  return current + delta;
}
float readVBat() {
  int raw = analogRead(PIN_VBAT);
  float vOut = (raw * VBAT_ADC_REF) / VBAT_ADC_BITS;
  float vin = vOut * ((VBAT_R1 + VBAT_R2) / VBAT_R2);
  return vin;
}

// ISRs: RC receiver capture
void ISR_ThrChange() {
  uint8_t level = digitalRead(PIN_IN_THROTTLE);
  uint32_t now = micros();
  if (level) { thrRiseUs = now; }
  else {
    uint32_t width = now - thrRiseUs;
    if (width >= 800 && width <= 2200) { thrPulseUs = (uint16_t)width; thrLastUpdateUs = now; }
  }
}
void ISR_StrChange() {
  uint8_t level = digitalRead(PIN_IN_STEERING);
  uint32_t now = micros();
  if (level) { strRiseUs = now; }
  else {
    uint32_t width = now - strRiseUs;
    if (width >= 800 && width <= 2200) { strPulseUs = (uint16_t)width; strLastUpdateUs = now; }
  }
}

// Button (debounced edg)
bool buttonPressed() {
  bool level = digitalRead(PIN_BTN_ARM); // HIGH = not pressed, LOW = pressed
  uint32_t now = millis();
  if (now - g_btnLastMs < BTN_DEBOUNCE_MS) return false;
  static bool prev = HIGH;
  if (level != prev) {
    prev = level; g_btnLastMs = now;
    if (level == LOW) return true;
  }
  return false;
}

// LED pattern
void setLedPattern(RunState s) {
  static uint32_t last = 0; static bool on = false;
  uint32_t now = millis();
  uint32_t interval = 500; // DISARMED slow
  if (s == FAULT) interval = 120;
  if (s == ARMED) { digitalWrite(PIN_STATUS_LED, HIGH); return; }
  if (now - last >= interval) { on = !on; digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW); last = now; }
}

// Calibration
void runCalibration() {
  Serial.println(F("\n=== RC Calibration Mode ==="));
  Serial.println(F("Move throttle/steering through full range, then press ARM button to save."));
  uint16_t thrMin = 2000, thrMax = 1000, thrMid = RC_PULSE_NEUTRAL_US;
  uint16_t strMin = 2000, strMax = 1000, strMid = STEER_CENTER_US;

  uint32_t start = millis();
  while (true) {
    noInterrupts();
    uint16_t t = thrPulseUs, s = strPulseUs;
    uint32_t tu = thrLastUpdateUs, su = strLastUpdateUs;
    interrupts();

    uint32_t nowUs = micros();
    bool tOk = (nowUs - tu) < RC_SIGNAL_TIMEOUT_US;
    bool sOk = (nowUs - su) < RC_SIGNAL_TIMEOUT_US;

    if (tOk) { if (t < thrMin) thrMin = t; if (t > thrMax) thrMax = t; thrMid = t; }
    if (sOk) { if (s < strMin) strMin = s; if (s > strMax) strMax = s; strMid = s; }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 250) {
      Serial.print(F("THR min/mid/max = ")); Serial.print(thrMin); Serial.print('/');
      Serial.print(thrMid); Serial.print('/'); Serial.print(thrMax);
      Serial.print(F("   STR min/mid/max = ")); Serial.print(strMin); Serial.print('/');
      Serial.print(strMid); Serial.print('/'); Serial.println(strMax);
      lastPrint = millis();
    }

    if (buttonPressed()) {
      CalData c{thrMin, thrMid, thrMax, strMin, strMid, strMax, CAL_VERSION, 0};
      c.checksum = calcChecksum(c);
      saveCal(c); g_cal = c;
      Serial.println(F("Saved calibration to EEPROM. Exiting."));
      delay(500); return;
    }

    if (millis() - start > 120000UL) { // timeout safety
      CalData c{thrMin, thrMid, thrMax, strMin, strMid, strMax, CAL_VERSION, 0};
      c.checksum = calcChecksum(c);
      saveCal(c); g_cal = c;
      Serial.println(F("Calibration timeout; saved current extents.")); return;
    }

    setLedPattern(DISARMED);
    delay(10);
  }
}

// Setup / Loop
void setup() {
  pinMode(PIN_BTN_ARM, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.begin(115200);
  Serial.println(F("\nRC F1 Firmware (UNO) booting..."));

  if (!loadCal(g_cal)) { Serial.println(F("No valid calibration; using defaults.")); setDefaultCal(g_cal); }
  else                 { Serial.println(F("Calibration loaded from EEPROM.")); }

  // Long-press on boot - calibration
  uint32_t bootStart = millis();
  while (millis() - bootStart < LONG_PRESS_MS) {
    if (digitalRead(PIN_BTN_ARM) == LOW) { runCalibration(); break; }
    setLedPattern(DISARMED);
  }

  // Inputs
  pinMode(PIN_IN_THROTTLE, INPUT);
  pinMode(PIN_IN_STEERING, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_THROTTLE), ISR_ThrChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_STEERING), ISR_StrChange, CHANGE);

  // Outputs
  esc.attach(PIN_OUT_ESC);
  steer.attach(PIN_OUT_STEER);

  // ESC neutral hold on boot
  esc.writeMicroseconds(ESC_NEUTRAL_US);
  steer.writeMicroseconds(STEER_CENTER_US);
  Serial.println(F("Holding ESC neutral for arming..."));
  uint32_t t0 = millis();
  while (millis() - t0 < ESC_NEUTRAL_ON_BOOT_MS) { setLedPattern(DISARMED); delay(10); }

  g_lastLoopUs = micros();
  Serial.println(F("Ready."));
}

void loop() {
  // Timing
  uint32_t nowUs = micros();
  float dt = (nowUs - g_lastLoopUs) / 1e6f; if (dt <= 0) dt = 0.001f;
  g_lastLoopUs = nowUs;

  // Copy RC inputs (atomic)
  uint16_t thrUs, strUs; uint32_t thrUpd, strUpd;
  noInterrupts();
  thrUs = thrPulseUs; strUs = strPulseUs; thrUpd = thrLastUpdateUs; strUpd = strLastUpdateUs;
  interrupts();

  bool thrOk = (nowUs - thrUpd) < RC_SIGNAL_TIMEOUT_US;
  bool strOk = (nowUs - strUpd) < RC_SIGNAL_TIMEOUT_US;
  bool signalOk = thrOk && strOk;

  // Button: toggle arm when safe
  if (buttonPressed()) {
    if (g_state == DISARMED && signalOk) {
      float thrNormTmp = mapPulseToNorm(thrUs, g_cal.thrMin, g_cal.thrMid, g_cal.thrMax);
      if (fabs(thrNormTmp) < 0.05f) { g_state = ARMED; Serial.println(F("STATE: ARMED")); }
      else                          { Serial.println(F("Refusing to ARM: throttle not neutral.")); }
    } else if (g_state == ARMED) {
      g_state = DISARMED; Serial.println(F("STATE: DISARMED"));
    } else if (g_state == FAULT) {
      if (signalOk) {
        float thrNormTmp = mapPulseToNorm(thrUs, g_cal.thrMin, g_cal.thrMid, g_cal.thrMax);
        if (fabs(thrNormTmp) < 0.05f) { g_state = DISARMED; Serial.println(F("FAULT cleared -> DISARMED")); }
      }
    }
  }

  // Safety stuff - signal loss -> FAULT
  if (!signalOk && g_state == ARMED) { g_state = FAULT; Serial.println(F("STATE: FAULT (signal timeout)")); }

  // Normalize & shape
  float thrNorm = mapPulseToNorm(thrUs, g_cal.thrMin, g_cal.thrMid, g_cal.thrMax);
  float strNorm = mapPulseToNorm(strUs, g_cal.strMin, g_cal.strMid, g_cal.strMax);

  thrNorm = applyDeadband(thrNorm, DEADBAND_NORM);
  strNorm = applyDeadband(strNorm, DEADBAND_NORM);
  thrNorm = applyExpo(thrNorm, EXPO);
  strNorm = applyExpo(strNorm, EXPO);

  // EMA toward target, then slew-limit toward tha EMA
  float thrEma = (1.0f - EMA_ALPHA) * g_thrOut + EMA_ALPHA * thrNorm;
  float strEma = (1.0f - EMA_ALPHA) * g_strOut + EMA_ALPHA * strNorm;
  g_thrOut = slewLimit(thrEma, g_thrOut, SLEW_PER_SEC, dt);
  g_strOut = slewLimit(strEma, g_strOut, SLEW_PER_SEC * 1.2f, dt);

  // Decide outputs based on state
  uint16_t escUs = ESC_NEUTRAL_US;
  uint16_t steerUs = STEER_CENTER_US;

  switch (g_state) {
    case DISARMED:
      escUs = ESC_NEUTRAL_US; steerUs = STEER_CENTER_US; break;
    case ARMED:
      escUs = mapNormToPulse(g_thrOut, ESC_MIN_US, ESC_NEUTRAL_US, ESC_MAX_US);
      steerUs = mapNormToPulse(g_strOut, STEER_MIN_US, STEER_CENTER_US, STEER_MAX_US);
      break;
    case FAULT:
      escUs = ESC_NEUTRAL_US; steerUs = STEER_CENTER_US; break;
  }

  // Write outputs 
  esc.writeMicroseconds(escUs);
  steer.writeMicroseconds(steerUs);

  // LED pattern
  setLedPattern(g_state);

  // Battery diagnostics (typically not included, still working on it)
  if (millis() - g_lastVBatMs > VBAT_SAMPLE_MS) {
    g_lastVBatMs = millis();
    float vbat = readVBat();
    if (vbat > 0.1f) {
      if (vbat < VBAT_LOW_WARN && g_state == ARMED) {
        Serial.print(F("WARN: Low VBAT = ")); Serial.println(vbat, 2);
      }
    }
  }

  // Debug stuff
  /*
  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 200) {
    lastDbg = millis();
    Serial.print(F("STATE=")); Serial.print((int)g_state);
    Serial.print(F(" THR="));  Serial.print(thrUs);
    Serial.print(F(" STR="));  Serial.print(strUs);
    Serial.print(F(" thrN=")); Serial.print(g_thrOut, 2);
    Serial.print(F(" strN=")); Serial.print(g_strOut, 2);
    Serial.print(F(" esc="));  Serial.print(escUs);
    Serial.print(F(" steer="));Serial.println(steerUs);
  }
  */

  // Soft timing (~100 Hz)
  uint32_t used = micros() - nowUs;
  int32_t targetUs = LOOP_DT_TARGET_MS * 1000;
  if ((int32_t)used < targetUs) delayMicroseconds(targetUs - used);
}
