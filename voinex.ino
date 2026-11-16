#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// ---------------- Pin assignments ----------------
const uint8_t CURRENT_SENSOR_PIN = A2;
const uint8_t VOLTAGE_SENSOR_PIN = A3;
const uint8_t POT_PIN            = A0;

const uint8_t RELAY_PIN      = 4;
const uint8_t GREEN_LED_PIN  = 6;
const uint8_t RED_LED_PIN    = 7;

// ---------------- LCD and ADS1115 ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_ADS1115 ads;

// ---------------- Calibration settings ----------------
// Arduino Mega ADC: 10-bit, 0 to 5 V
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float ADC_COUNTS = 1023.0;

// ACS712 5 A version sensitivity:
// 185 mV/A = 0.185 V/A
const float ACS712_SENSITIVITY = 0.185;

// Adjust this after testing with a known current.
const float CURRENT_CALIBRATION = 1.00;

// ZMPT101B scale factor.
// This value must be calibrated using a known AC mains voltage.
const float ZMPT_CALIBRATION = 100.0;

// Protection threshold
const float DEFAULT_CURRENT_LIMIT = 2.00;

// Set this to false if your relay turns ON with HIGH.
const bool RELAY_ACTIVE_LOW = true;

// ---------------- Measurement settings ----------------
const uint16_t SAMPLE_COUNT = 300;
const uint16_t SAMPLE_DELAY_US = 250;

// ---------------- Global values ----------------
float currentAmps = 0.0;
float voltageRMS = 0.0;
float powerWatts = 0.0;
float currentLimit = DEFAULT_CURRENT_LIMIT;

bool overload = false;

// -----------------------------------------------------
// Relay control
// -----------------------------------------------------
void setRelay(bool turnOn) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PIN, turnOn ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PIN, turnOn ? HIGH : LOW);
  }
}

// -----------------------------------------------------
// Read ACS712 RMS current
// -----------------------------------------------------
float readCurrentRMS() {
  float offset = 0.0;

  // First determine the ACS712 zero-current midpoint.
  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    offset += analogRead(CURRENT_SENSOR_PIN);
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  offset /= SAMPLE_COUNT;

  float sumSquares = 0.0;

  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    float sample = analogRead(CURRENT_SENSOR_PIN);
    float deviation = sample - offset;

    sumSquares += deviation * deviation;
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  float rmsCounts = sqrt(sumSquares / SAMPLE_COUNT);
  float rmsVoltage = rmsCounts * ADC_REFERENCE_VOLTAGE / ADC_COUNTS;

  float measuredCurrent =
    (rmsVoltage / ACS712_SENSITIVITY) * CURRENT_CALIBRATION;

  // Remove very small noise readings.
  if (measuredCurrent < 0.05) {
    measuredCurrent = 0.0;
  }

  return measuredCurrent;
}

// -----------------------------------------------------
// Read ZMPT101B RMS voltage
// -----------------------------------------------------
float readVoltageRMS() {
  float offset = 0.0;

  // Determine the ZMPT101B output midpoint.
  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    offset += analogRead(VOLTAGE_SENSOR_PIN);
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  offset /= SAMPLE_COUNT;

  float sumSquares = 0.0;

  for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
    float sample = analogRead(VOLTAGE_SENSOR_PIN);
    float deviation = sample - offset;

    sumSquares += deviation * deviation;
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  float rmsCounts = sqrt(sumSquares / SAMPLE_COUNT);
  float rmsVoltageAtArduino =
    rmsCounts * ADC_REFERENCE_VOLTAGE / ADC_COUNTS;

  float measuredVoltage = rmsVoltageAtArduino * ZMPT_CALIBRATION;

  if (measuredVoltage < 1.0) {
    measuredVoltage = 0.0;
  }

  return measuredVoltage;
}

// -----------------------------------------------------
// Read potentiometer current limit
// -----------------------------------------------------
float readCurrentLimit() {
  int potValue = analogRead(POT_PIN);

  // Potentiometer sets limit from 0.5 A to 5.0 A.
  float limit = 0.5 + (potValue / ADC_COUNTS) * 4.5;

  return limit;
}

// -----------------------------------------------------
// Update LCD
// -----------------------------------------------------
void updateLCD() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(voltageRMS, 1);
  lcd.print(" I:");
  lcd.print(currentAmps, 2);

  lcd.setCursor(0, 1);

  if (overload) {
    lcd.print("OVERLOAD");
  } else {
    lcd.print("P:");
    lcd.print(powerWatts, 0);
    lcd.print("W L:");
    lcd.print(currentLimit, 1);
  }
}

// -----------------------------------------------------
// Setup
// -----------------------------------------------------
void setup() {
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(VOLTAGE_SENSOR_PIN, INPUT);
  pinMode(POT_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  setRelay(false);

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Energy Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  bool adsAvailable = ads.begin(0x48);

  Serial.print("ADS1115 Status: ");
  Serial.println(adsAvailable ? "CONNECTED" : "NOT DETECTED");

  Serial.println("System: Energy monitor started");
  Serial.println("Note: ACS712 and ZMPT101B are read through Mega analog pins");

  delay(1500);
}

// -----------------------------------------------------
// Main loop
// -----------------------------------------------------
void loop() {
  currentLimit = readCurrentLimit();

  currentAmps = readCurrentRMS();
  voltageRMS = readVoltageRMS();

  powerWatts = voltageRMS * currentAmps;

  overload = currentAmps > currentLimit;

  if (overload) {
    setRelay(false);

    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  } else {
    setRelay(true);

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }

  updateLCD();

  // Each reading uses one complete line for serial-monitor parsing.
  Serial.print("Voltage RMS: ");
  Serial.println(voltageRMS, 2);

  Serial.print("Current RMS: ");
  Serial.println(currentAmps, 3);

  Serial.print("Power: ");
  Serial.println(powerWatts, 2);

  Serial.print("Current Limit: ");
  Serial.println(currentLimit, 2);

  Serial.print("Relay Status: ");
  Serial.println(overload ? "OFF - OVERLOAD" : "ON - NORMAL");

  Serial.print("System Status: ");
  Serial.println(overload ? "FAULT" : "NORMAL");

  Serial.println();

  delay(500);
}
