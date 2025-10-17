#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ======== LCD Setup ========
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ======== Pin Definitions ========
#define VOLTAGE_PIN 34
#define CURRENT_PIN 35
#define HALL_PIN 32
#define RELAY_PIN 33

// ======== ADC & Sensor Constants ========
#define ADC_RESOLUTION 4095.0
#define ADC_VOLTAGE 3.3
float VOLTAGE_MULTIPLIER = 1.0; // To be calibrated
float ACS_ZERO_VOLT = 1.65;     // To be calibrated
#define ACS_SENSITIVITY 0.100

// ======== Variables ========
float voltageRMS = 0.0;
float currentRMS = 0.0;
float power = 0.0;
float energy = 0.0;
bool theftDetected = false;
bool tamperDetected = false;

unsigned long previousMillis = 0;
const unsigned long interval = 1000; 

// ======== Simulation & Calibration Flags ========
bool simulateLoad = false;     // true = simulate 9W bulb
bool calibrationMode = false;  // true = run calibration helper once

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Smart Energy Meter");

  delay(1000);

  if (calibrationMode) {
    runCalibration();
    calibrationMode = false; // Only run once
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (simulateLoad) {
      voltageRMS = 230.0;
      currentRMS = 0.039; // 9W / 230V
    } else {
      int rawVoltage = analogRead(VOLTAGE_PIN);
      int rawCurrent = analogRead(CURRENT_PIN);

      // Voltage conversion
      float sensorVoltage = (rawVoltage / ADC_RESOLUTION) * ADC_VOLTAGE;
      voltageRMS = sensorVoltage * VOLTAGE_MULTIPLIER;

      // Current conversion (ACS712 20A)
      float currentVoltage = (rawCurrent / ADC_RESOLUTION) * ADC_VOLTAGE;
      currentRMS = (currentVoltage - ACS_ZERO_VOLT) / ACS_SENSITIVITY;
    }

    // Power & Energy
    power = voltageRMS * currentRMS;
    energy += power / 3600.0;

    // Tamper/Theft
    tamperDetected = digitalRead(HALL_PIN) == LOW;
    theftDetected = tamperDetected;
    digitalWrite(RELAY_PIN, theftDetected ? HIGH : LOW);

    // Serial output
    Serial.print("V:"); Serial.print(voltageRMS);
    Serial.print(" | I:"); Serial.print(currentRMS);
    Serial.print(" | P:"); Serial.print(power);
    Serial.print(" | E:"); Serial.print(energy,3);
    Serial.print(" | Theft:"); Serial.println(theftDetected ? "YES" : "NO");

    // LCD display
    lcd.setCursor(0,0); lcd.print("Voltage: "); lcd.print(voltageRMS,1); lcd.print(" V     ");
    lcd.setCursor(0,1); lcd.print("Current: "); lcd.print(currentRMS,3); lcd.print(" A     ");
    lcd.setCursor(0,2); lcd.print("Power: "); lcd.print(power,1); lcd.print(" W     ");
    lcd.setCursor(0,3); lcd.print("Energy: "); lcd.print(energy,3); lcd.print(" Wh ");
    lcd.print(theftDetected ? "THEFT!" : "OK    ");
  }
}

// ======== Calibration Helper ========
void runCalibration() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibration Mode");
  lcd.setCursor(0,1);
  lcd.print("Reading sensors...");

  delay(3000); // wait for stable readings

  int rawVoltage = analogRead(VOLTAGE_PIN);
  int rawCurrent = analogRead(CURRENT_PIN);

  // Voltage multiplier
  VOLTAGE_MULTIPLIER = 230.0 / ((rawVoltage / ADC_RESOLUTION) * ADC_VOLTAGE);
  ACS_ZERO_VOLT = (rawCurrent / ADC_RESOLUTION) * ADC_VOLTAGE;

  Serial.println("=== Calibration Complete ===");
  Serial.print("Voltage Multiplier: "); Serial.println(VOLTAGE_MULTIPLIER,6);
  Serial.print("ACS_ZERO_VOLT: "); Serial.println(ACS_ZERO_VOLT,3);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibration Done!");
  delay(2000);
  lcd.clear();
}
