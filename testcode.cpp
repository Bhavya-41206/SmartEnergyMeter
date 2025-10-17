/*
  Smart Energy Meter (ESP32)
  - 20x4 LCD
  - Voltage & Current (ACS712 20A) measurement
  - Power & Energy calculation
  - Theft/Tamper detection via Hall sensor
  - Relay control
  - ADC attenuation and calibration placeholders
  - Optional 9W bulb simulation
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ======== LCD Setup ========
LiquidCrystal_I2C lcd(0x27, 20, 4); // 20x4 LCD

// ======== Pin Definitions ========
#define VOLTAGE_PIN 34    // Voltage sensor analog pin
#define CURRENT_PIN 35    // ACS712 20A analog pin
#define HALL_PIN 32       // Hall sensor pin
#define RELAY_PIN 33      // Relay output

// ======== ADC & Sensor Constants ========
#define ADC_RESOLUTION 4095.0   // 12-bit ADC
#define ADC_VOLTAGE 3.3         // ESP32 ADC reference voltage
#define VOLTAGE_SENSOR_MAX 230.0 // Maximum AC voltage your sensor measures
#define ACS_SENSITIVITY 0.100   // V/A for 20A ACS712
float ACS_ZERO_VOLT = 1.65;     // Measured at no current; will calibrate

// ======== Variables ========
float voltageRMS = 0.0;
float currentRMS = 0.0;
float power = 0.0;
float energy = 0.0;
bool theftDetected = false;
bool tamperDetected = false;

// ======== Timing ========
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second

// ======== Simulation (set true to simulate 9W bulb without AC) ========
bool simulateLoad = false;

void setup() {
  Serial.begin(115200);

  // ADC settings
  analogReadResolution(12);        // 12-bit ADC
  analogSetAttenuation(ADC_11db);  // 0-3.6V input range

  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Smart Energy Meter");

  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (simulateLoad) {
      // ===== Simulation of 9W bulb =====
      voltageRMS = 230.0;
      currentRMS = 0.039; // 9W/230V
    } else {
      // ===== Read raw sensors =====
      int rawVoltage = analogRead(VOLTAGE_PIN);
      int rawCurrent = analogRead(CURRENT_PIN);

      // ===== Voltage conversion =====
      float sensorVoltage = (rawVoltage / ADC_RESOLUTION) * ADC_VOLTAGE; 
      voltageRMS = (sensorVoltage / ADC_VOLTAGE) * VOLTAGE_SENSOR_MAX; // adjust multiplier after calibration

      // ===== Current conversion (ACS712 20A) =====
      float currentVoltage = (rawCurrent / ADC_RESOLUTION) * ADC_VOLTAGE;
      currentRMS = (currentVoltage - ACS_ZERO_VOLT) / ACS_SENSITIVITY;
    }

    // ===== Power & Energy =====
    power = voltageRMS * currentRMS;
    energy += power / 3600.0; // Wh per second

    // ===== Tamper / Theft detection =====
    tamperDetected = digitalRead(HALL_PIN) == LOW;
    theftDetected = tamperDetected;

    if (theftDetected) {
      digitalWrite(RELAY_PIN, HIGH); // Cut power
    } else {
      digitalWrite(RELAY_PIN, LOW);
    }

    // ===== Serial Output =====
    Serial.print("Voltage: "); Serial.print(voltageRMS); Serial.print(" V | ");
    Serial.print("Current: "); Serial.print(currentRMS); Serial.print(" A | ");
    Serial.print("Power: "); Serial.print(power); Serial.print(" W | ");
    Serial.print("Energy: "); Serial.print(energy,3); Serial.print(" Wh | ");
    Serial.print("Theft: "); Serial.println(theftDetected ? "YES" : "NO");

    // ===== Display on 20x4 LCD =====
    lcd.setCursor(0,0);
    lcd.print("Voltage: "); lcd.print(voltageRMS,1); lcd.print(" V     ");
    lcd.setCursor(0,1);
    lcd.print("Current: "); lcd.print(currentRMS,3); lcd.print(" A     ");
    lcd.setCursor(0,2);
    lcd.print("Power: "); lcd.print(power,1); lcd.print(" W     ");
    lcd.setCursor(0,3);
    lcd.print("Energy: "); lcd.print(energy,3); lcd.print(" Wh ");
    lcd.print(theftDetected ? "THEFT!" : "OK    ");
  }
}
