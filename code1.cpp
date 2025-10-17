#include "EmonLib.h"       // For energy monitoring
#include <EEPROM.h>        // For storing energy data
#include <Wire.h>          // For I2C
#include <LiquidCrystal_I2C.h>  // For LCD

LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD 16x2

// Calibration constants
const float vCalibration = 42.5;  // Voltage calibration factor
const float currCalibration = 1.80;  // Current calibration factor

// EnergyMonitor instance
EnergyMonitor emon;

// Variables for energy calculation
float kWh = 0.0;
float cost = 0.0;
const float ratePerkWh = 6.5;
unsigned long lastMillis = 0;

// EEPROM addresses
const int addrKWh = 12;
const int addrCost = 16;

// Reset button pin
const int resetButtonPin = 4;

// Function prototypes
void readEnergyDataFromEEPROM();
void saveEnergyDataToEEPROM();
void updateLCD();
void resetEEPROM();

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize EEPROM
  EEPROM.begin(32);

  // Initialize reset button
  pinMode(resetButtonPin, INPUT_PULLUP);

  // Read stored data
  readEnergyDataFromEEPROM();

  // Setup voltage and current inputs
  emon.voltage(35, vCalibration, 1.7);  // Voltage: pin, calibration, phase
  emon.current(34, currCalibration);    // Current: pin, calibration

  lastMillis = millis();
}

void loop() {
  // Read voltage, current, and power
  emon.calcVI(20, 2000);  // Number of crossings, timeout
  float Vrms = emon.Vrms;
  float Irms = emon.Irms;
  float apparentPower = emon.apparentPower;

  // Update energy
  unsigned long currentMillis = millis();
  kWh += apparentPower * (currentMillis - lastMillis) / 3600000000.0;
  lastMillis = currentMillis;

  // Update cost
  cost = kWh * ratePerkWh;

  // Save to EEPROM
  saveEnergyDataToEEPROM();

  // Print to Serial
  Serial.print("Voltage: "); Serial.print(Vrms); Serial.print(" V\t");
  Serial.print("Current: "); Serial.print(Irms); Serial.print(" A\t");
  Serial.print("Power: "); Serial.print(apparentPower); Serial.print(" W\t");
  Serial.print("Energy: "); Serial.print(kWh); Serial.print(" kWh\t");
  Serial.print("Cost: "); Serial.println(cost);

  // Update LCD
  updateLCD();

  // Check reset button
  if (digitalRead(resetButtonPin) == LOW) {
    delay(200);
    resetEEPROM();
  }

  delay(2000);  // Read every 2 seconds
}

void readEnergyDataFromEEPROM() {
  EEPROM.get(addrKWh, kWh);
  EEPROM.get(addrCost, cost);

  if (isnan(kWh)) kWh = 0.0;
  if (isnan(cost)) cost = 0.0;
}

void saveEnergyDataToEEPROM() {
  EEPROM.put(addrKWh, kWh);
  EEPROM.put(addrCost, cost);
  EEPROM.commit();
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("V:%.1fV I:%.2fA", emon.Vrms, emon.Irms);
  lcd.setCursor(0, 1);
  lcd.printf("P:%.1fW E:%.2fkWh", emon.apparentPower, kWh);
}

void resetEEPROM() {
  kWh = 0.0;
  cost = 0.0;
  saveEnergyDataToEEPROM();
}
