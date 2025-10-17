#include "EmonLib.h"          // Energy monitoring
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize 20x4 LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Sensor calibration constants (from your setup)
const float vCalibration = 42.5;
const float currCalibration = 1.80;

// Proportional scaling factors (adjust after measuring a known load)
float voltageFactor = 4.34;  // Example: adjust so measured Vrms matches 230V
float currentFactor = 1.3;   // Example: adjust so measured Irms matches actual load

// EnergyMonitor instance
EnergyMonitor emon;

// Energy variables
float kWh = 0.0;
const float ratePerkWh = 6.5;
unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Setup voltage and current inputs
  emon.voltage(35, vCalibration, 1.7);
  emon.current(34, currCalibration);

  lastMillis = millis();
}

void loop() {
  // Measure voltage, current, power
  emon.calcVI(20, 2000);

  // Apply proportional scaling
  float Vrms = emon.Vrms * voltageFactor;
  float Irms = emon.Irms * currentFactor;
  float apparentPower = Vrms * Irms;  // approximate power

  // Update energy
  unsigned long currentMillis = millis();
  kWh += apparentPower * (currentMillis - lastMillis) / 3600000000.0;
  lastMillis = currentMillis;

  // Calculate cost
  float cost = kWh * ratePerkWh;

  // Print to Serial
  Serial.print("Voltage: "); Serial.print(Vrms); Serial.print(" V\t");
  Serial.print("Current: "); Serial.print(Irms); Serial.print(" A\t");
  Serial.print("Power: "); Serial.print(apparentPower); Serial.print(" W\t");
  Serial.print("Energy: "); Serial.print(kWh); Serial.print(" kWh\t");
  Serial.print("Cost: "); Serial.println(cost);

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("Voltage : %.1f V", Vrms);
  lcd.setCursor(0, 1);
  lcd.printf("Current : %.2f A", Irms);
  lcd.setCursor(0, 2);
  lcd.printf("Power   : %.1f W", apparentPower);
  lcd.setCursor(0, 3);
  lcd.printf("Energy  : %.2f kWh", kWh);

  delay(2000);  // Update every 2 seconds
}
