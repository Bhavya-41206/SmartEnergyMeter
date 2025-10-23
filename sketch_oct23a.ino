#define BLYNK_TEMPLATE_ID "TMPL36n7PL96f"  // Define Blynk template ID
#define BLYNK_TEMPLATE_NAME "Hardware test"  // Define Blynk template name
#define BLYNK_PRINT Serial  // Enable Serial printing for Blynk debug information

#include "EmonLib.h"  // Include EmonLib for energy monitoring
#include <EEPROM.h>  // Include EEPROM library for storing data
#include <WiFi.h>  // Include WiFi library for network connectivity
#include <BlynkSimpleEsp32.h>  // Include Blynk library for ESP32
#include <Wire.h>  // Include Wire library for I2C communication
#include <LiquidCrystal_I2C.h>  // Include LiquidCrystal_I2C library for LCD display
#include <HTTPClient.h>  // Include HTTPClient library for HTTP requests
#include <ArduinoJson.h>  // Include ArduinoJson library for JSON handling

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Initialize LCD with I2C address 0x27 and size 16x2

// Website configuration
const char* websiteURL = "https://smart-energy-meter-portal.onrender.com/api/energy-data";
const char* deviceId = "ESP32_Energy_Meter_001"; // Unique device identifier

// Constants for calibration
const float vCalibration = 42.5;  // Voltage calibration factor
const float currCalibration = 1.80;  // Current calibration factor

// Blynk and WiFi credentials
const char auth[] = "3vCchEayeoses3vxTbX9meJvjNDeY6Z4";  // Blynk authentication token
const char ssid[] = "Bhavya";  // WiFi SSID
const char pass[] = "20062011";  // WiFi password

// EnergyMonitor instance
EnergyMonitor emon;  // Create an instance of EnergyMonitor

// Timer for regular updates
BlynkTimer timer;  // Create a Blynk timer instance

// Variables for energy calculation
float kWh = 0.0;  // Variable to store energy consumed in kWh
float cost = 0.0;  // Variable to store cost of energy consumed
const float ratePerkWh = 6.5;  // Cost rate per kWh
unsigned long lastMillis = millis();  // Variable to store last time in milliseconds

// EEPROM addresses for each variable
const int addrKWh = 12;  // EEPROM address for kWh
const int addrCost = 16;  // EEPROM address for cost

// Display page variable
int displayPage = 0;  // Variable to track current LCD display page

// Reset button pin
const int resetButtonPin = 4;  // Pin for reset button (change to your button pin)

// Theft detection variables
bool theftDetected = false;
bool tamperDetected = false;
#define HALL_SENSOR_PIN 32  // Hall effect sensor for tamper detection

// Function prototypes
void sendEnergyDataToBlynk();  // Prototype for sending energy data to Blynk
void readEnergyDataFromEEPROM();  // Prototype for reading energy data from EEPROM
void saveEnergyDataToEEPROM();  // Prototype for saving energy data to EEPROM
void updateLCD();  // Prototype for updating LCD display
void changeDisplayPage();  // Prototype for changing LCD display page
void sendDataToWebsite();  // Prototype for sending data to website
void resetEEPROM();  // Prototype for resetting EEPROM data
void checkTheft();  // Prototype for theft detection

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(resetButtonPin, INPUT_PULLUP);  // Set reset button pin as input with pull-up resistor
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);  // Hall sensor for tamper detection
  
  // Initialize the LCD
  lcd.init();  // Initialize the LCD
  lcd.backlight();  // Turn on LCD backlight
  
  lcd.setCursor(0, 0);
  lcd.print("Smart Energy");
  lcd.setCursor(0, 1);
  lcd.print("Metering System");
  delay(2000);
  lcd.clear();

  // Connect to WiFi
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  WiFi.begin(ssid, pass);  // Start WiFi connection
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);  // Wait for WiFi connection
    lcd.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    Blynk.begin(auth, ssid, pass);  // Start Blynk connection
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed");
  }
  delay(3000);
  lcd.clear();

  // Initialize EEPROM
  EEPROM.begin(32);  // Initialize EEPROM with 32 bytes of storage

  // Read stored data from EEPROM
  readEnergyDataFromEEPROM();  // Read energy data from EEPROM

  // Setup voltage and current inputs
  emon.voltage(35, vCalibration, 1.7);  // Configure voltage measurement: input pin, calibration, phase shift
  emon.current(34, currCalibration);  // Configure current measurement: input pin, calibration

  // Setup timers
  timer.setInterval(2000L, sendEnergyDataToBlynk);  // Set timer to send energy data to Blynk every 2 seconds
  timer.setInterval(2000L, changeDisplayPage);  // Set timer to change display page every 2 seconds
  timer.setInterval(5000L, sendDataToWebsite);  // Set timer to send data to website every 5 seconds
}

void loop() {
  Blynk.run();  // Run Blynk
  timer.run();  // Run timers

  // Check theft detection
  checkTheft();

  // Check if the reset button is pressed
  if (digitalRead(resetButtonPin) == LOW) {  // If reset button is pressed (assuming button press connects to ground)
    delay(200);  // Debounce delay
    resetEEPROM();  // Reset EEPROM data
  }
}

void checkTheft() {
  tamperDetected = digitalRead(HALL_SENSOR_PIN) == LOW;
  
  theftDetected = false;
  
  if (tamperDetected) {
    theftDetected = true;
    Serial.println("⚠ TAMPER detected (Hall sensor).");
  } else if (emon.Vrms > 180 && emon.Irms < 0.05) {
    theftDetected = true;
    Serial.println("⚠ POWER THEFT detected (bypass condition).");
  }
}

void sendEnergyDataToBlynk() {
  emon.calcVI(20, 2000);  // Calculate voltage and current: number of half wavelengths (crossings), time-out
  float Vrms = emon.Vrms;  // Get root mean square voltage
  float Irms = emon.Irms;  // Get root mean square current
  float apparentPower = emon.apparentPower;  // Get apparent power

  // Calculate energy consumed in kWh
  unsigned long currentMillis = millis();  // Get current time in milliseconds
  kWh += apparentPower * (currentMillis - lastMillis) / 3600000000.0;  // Update kWh
  lastMillis = currentMillis;  // Update last time

  // Calculate the cost based on the rate per kWh
  cost = kWh * ratePerkWh;  // Calculate cost

  // Save the latest values to EEPROM
  saveEnergyDataToEEPROM();  // Save energy data to EEPROM

  // Convert kWh to Wh for Blynk V3
  float energyWh = kWh * 1000.0;

  // Send data to Blynk - ONLY V0 to V3
  Blynk.virtualWrite(V0, Vrms);  // Send voltage to Blynk virtual pin V0
  Blynk.virtualWrite(V1, Irms);  // Send current to Blynk virtual pin V1
  Blynk.virtualWrite(V2, apparentPower);  // Send apparent power to Blynk virtual pin V2
  Blynk.virtualWrite(V3, energyWh);  // Send energy in Wh to Blynk virtual pin V3

  // Debug print
  Serial.print("Blynk - V0:");
  Serial.print(Vrms);
  Serial.print(" V1:");
  Serial.print(Irms);
  Serial.print(" V2:");
  Serial.print(apparentPower);
  Serial.print(" V3:");
  Serial.print(energyWh);
  Serial.println("Wh");

  // Update the LCD with the new values
  updateLCD();  // Update LCD display
}

void sendDataToWebsite() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    
    // Create JSON data
    DynamicJsonDocument doc(1024);
    doc["deviceId"] = deviceId;
    doc["voltage"] = emon.Vrms;
    doc["current"] = emon.Irms;
    doc["power"] = emon.apparentPower;
    doc["energy"] = kWh;
    doc["cost"] = cost;
    doc["theftDetected"] = theftDetected;
    doc["tamperDetected"] = tamperDetected;
    doc["timestamp"] = millis();

    String jsonString;
    serializeJson(doc, jsonString);

    // Debug print
    Serial.println("Sending to website:");
    Serial.println(jsonString);
    Serial.print("URL: ");
    Serial.println(websiteURL);

    // Send POST request
    http.begin(websiteURL);
    http.addHeader("Content-Type", "application/json");
    
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
      Serial.print("Data sent successfully. Response: ");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.print("Server response: ");
      Serial.println(response);
    } else {
      Serial.print("Error: ");
      Serial.println(httpResponseCode);
    }
    
    http.end();
  }
}

void readEnergyDataFromEEPROM() {
  EEPROM.get(addrKWh, kWh);  // Read kWh from EEPROM
  EEPROM.get(addrCost, cost);  // Read cost from EEPROM

  // Initialize to zero if values are invalid
  if (isnan(kWh)) {
    kWh = 0.0;  // Set kWh to 0 if invalid
    saveEnergyDataToEEPROM();  // Save to EEPROM
  }
  if (isnan(cost)) {
    cost = 0.0;  // Set cost to 0 if invalid
    saveEnergyDataToEEPROM();  // Save to EEPROM
  }
}

void saveEnergyDataToEEPROM() {
  EEPROM.put(addrKWh, kWh);  // Save kWh to EEPROM
  EEPROM.put(addrCost, cost);  // Save cost to EEPROM
  EEPROM.commit();  // Commit EEPROM changes
}

void updateLCD() {
  lcd.clear();  // Clear LCD display
    lcd.setCursor(0, 0);  // Set cursor to first row
    lcd.printf("V:%.1fV I:%.2fA", emon.Vrms, emon.Irms);  // Print voltage and current
    lcd.setCursor(0, 1);  // Set cursor to second row
    lcd.printf("P:%.1fW E:%.1fWh", emon.apparentPower, kWh*1000);  // Print power and energy
  
    lcd.setCursor(0, 2);  // Set cursor to first row
    lcd.printf("Energy: %.3fkWh", kWh);  // Print energy
    lcd.setCursor(0, 3);  // Set cursor to second row
    if (theftDetected) {
      lcd.print("THEFT DETECTED!");
    } else {
      lcd.printf("Cost: Rs.%.2f", cost);  // Print cost
    
  }
}

void changeDisplayPage() {
  displayPage = (displayPage + 1) % 2;  // Change display page
  updateLCD();  // Update LCD display
}

void resetEEPROM() {
  kWh = 0.0;  // Reset kWh to 0
  cost = 0.0;  // Reset cost to 0
  saveEnergyDataToEEPROM();  // Save to EEPROM
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Data Reset");
  lcd.setCursor(0, 1);
  lcd.print("Successfully");
  delay(2000);
}
