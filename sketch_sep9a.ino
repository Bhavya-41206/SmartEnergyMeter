/* Smart Energy Meter (ESP32 + Blynk.Edgent + LCD)
   - RMS Voltage & Current Measurement
   - Real-time Power (W) & Energy (Wh) accumulation
   - Tamper detection (Hall Sensor) -> relay trip & alarm
   - Power Theft detection (bypass condition)
   - Local Display (I2C LCD)
   - Cloud Monitoring (Blynk App, auto WiFi provisioning)
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkEdgent.h>

// -------------------- BLYNK CONFIG --------------------
#define BLYNK_TEMPLATE_ID "TMPL36n7PL96f"
#define BLYNK_TEMPLATE_NAME "Smart Energy Meter"
#define BLYNK_FIRMWARE_VERSION "0.2.0"

BlynkTimer timer;

// -------------------- PIN CONFIG --------------------
const int CURRENT_SENSOR_PIN = 34;
const int VOLTAGE_SENSOR_PIN = 35;
const int HALL_TAMPER_PIN    = 32;
const int RELAY_PIN          = 25;

// I2C LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// -------------------- SAMPLING / CALIBRATION --------------------
const unsigned int SAMPLES = 500;
const unsigned int SAMPLE_INTERVAL_MICROS = 250;

const float VREF = 3.3f;
const int ADC_MAX_COUNTS = 4095;
float ADC_COUNT_TO_VOLT = VREF / (float)ADC_MAX_COUNTS;

float CURRENT_SENSITIVITY_V_PER_A = 0.100f;  // Calibrate for your sensor
float VOLTAGE_DIVIDER_RATIO = 230.0f / 1.0f; // Calibrate for your sensor

// -------------------- ENERGY --------------------
double energy_Wh = 0.0;
unsigned long lastEnergyMillis = 0;

// -------------------- FLAGS --------------------
bool tamperDetected = false;

// -------------------- BLYNK RELAY CONTROL --------------------
BLYNK_WRITE(V4) {   // Button on Blynk to control Relay
  int relayState = param.asInt();
  digitalWrite(RELAY_PIN, relayState);
}

// -------------------- FUNCTIONS --------------------
void readTamper() {
  tamperDetected = (digitalRead(HALL_TAMPER_PIN) == LOW);
}

void handleTamper() {
  digitalWrite(RELAY_PIN, LOW);  // Trip relay
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("! TAMPER ALERT !");
  lcd.setCursor(0,1); lcd.print("Load DISCONNECTED");
  Serial.println("⚠ TAMPER detected - relay tripped.");
  Blynk.virtualWrite(V5, "⚠ TAMPER ALERT!");
}

float readMeasurements(float &Vrms, float &Irms) {
  double sumV2 = 0, sumI2 = 0, sumVI = 0;

  for (unsigned int i = 0; i < SAMPLES; i++) {
    int rawV = analogRead(VOLTAGE_SENSOR_PIN);
    int rawI = analogRead(CURRENT_SENSOR_PIN);

    float vADC = rawV * ADC_COUNT_TO_VOLT;
    float iADC = rawI * ADC_COUNT_TO_VOLT;

    float vInst = (vADC - VREF/2.0f) * VOLTAGE_DIVIDER_RATIO;
    float iInst = (iADC - VREF/2.0f) / CURRENT_SENSITIVITY_V_PER_A;

    sumV2 += vInst * vInst;
    sumI2 += iInst * iInst;
    sumVI += vInst * iInst;

    delayMicroseconds(SAMPLE_INTERVAL_MICROS);
  }

  Vrms = sqrt(sumV2 / SAMPLES);
  Irms = sqrt(sumI2 / SAMPLES);
  float realPower = sumVI / SAMPLES;

  if (Vrms < 0.5f) Vrms = 0;
  if (Irms < 0.005f) Irms = 0;
  if (realPower < 0.1f) realPower = 0;

  return realPower;
}

void accumulateEnergy(float powerW) {
  unsigned long now = millis();
  unsigned long dt_ms = now - lastEnergyMillis;
  if (dt_ms == 0) return;
  double dt_h = (double)dt_ms / 3600000.0;
  energy_Wh += powerW * dt_h;
  lastEnergyMillis = now;
}

void showOnLCD(float Vrms, float Irms, float P, double Ewh) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("V:"); lcd.print(Vrms,1); lcd.print(" I:"); lcd.print(Irms,2);
  lcd.setCursor(0,1);
  lcd.print("P:"); lcd.print(P,1); lcd.print("W E:"); lcd.print(Ewh,1);
}

// -------------------- MEASUREMENT TASK --------------------
void measureAndUpdate() {
  readTamper();

  if (tamperDetected) {
    handleTamper();
    return;
  }

  float Vrms, Irms;
  float power_W = readMeasurements(Vrms, Irms);

  // -------- THEFT DETECTION --------
  if (Vrms > 180.0 && Irms < 0.05) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("! POWER THEFT !");
    lcd.setCursor(0,1); lcd.print("Bypass Suspected");
    Serial.println("⚠ Power theft detected (bypass).");
    Blynk.virtualWrite(V5, "⚠ THEFT DETECTED!");
    digitalWrite(RELAY_PIN, LOW); // trip relay
    return;
  }

  accumulateEnergy(power_W);

  // Display
  showOnLCD(Vrms, Irms, power_W, energy_Wh);

  // Send to Blynk
  Blynk.virtualWrite(V0, Vrms);
  Blynk.virtualWrite(V1, Irms);
  Blynk.virtualWrite(V2, power_W);
  Blynk.virtualWrite(V3, energy_Wh);

  // Debug
  Serial.printf("Vrms=%.2f V, Irms=%.3f A, P=%.2f W, E=%.3f Wh\n",
                Vrms, Irms, power_W, energy_Wh);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // LCD init
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("Smart Energy");
  lcd.setCursor(0,1); lcd.print("Metering ESP32");
  delay(1500);
  lcd.clear();

  pinMode(HALL_TAMPER_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  BlynkEdgent.begin();   // WiFi provisioning
  lastEnergyMillis = millis();

  // Schedule measurement every 1s
  timer.setInterval(1000L, measureAndUpdate);
}

// -------------------- LOOP --------------------
void loop() {
  BlynkEdgent.run();
  timer.run();
}
