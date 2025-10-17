/* Smart Energy Meter (ESP32 + Blynk + LCD + Gov Logging + WiFiManager + Webhook)
   - Proper voltage divider calculation (set your resistor values R_UP, R_DOWN)
   - Current displayed/sent in mA
   - WiFiManager captive-AP for one-time WiFi provisioning (no repeated manual credentials)
   - Webhook POST when theft detected (to your website/backend)
   - Calibration multipliers for voltage/current
*/

#define BLYNK_TEMPLATE_ID "TMPL36n7PL96f"
#define BLYNK_TEMPLATE_NAME "Hardware test"
#define BLYNK_AUTH_TOKEN "3vCchEayeoses3vxTbX9meJvjNDeY6Z4"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// -------------------- CONFIG --------------------
char auth[] = BLYNK_AUTH_TOKEN;

BlynkTimer timer;

const int CURRENT_SENSOR_PIN = 34;   // ADC pin for current sensor
const int VOLTAGE_SENSOR_PIN = 35;   // ADC pin for voltage divider
const int HALL_TAMPER_PIN    = 32;
const int RELAY_PIN          = 25;

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ADC / sampling
const unsigned int SAMPLES = 200;
const unsigned int SAMPLE_INTERVAL_US = 200;
const float VREF = 3.3f;
const int ADC_MAX = 4095;
const float ADC_TO_V = VREF / (float)ADC_MAX;

// --- Voltage divider physical resistors (SET THESE to match your hardware) ---
// R_up = resistor from mains stepped-down node to ADC input (the resistor to VIN side)
// R_down = resistor from ADC node to ground
// Voltage divider ratio = (R_up + R_down) / R_down
// Example placeholders: R_up = 300000 (300k), R_down = 6800 (6.8k)
const float R_up = 300000.0f;  // <-- set to the resistor you used
const float R_down = 6800.0f;  // <-- set to the resistor you used

const float VOLTAGE_DIVIDER_RATIO = (R_up + R_down) / R_down;

// --- Current sensor specifics (SET ACCORDING TO SENSOR) ---
// If using ACS712-30A: sensitivity ~ 0.066 V / A (for 30A module). For ACS712-20A it's 0.100 V/A.
// Set CURRENT_SENSITIVITY_V_PER_A accordingly.
const float CURRENT_SENSITIVITY_V_PER_A = 0.100f; // adjust for your module

// Use calibration multipliers to fine tune readings after testing
float voltageCalibration = 1.0f; // multiply Vrms by this
float currentCalibration = 1.0f; // multiply Irms by this

// energy
double energy_Wh = 0.0;
unsigned long lastEnergyMillis = 0;

// flags
bool tamperDetected = false;
bool theftDetected = false;

// Government / gov Blynk token (same idea)
const char* GOV_AUTH = "ukiSbVaUr-3h2-Sorur2JGiXxQ6LBMct";
const unsigned long GOV_INTERVAL_MS = 60000;
unsigned long lastGovMillis = 0;

// Webhook (your website) to notify when theft/tamper occurs
const char* WEBHOOK_URL = "https://yourserver.example.com/esp_alert"; // <-- set this to your server endpoint

// -------------------- BLYNK CONTROLS --------------------
BLYNK_WRITE(V4) {    // relay control from app
  int relayState = param.asInt();
  digitalWrite(RELAY_PIN, relayState);
}

BLYNK_WRITE(V5) {    // reset alerts button in dashboard
  tamperDetected = false;
  theftDetected = false;
  digitalWrite(RELAY_PIN, HIGH);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Alerts Reset");
  delay(800);
  lcd.clear();
}

// -------------------- UTILS --------------------
void sendWebhook(const char* eventType, float Vrms, float Irms_mA, float powerW) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(WEBHOOK_URL);
  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"event\":\""; payload += eventType; payload += "\"";
  payload += ",\"Vrms\":"; payload += String(Vrms,2);
  payload += ",\"Irms_mA\":"; payload += String(Irms_mA,2);
  payload += ",\"P_W\":"; payload += String(powerW,2);
  payload += ",\"ts\":"; payload += String(millis()/1000);
  payload += "}";

  int code = http.POST(payload);
  if (code > 0) {
    Serial.println("Webhook POST code: " + String(code));
  } else {
    Serial.println("Webhook POST failed");
  }
  http.end();
}

void readTamper() {
  tamperDetected = (digitalRead(HALL_TAMPER_PIN) == LOW);
}

float readMeasurements(float &Vrms_out, float &Irms_out) {
  double sumV2 = 0, sumI2 = 0, sumVI = 0;

  for (unsigned int i = 0; i < SAMPLES; i++) {
    int rawV = analogRead(VOLTAGE_SENSOR_PIN);
    int rawI = analogRead(CURRENT_SENSOR_PIN);

    float vADC = rawV * ADC_TO_V; // measured ADC voltage
    float iADC = rawI * ADC_TO_V;

    // Convert ADC voltage to instantaneous mains volt and amps
    // We assume the sensors are AC-coupled and biased at VREF/2.
    float vInst = (vADC - (VREF / 2.0f)) * VOLTAGE_DIVIDER_RATIO * voltageCalibration;
    float iInst = (iADC - (VREF / 2.0f)) / CURRENT_SENSITIVITY_V_PER_A * currentCalibration;

    sumV2 += vInst * vInst;
    sumI2 += iInst * iInst;
    sumVI += vInst * iInst;

    delayMicroseconds(SAMPLE_INTERVAL_US);
  }

  float Vrms = sqrt(sumV2 / SAMPLES);
  float Irms = sqrt(sumI2 / SAMPLES);
  float realPower = sumVI / SAMPLES;

  if (Vrms < 0.5f) Vrms = 0.0f;
  if (Irms < 0.0005f) Irms = 0.0f;
  if (realPower < 0.05f) realPower = 0.0f;

  Vrms_out = Vrms;
  Irms_out = Irms;
  return realPower;
}

void accumulateEnergy(float powerW) {
  unsigned long now = millis();
  if (lastEnergyMillis == 0) lastEnergyMillis = now;
  double dt_h = (now - lastEnergyMillis) / 3600000.0;
  energy_Wh += powerW * dt_h;
  lastEnergyMillis = now;
}

void updateLCD(float Vrms, float Irms_mA, float P, double Ewh) {
  // update without full clear to reduce flicker
  lcd.setCursor(0,0);
  char line1[21];
  snprintf(line1, sizeof(line1), "V:%6.1fV  I:%7.1fmA", Vrms, Irms_mA);
  lcd.print(line1);

  lcd.setCursor(0,1);
  char line2[21];
  snprintf(line2, sizeof(line2), "P:%6.1fW  E:%6.1fWh", P, Ewh);
  lcd.print("                "); // overwrite
  lcd.setCursor(0,1);
  lcd.print(line2);
}

// -------------------- GOVERNMENT LOGGING --------------------
void sendToGovernment(float Vrms, float Irms, float powerW, double energyWh) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  String url = "https://blynk.cloud/external/api/update?token=" + String(GOV_AUTH) +
               "&V1=" + String(Vrms,2) + "&V2=" + String(Irms,3) +
               "&V3=" + String(powerW,2) + "&V4=" + String(energyWh,2) +
               "&V5=" + String(millis()/1000);
  http.begin(url);
  int httpCode = http.GET();
  Serial.println("Gov send code: " + String(httpCode));
  http.end();
}

// -------------------- MAIN TASK --------------------
void measureAndUpdate() {
  readTamper();
  if (tamperDetected) {
    digitalWrite(RELAY_PIN, LOW);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("! TAMPER ALERT !");
    lcd.setCursor(0,1);
    lcd.print("Load DISCONNECTED");
    Serial.println("⚠ TAMPER detected - relay tripped.");
    Blynk.logEvent("tamper_alert", "⚠ TAMPER ALERT! Load disconnected!");
    sendWebhook("tamper", 0, 0, 0);
    return;
  }

  float Vrms, Irms_A;
  float power_W = readMeasurements(Vrms, Irms_A);

  // convert Irms to mA for display & web/Blynk
  float Irms_mA = Irms_A * 1000.0f;

  // Theft detection: low current while mains present
  if (Vrms > 180.0f && Irms_A < 0.05f) {
    theftDetected = true;
    digitalWrite(RELAY_PIN, LOW);
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("! POWER THEFT !");
    lcd.setCursor(0,1); lcd.print("Bypass Suspected");
    Serial.println("⚠ Power theft detected (bypass).");
    Blynk.logEvent("power_theft", "⚠ POWER THEFT detected! Bypass suspected!");
    // send webhook to your website backend
    sendWebhook("power_theft", Vrms, Irms_mA, power_W);
    return;
  }

  accumulateEnergy(power_W);
  updateLCD(Vrms, Irms_mA, power_W, energy_Wh);

  // send to Blynk app (V0-V3). Note: send current in mA to match your requirement.
  Blynk.virtualWrite(V0, Vrms);
  Blynk.virtualWrite(V1, Irms_mA);
  Blynk.virtualWrite(V2, power_W);
  Blynk.virtualWrite(V3, energy_Wh);

  if (millis() - lastGovMillis > GOV_INTERVAL_MS) {
    sendToGovernment(Vrms, Irms_A, power_W, energy_Wh);
    lastGovMillis = millis();
  }

  Serial.printf("Vrms=%.2f V, Irms=%.3f A (%.1fmA), P=%.2f W, E=%.3f Wh\n",
                Vrms, Irms_A, Irms_mA, power_W, energy_Wh);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.init();
  lcd.backlight();

  pinMode(HALL_TAMPER_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // default: relay ON

  // WiFiManager: start captive portal if no saved credentials
  WiFiManager wifiManager;
  wifiManager.autoConnect("SmartMeter-Setup"); // AP shown if no creds saved

  // Start Blynk after WiFi is connected
  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());

  lastEnergyMillis = millis();

  timer.setInterval(1000L, measureAndUpdate); // run every 1s
}

// -------------------- LOOP --------------------
void loop() {
  Blynk.run();
  timer.run();
}
