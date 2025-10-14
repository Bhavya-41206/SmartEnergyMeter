/* Smart Energy Meter (ESP32 + Blynk + LCD + Gov Logging)
   - RMS Voltage & Current Measurement
   - Real-time Power (W) & Energy (Wh) accumulation
   - Tamper detection (Hall Sensor) -> relay trip & alarm
   - Power Theft detection (bypass condition)
   - Local Display (I2C LCD)
   - Cloud Monitoring:
       User dashboard via Blynk (mobile)
       Government logging via HTTP API (desktop auth token)
   - Mobile notifications for tamper/theft alerts
*/

#define BLYNK_TEMPLATE_ID "TMPL36n7PL96f"
#define BLYNK_TEMPLATE_NAME "Hardware test"
#define BLYNK_AUTH_TOKEN "YourUserAuthTokenHere"   // 🔹 Replace this with your own Blynk Auth Token

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>  // ✅ Use this instead of BlynkEdgent.h

// -------------------- BLYNK USER DASHBOARD --------------------
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "YourWiFiName";       // 🔹 Replace with your Wi-Fi SSID
char pass[] = "YourWiFiPassword";   // 🔹 Replace with your Wi-Fi Password

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

// -------------------- GOVERNMENT LOGGING --------------------
const char* GOV_AUTH = "ukiSbVaUr-3h2-Sorur2JGiXxQ6LBMct"; // replace with Desktop Auth Token
const unsigned long GOV_INTERVAL_MS = 60000;          // send data every 1 minute
unsigned long lastGovMillis = 0;

// -------------------- BLYNK RELAY CONTROL --------------------
BLYNK_WRITE(V4) {   // Button on user dashboard to control Relay
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
  Blynk.logEvent("tamper_alert", "⚠ TAMPER ALERT! Load disconnected!");
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
  double dt_h = (now - lastEnergyMillis) / 3600000.0;
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

// -------------------- GOVERNMENT LOGGING --------------------
void sendToGovernment(float Vrms, float Irms, float powerW, double energyWh) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "https://blynk.cloud/external/api/update?token=";
    url += GOV_AUTH;
    url += "&V1="; url += Vrms;
    url += "&V2="; url += Irms;
    url += "&V3="; url += powerW;
    url += "&V4="; url += energyWh;
    url += "&V5="; url += String(millis()/1000); // timestamp in seconds

    http.begin(url);
    int httpCode = http.GET();
    if (httpCode > 0) {
      Serial.println("Sent to Government Blynk, code: " + String(httpCode));
    } else {
      Serial.println("Error sending to Government Blynk");
    }
    http.end();
  }
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

    Blynk.logEvent("power_theft", "⚠ POWER THEFT detected! Bypass suspected!");
    digitalWrite(RELAY_PIN, LOW);
    return;
  }

  accumulateEnergy(power_W);
  showOnLCD(Vrms, Irms, power_W, energy_Wh);

  // Send to User Blynk (mobile)
  Blynk.virtualWrite(V0, Vrms);
  Blynk.virtualWrite(V1, Irms);
  Blynk.virtualWrite(V2, power_W);
  Blynk.virtualWrite(V3, energy_Wh);

  // Send to Government Blynk (desktop) every GOV_INTERVAL_MS
  if (millis() - lastGovMillis > GOV_INTERVAL_MS) {
    sendToGovernment(Vrms, Irms, power_W, energy_Wh);
    lastGovMillis = millis();
  }

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
  lcd.setCursor(0,0); lcd.print("Smart Energy Meter");
  lcd.setCursor(0,1); lcd.print("Connecting WiFi...");
  delay(1500);
  lcd.clear();

  pinMode(HALL_TAMPER_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Blynk.begin(auth, ssid, pass);   // ✅ Normal Wi-Fi + Blynk connection
  lastEnergyMillis = millis();
  timer.setInterval(1000L, measureAndUpdate); // measure every second
}

// -------------------- LOOP --------------------
void loop() {
  Blynk.run();
  timer.run();
}
