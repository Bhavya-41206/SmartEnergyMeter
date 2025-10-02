#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32_Edgent.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// -------------------- PIN CONFIG --------------------
const int CURRENT_SENSOR_PIN = 34;
const int VOLTAGE_SENSOR_PIN = 35;
const int HALL_TAMPER_PIN     = 32;
const int RELAY_PIN           = 25;

// I2C LCD
const uint8_t LCD_ADDRESS = 0x27;
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);

// -------------------- SAMPLING --------------------
const unsigned int SAMPLES = 500;
const unsigned int SAMPLE_INTERVAL_MICROS = 250;
const float VREF = 3.3f;
const int ADC_MAX_COUNTS = 4095;

float CURRENT_SENSITIVITY_V_PER_A = 0.100f;
float VOLTAGE_DIVIDER_RATIO = 230.0f / 1.0f;
float ADC_COUNT_TO_VOLT = VREF / (float)ADC_MAX_COUNTS;

// -------------------- ENERGY --------------------
double energy_Wh = 0.0;
unsigned long lastEnergyMillis = 0;

// -------------------- TAMPER --------------------
bool tamperDetected = false;

// -------------------- DISPLAY --------------------
unsigned long lastDisplayMillis = 0;
const unsigned long DISPLAY_INTERVAL_MS = 1000;

void setup() {
  Serial.begin(115200);

  pinMode(HALL_TAMPER_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Smart Energy");
  lcd.setCursor(0,1);
  lcd.print("Metering ESP32");
  delay(1500);
  lcd.clear();

  lastEnergyMillis = millis();

  BlynkEdgent.begin();  // Edgent automatically handles WiFi & Auth
  Serial.println("Setup done. Starting measurements...");
}

void loop() {
  BlynkEdgent.run(); // keep Blynk connection alive
  readTamper();

  if (tamperDetected) {
    handleTamper();
    Blynk.virtualWrite(V5, 255); // Tamper ON
    Blynk.logEvent("tamper_alert", "Tamper Detected! Relay Tripped");
  } else {
    float Irms = readRMS_Current(CURRENT_SENSOR_PIN);
    float Vrms = readRMS_Voltage(VOLTAGE_SENSOR_PIN);
    float power_W = Vrms * Irms;

    accumulateEnergy(power_W);

    if (millis() - lastDisplayMillis >= DISPLAY_INTERVAL_MS) {
      showOnLCD(Vrms, Irms, power_W, energy_Wh);

      // Send data to Blynk
      Blynk.virtualWrite(V1, Vrms);
      Blynk.virtualWrite(V2, Irms);
      Blynk.virtualWrite(V3, power_W);
      Blynk.virtualWrite(V4, energy_Wh);
      Blynk.virtualWrite(V5, 0); // Tamper OFF

      lastDisplayMillis = millis();
    }

    Serial.print("V:"); Serial.print(Vrms);
    Serial.print(" I:"); Serial.print(Irms);
    Serial.print(" P:"); Serial.print(power_W);
    Serial.print(" E:"); Serial.println(energy_Wh);
  }
  delay(50);
}

// -------------------- FUNCTIONS --------------------
void readTamper() {
  int val = digitalRead(HALL_TAMPER_PIN);
  tamperDetected = (val == LOW);
}

void handleTamper() {
  digitalWrite(RELAY_PIN, LOW); // trip relay
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("! TAMPER ALERT !");
  lcd.setCursor(0,1); lcd.print("Load DISCONNECTED");
  Serial.println("TAMPER detected - relay tripped.");
  delay(500);
}

float readRMS_Current(int pin) {
  double sumSq = 0.0;
  for (unsigned int i=0;i<SAMPLES;i++){
    int raw = analogRead(pin);
    float v = raw*ADC_COUNT_TO_VOLT;
    float centered = v - (VREF/2.0f);
    float amps = centered / CURRENT_SENSITIVITY_V_PER_A;
    sumSq += amps*amps;
    delayMicroseconds(SAMPLE_INTERVAL_MICROS);
  }
  float Irms = sqrt(sumSq/SAMPLES);
  if (Irms<0.005f) Irms=0.0f;
  return Irms;
}

float readRMS_Voltage(int pin) {
  double sumSq = 0.0;
  for (unsigned int i=0;i<SAMPLES;i++){
    int raw = analogRead(pin);
    float vMeasured = raw*ADC_COUNT_TO_VOLT;
    float centered = vMeasured - (VREF/2.0f);
    float vInst = centered * VOLTAGE_DIVIDER_RATIO;
    sumSq += vInst*vInst;
    delayMicroseconds(SAMPLE_INTERVAL_MICROS);
  }
  float Vpeak = sqrt(sumSq/SAMPLES);
  float Vrms = Vpeak/1.41421356237f;
  if (Vrms<0.5f) Vrms=0.0f;
  return Vrms;
}

void accumulateEnergy(float powerW){
  unsigned long now = millis();
  unsigned long dt_ms = now - lastEnergyMillis;
  if (dt_ms == 0) return;
  double dt_h = dt_ms/3600000.0;
  energy_Wh += powerW*dt_h;
  lastEnergyMillis = now;
}

void showOnLCD(float Vrms, float Irms, float P, double Ewh){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("V:"); lcd.print(Vrms,1); lcd.print("V I:"); lcd.print(Irms,2); lcd.print("A");
  lcd.setCursor(0,1); lcd.print("P:"); lcd.print(P,1); lcd.print("W E:"); lcd.print(Ewh,2); lcd.print("Wh");
}
