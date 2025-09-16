/* Smart Energy Meter (ESP32) - Arduino IDE
   - Reads current sensor (analog)
   - Reads voltage sensor (analog)
   - Computes Vrms, Irms, Power (W), accumulates Energy (Wh)
   - Hall effect tamper detection (digital) -> trips relay and shows alarm
   - Displays values on I2C LCD (16x2)
   - Author: example for your project (calibrate constants before use)
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// -------------------- PIN CONFIG --------------------
const int CURRENT_SENSOR_PIN = 34;   // ADC1_CH6 (GPIO34) - input only
const int VOLTAGE_SENSOR_PIN = 35;   // ADC1_CH7 (GPIO35)
const int HALL_TAMPER_PIN     = 32;  // Digital input from hall effect sensor (D0)
const int RELAY_PIN           = 25;  // Relay control (active HIGH assumed)

// I2C LCD
const uint8_t LCD_ADDRESS = 0x27; // change to 0x3F if your module uses that
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);

// -------------------- SAMPLING / CALIBRATION --------------------
// Sampling parameters
const unsigned int SAMPLES = 500;   // samples per RMS measurement (increase for smoother)
const unsigned int SAMPLE_INTERVAL_MICROS = 250; // microseconds between samples (~4kHz sampling)

// ADC reference & counts (ESP32 ADC default bounds)
const float VREF = 3.3f;            // ESP32 reference voltage (approx) - calibrate if necessary
const int ADC_MAX_COUNTS = 4095;    // 12-bit ADC for ESP32 core

// Sensor calibration - YOU MUST CALIBRATE THESE
// Current sensor: output sits at Vref/2 at 0A and varies +/- sensitivity*I
// Example: ACS712 5A -> ~0.185 V/A
float CURRENT_SENSITIVITY_V_PER_A = 0.100f; // V per amp (placeholder) - set for your sensor
// Voltage sensor: ratio to convert ADC measured volts -> mains peak voltage.
// Example: if your sensor outputs 1V when mains peak is 325V, ratio = 325.0
float VOLTAGE_DIVIDER_RATIO = 230.0f / 1.0f; // placeholder: actual Vrms = measuredVoltage * (VOLTAGE_DIVIDER_RATIO / sqrt(2))

// Derived calibration convenience:
float ADC_COUNT_TO_VOLT = VREF / (float)ADC_MAX_COUNTS; // Volts per ADC count

// -------------------- ENERGY ACCUMULATION --------------------
double energy_Wh = 0.0;            // accumulated watt-hours
unsigned long lastEnergyMillis = 0;

// -------------------- TAMper / SAFETY --------------------
bool tamperDetected = false;
const unsigned long TAMPER_RETRY_MS = 10000; // time to wait after tamper before auto-reset attempt (optional)

// -------------------- DISPLAY / UI --------------------
unsigned long lastDisplayMillis = 0;
const unsigned long DISPLAY_INTERVAL_MS = 1000; // update display every second

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // pins
  pinMode(HALL_TAMPER_PIN, INPUT_PULLUP); // hall sensor likely pulls low when magnet present; adjust if needed
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // initial relay off (or change to HIGH if your relay is active LOW)

  // LCD init
  Wire.begin(); // SDA(21) SCL(22) default for ESP32
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // show startup
  lcd.setCursor(0,0);
  lcd.print("Smart Energy");
  lcd.setCursor(0,1);
  lcd.print("Metering - ESP32");
  delay(1500);
  lcd.clear();

  lastEnergyMillis = millis();
  Serial.println("Setup done. Starting measurements...");
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // Read tamper/hall sensor
  readTamper();

  // If tamper detected - trip relay and show error
  if (tamperDetected) {
    handleTamper();
  } else {
    // normal measurement
    float Irms = readRMS_Current(CURRENT_SENSOR_PIN);
    float Vrms = readRMS_Voltage(VOLTAGE_SENSOR_PIN);

    // calculate power and accumulate energy
    float power_W = Vrms * Irms; // approximate real power (no phase correction)
    // ----------------- Theft Detection -----------------
    if (Vrms > 200.0 && Irms < 0.05) { 
      // mains present, but very little current -> bypass theft
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("! POWER THEFT !");
      lcd.setCursor(0,1);
      lcd.print("Bypass Suspected");
      Serial.println("Power theft detected (bypass).");
      digitalWrite(RELAY_PIN, HIGH); // trip relay
    } else {
      accumulateEnergy(power_W);

      if (millis() - lastDisplayMillis >= DISPLAY_INTERVAL_MS) {
        showOnLCD(Vrms, Irms, power_W, energy_Wh);
        lastDisplayMillis = millis();
      }

      Serial.print("Vrms: "); Serial.print(Vrms, 2);
      Serial.print(" V, Irms: "); Serial.print(Irms, 3);
      Serial.print(" A, P: "); Serial.print(power_W, 2);
      Serial.print(" W, Energy(Wh): "); Serial.println(energy_Wh, 4);
    }
  }

  // short delay to avoid tight looping (measurements include internal delays)
  delay(50);
}

// -------------------- FUNCTIONS --------------------

// Read tamper / hall effect sensor (digital)
void readTamper() {
  int val = digitalRead(HALL_TAMPER_PIN);
  // depending on your module logic: adjust detection condition
  // Many hall modules output LOW when magnet is present -> treat as tamper
  if (val == LOW) {
    tamperDetected = true;
  } else {
    tamperDetected = false;
  }
}

// On tamper: trip relay (turn off load), show alarm
void handleTamper() {
  // Trip relay (turn it OFF to disconnect load). If your relay is active LOW, invert logic.
  digitalWrite(RELAY_PIN, HIGH); // using HIGH to trip — change if circuit differs
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("! TAMPER ALERT !");
  lcd.setCursor(0,1);
  lcd.print("Load DISCONNECTED");
  Serial.println("TAMPER detected - relay tripped.");
  // We stay here until tamper cleared. Optionally attempt auto-reset after a delay.
  // Simple behavior: do not continue measurements while tamper persists.
  delay(500);
}

// Compute RMS from current sensor by sampling ADC
float readRMS_Current(int pin) {
  // measure samples, convert to volts, subtract offset, compute rms in amps
  unsigned long start = micros();
  double sumSq = 0.0;
  for (unsigned int i = 0; i < SAMPLES; ++i) {
    int raw = analogRead(pin); // 0..4095
    float v = raw * ADC_COUNT_TO_VOLT; // measured voltage at sensor pin
    // current sensor center offset assumed at VREF/2
    float centered = v - (VREF / 2.0f);
    float amps = centered / CURRENT_SENSITIVITY_V_PER_A;
    sumSq += (double)amps * (double)amps;
    delayMicroseconds(SAMPLE_INTERVAL_MICROS);
  }
  unsigned long dur = micros() - start;
  float meanSq = sumSq / (double)SAMPLES;
  float Irms = sqrt(meanSq);
  // If very small noise -> return 0 to avoid showing micro amps
  if (Irms < 0.005f) Irms = 0.0f;
  return Irms;
}

// Compute RMS from voltage sensor by sampling ADC
float readRMS_Voltage(int pin) {
  unsigned long start = micros();
  double sumSq = 0.0;
  for (unsigned int i = 0; i < SAMPLES; ++i) {
    int raw = analogRead(pin);
    float vMeasured = raw * ADC_COUNT_TO_VOLT; // volts at sensor output
    // If your voltage sensor outputs centered at Vref/2, subtract that
    float centered = vMeasured - (VREF / 2.0f);
    // Convert to mains peak voltage using your sensor ratio (calibrate)
    // instantaneous mains voltage (approx): vinstant = centered * VOLTAGE_DIVIDER_RATIO
    float vInst = centered * VOLTAGE_DIVIDER_RATIO;
    sumSq += (double)vInst * (double)vInst;
    delayMicroseconds(SAMPLE_INTERVAL_MICROS);
  }
  unsigned long dur = micros() - start;
  float meanSq = sumSq / (double)SAMPLES;
  float Vpeak = sqrt(meanSq);
  // convert peak to RMS: Vrms = Vpeak / sqrt(2)
  float Vrms = Vpeak / 1.41421356237f;
  if (Vrms < 0.5f) Vrms = 0.0f;
  return Vrms;
}

// Accumulate energy: energy_Wh increases by P(W)*dt(h)
void accumulateEnergy(float powerW) {
  unsigned long now = millis();
  unsigned long dt_ms = now - lastEnergyMillis;
  if (dt_ms == 0) return;
  double dt_h = (double)dt_ms / 3600000.0; // ms -> hours
  energy_Wh += (double)powerW * dt_h;
  lastEnergyMillis = now;
}

// Write key values to LCD
void showOnLCD(float Vrms, float Irms, float P, double Ewh) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("V:"); lcd.print(Vrms,1); lcd.print("V ");
  lcd.print("I:"); lcd.print(Irms,2); lcd.print("A");
  lcd.setCursor(0,1);
  lcd.print("P:"); lcd.print(P,1); lcd.print("W ");
  lcd.print("E:"); lcd.print(Ewh,2); lcd.print("Wh");
}

