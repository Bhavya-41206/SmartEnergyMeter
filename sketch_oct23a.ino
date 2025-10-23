// Simple ESP32 Relay Test
const int relayPin = 4;

void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  Serial.println("ESP32 Relay Test Ready");
}

void loop() {
  // Try both patterns to find what works
  
  // Pattern A: Active LOW
  Serial.println("Trying Active LOW - Relay should turn ON");
  digitalWrite(relayPin, LOW);
  delay(3000);
  
  Serial.println("Relay should turn OFF");
  digitalWrite(relayPin, HIGH);
  delay(3000);
  
  // Pattern B: Active HIGH  
  Serial.println("Trying Active HIGH - Relay should turn ON");
  digitalWrite(relayPin, HIGH);
  delay(3000);
  
  Serial.println("Relay should turn OFF");
  digitalWrite(relayPin, LOW);
  delay(3000);
}