#include <Arduino.h>

// --- KONFIGURATION AF PORTE ---
const int sensorPin = 35;       // Fugtighedsmåler (Jord)
const int relayPin = 4;         // Relæ (Pumpe)
const int tankSwitchPin = 32;   // Vandtank sensor (Beskytter pumpe)
const int overflowPin = 33;     // Karse-spand sensor (Overløbs-sikring)

// Lamper
const int greenLedPin = 21;     // Alt OK (Grøn)
const int yellowLedPin = 22;    // Tank tom (Gul)
const int redLedPin = 19;       // Vander / Alarm (Rød)

// --- INDSTILLINGER ---
const int dryThreshold = 2400;  // Grænse for tør jord
const int pumpDuration = 1000;  // Vandetid (1 sek)
const int soakTime = 30000;     // Ventetid (30 sek)

void setup() {
  Serial.begin(115200);
  
  // Inputs
  pinMode(sensorPin, INPUT);
  pinMode(tankSwitchPin, INPUT_PULLUP);
  pinMode(overflowPin, INPUT_PULLUP);
  
  // Outputs
  pinMode(relayPin, INPUT); // Start slukket
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  Serial.println("--- System Startet: Med Øjeblikkelig Alarm ---");
  delay(1000);
}

void loop() {
  // 1. TJEK HOVEDTANKEN
  int tankLevel = digitalRead(tankSwitchPin);

  if (tankLevel == 0) { 
    Serial.println("ALARM: Hovedtanken er tom! -> GUL LAMPE");
    pinMode(relayPin, INPUT); 
    
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    delay(1000);
    return; 
  }

  // 2. TJEK OVERLØB I KARSE-SPANDEN
  // Dette tjekker vi i starten af hver runde
  int overflowLevel = digitalRead(overflowPin);
  
  if (overflowLevel == 0) { 
     Serial.println("ALARM: OVERLØB! -> BLINKER RØDT");
     
     pinMode(relayPin, INPUT);
     
     digitalWrite(greenLedPin, LOW);
     digitalWrite(yellowLedPin, LOW);

     // --- BLINK-EFFEKT ---
     digitalWrite(redLedPin, HIGH);
     delay(200); 
     digitalWrite(redLedPin, LOW);
     delay(200); 
     
     return; // Starter forfra med det samme for at blive i alarm-mode
  }

  // 3. TJEK JORDEN (Normal drift)
  int sensorValue = analogRead(sensorPin);
  Serial.print("Status OK. Fugtighed: ");
  Serial.print(sensorValue);

  if (sensorValue > dryThreshold) {
    // --- TØRT -> VANDE ---
    Serial.println(" -> TØRT! Giver vand -> FAST RØDT LYS");
    
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);

    // Start Pumpe
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);
    
    delay(pumpDuration); 

    // Stop Pumpe
    pinMode(relayPin, INPUT);
    Serial.println(" -> PAUSE. Venter (Overvåger overløb)...");
    
    // --- HER ER ÆNDRINGEN: SMART VENTETID ---
    // I stedet for delay(30000), laver vi en tæller
    unsigned long startTime = millis();
    
    // Bliv i denne løkke indtil tiden er gået (soakTime)
    while (millis() - startTime < soakTime) {
      
      // HVERT 0.1 sekund tjekker vi sensoren:
      if (digitalRead(overflowPin) == 0) {
        Serial.println(" -> ALARM! Overløb opdaget midt i pausen! Afbryder.");
        // Vi 'breaker' ud af løkken. 
        // Det betyder, vi skipper resten af ventetiden.
        // Næste gang loop() starter (om få millisekunder), fanger 'Trin 2' alarmen og blinker.
        break; 
      }
      
      delay(100); // Vent et kort øjeblik
    }
    
  } else {
    // --- FUGTIGT -> VENT ---
    Serial.println(" -> Jorden er fugtig nok. -> GRØN LAMPE");
    
    pinMode(relayPin, INPUT);
    
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    
    delay(1000);
  }
}
