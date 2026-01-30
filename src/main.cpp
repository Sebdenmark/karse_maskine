#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- 1. WIFI & MQTT INDSTILLINGER ---
const char* ssid = "IoT_H3/4";       
const char* password = "98806829";   

// HER ER DIN RASPBERRY PI IP INDSAT:
const char* mqtt_server = "192.168.0.152"; 

// --- KONFIGURATION AF PORTE ---
const int sensorPin = 35;       
const int relayPin = 4;         
const int tankSwitchPin = 32;   
const int overflowPin = 33;     

// Lamper
const int greenLedPin = 21;     
const int yellowLedPin = 22;    
const int redLedPin = 19;       

// --- INDSTILLINGER ---
const int dryThreshold = 2400;  // Grænse for tør jord
const int pumpDuration = 1000;  // 1 sekund
const int soakTime = 30000;     // 30 sekunder

// --- DATA VARIABLER ---
int waterCount = 0; 

WiFiClient espClient;
PubSubClient client(espClient);

// --- WIFI FORBINDELSE ---
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Forbinder til WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi forbundet!");
  Serial.print("ESP32 IP-adresse: ");
  Serial.println(WiFi.localIP());
}

// --- MQTT FORBINDELSE ---
void reconnect() {
  while (!client.connected()) {
    Serial.print("Forsøger MQTT forbindelse til ");
    Serial.print(mqtt_server);
    Serial.print("...");
    
    // Vi bruger et unikt ID
    if (client.connect("ESP32_Karse_Vander")) { 
      Serial.println("forbundet!");
    } else {
      Serial.print("fejlede, rc=");
      Serial.print(client.state());
      Serial.println(" prøver igen om 5 sek");
      delay(5000);
    }
  }
}

// --- SEND DATA ---
void sendData(String statusMsg, int tankTom, int overloeb) {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int fugt = analogRead(sensorPin);

  // JSON format
  String payload = "{";
  payload += "\"fugt\": " + String(fugt) + ",";
  payload += "\"msg\": \"" + statusMsg + "\",";
  payload += "\"tank_tom\": " + String(tankTom) + ",";
  payload += "\"overloeb\": " + String(overloeb) + ",";
  payload += "\"count\": " + String(waterCount);
  payload += "}";

  client.publish("karse/data", payload.c_str());
  Serial.println("DATA SENDT: " + payload);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(sensorPin, INPUT);
  pinMode(tankSwitchPin, INPUT_PULLUP);
  pinMode(overflowPin, INPUT_PULLUP);
  pinMode(relayPin, INPUT); 
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  Serial.println("--- System Startet ---");
  delay(1000);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 1. TJEK HOVEDTANKEN
  int tankLevel = digitalRead(tankSwitchPin);

  if (tankLevel == 0) { 
    Serial.println("ALARM: Hovedtanken er tom!");
    pinMode(relayPin, INPUT); 
    
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    
    sendData("TankTomAlarm", 1, 0);
    
    delay(1000);
    return; 
  }

  // 2. TJEK OVERLØB
  int overflowLevel = digitalRead(overflowPin);
  
  if (overflowLevel == 0) { 
     Serial.println("ALARM: OVERLØB!");
     pinMode(relayPin, INPUT);
     digitalWrite(greenLedPin, LOW);
     digitalWrite(yellowLedPin, LOW);

     digitalWrite(redLedPin, HIGH);
     delay(200); 
     digitalWrite(redLedPin, LOW);
     
     sendData("OverloebAlarm", 0, 1);
     
     delay(200); 
     return; 
  }

  // 3. TJEK JORDEN
  int sensorValue = analogRead(sensorPin);

  if (sensorValue > dryThreshold) {
    // --- TØRT -> VANDE ---
    Serial.println(" -> TØRT! Giver vand");
    
    waterCount++;
    sendData("Vander", 0, 0);
    
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);

    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);
    delay(pumpDuration); 

    pinMode(relayPin, INPUT);
    Serial.println(" -> PAUSE.");
    
    unsigned long startTime = millis();
    while (millis() - startTime < soakTime) {
      client.loop(); 
      if (digitalRead(overflowPin) == 0) {
        sendData("Overloeb_Pause", 0, 1);
        break; 
      }
      delay(100); 
    }
    
  } else {
    // --- STATUS OK ---
    Serial.println(" -> Status OK.");
    
    sendData("Alt_OK", 0, 0);

    pinMode(relayPin, INPUT);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    
    unsigned long okWait = millis();
    while (millis() - okWait < 30000) { 
       client.loop(); 
       if (digitalRead(overflowPin) == 0 || digitalRead(tankSwitchPin) == 0) break;
       delay(100);
    }
  }
}

