#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

BluetoothSerial SerialBT;

const int led1Pin = 4;
const int led2Pin = 5;
const int potPin = 34;   // Potentiometer on GPIO34 (ADC1)

int led1Value = 0;
int led2Value = 0;
bool autoMode = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_mamamo"); // Bluetooth name
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
}

void loop() {
  // Receive data from phone
  if (SerialBT.available()) {
    String msg = SerialBT.readStringUntil('\n');  
    msg.trim();

    if (msg.length() > 0) {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, msg);

      if (!error) {
        if (doc.containsKey("LED1")) led1Value = doc["LED1"];
        if (doc.containsKey("LED2")) led2Value = doc["LED2"];
        if (doc.containsKey("Mode")) autoMode = (String(doc["Mode"]) == "Auto");
      }
    }
  }

  // Handle Auto Mode with potentiometer
  if (autoMode) {
    int potValue = analogRead(potPin);               // 0–4095
    int brightness = map(potValue, 0, 4095, 0, 255); // Scale to 0–255
    led1Value = brightness;
    led2Value = 255 - brightness;
  }

  // Update LEDs
  analogWrite(led1Pin, led1Value);
  analogWrite(led2Pin, led2Value);

  // Send status every 500ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 500) {
    lastSend = millis();
    StaticJsonDocument<200> doc;
    doc["LED1"] = led1Value;
    doc["LED2"] = led2Value;
    doc["Mode"] = autoMode ? "Auto" : "Manual";
    doc["Pot"]  = analogRead(potPin); // also send raw pot value for debug

    serializeJson(doc, SerialBT);
    SerialBT.print("\n");   // newline delimiter
  }
}