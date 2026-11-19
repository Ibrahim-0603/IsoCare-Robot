#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("ESP32 Arduino ready");
  delay(1000);
}