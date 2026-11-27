// Plain I2C address scan for the IsoCare sensor breadboard.
#include <Arduino.h>
#include <Wire.h>

namespace Pins {
constexpr uint8_t SDA = 21;
constexpr uint8_t SCL = 22;
}

namespace Config {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 20;
constexpr uint32_t SCAN_INTERVAL_MS = 5000;
constexpr uint8_t FIRST_ADDRESS = 1;
constexpr uint8_t LAST_ADDRESS = 126;
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  Wire.begin(Pins::SDA, Pins::SCL);
  Wire.setClock(Config::I2C_CLOCK_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
}

void loop() {
  static uint32_t lastScanMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastScanMs < Config::SCAN_INTERVAL_MS) {
    return;
  }
  lastScanMs = nowMs;

  bool foundAny = false;
  Serial.println("I2C scan:");
  for (uint8_t address = Config::FIRST_ADDRESS;
       address <= Config::LAST_ADDRESS; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", address);
      foundAny = true;
    }
  }
  if (!foundAny) {
    Serial.println("  no devices found");
  }
  Serial.println("Scan complete");
}
