// MAX30102 test. Install SparkFun MAX3010x Sensor Library in Arduino IDE first.
// The Maxim algorithm is hobby-grade only; these results are not medical measurements.
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

namespace Pins {
constexpr uint8_t SDA = 21;
constexpr uint8_t SCL = 22;
}

namespace Config {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 20;
constexpr uint8_t ADDRESS = 0x57;
constexpr uint8_t PART_ID_REGISTER = 0xFF;
constexpr uint8_t EXPECTED_PART_ID = 0x15;
constexpr uint8_t LED_BRIGHTNESS = 0x1F;
constexpr uint8_t SAMPLE_AVERAGE = 1;
constexpr uint8_t LED_MODE_RED_AND_IR = 2;
constexpr uint16_t SENSOR_RATE_HZ = 100;
constexpr uint16_t ALGORITHM_RATE_HZ = 25;
constexpr uint16_t PULSE_WIDTH_US = 411;
constexpr uint16_t ADC_RANGE = 4096;
constexpr uint32_t FINGER_IR_THRESHOLD = 50000;
constexpr uint16_t WINDOW_SIZE = 100;
constexpr uint16_t RECOMPUTE_SAMPLES = 25;
constexpr uint32_t POLL_INTERVAL_MS = 1;
constexpr uint32_t PRINT_INTERVAL_MS = 10;
}

MAX30105 sensor;

struct MaxState {
  bool ready = false;
  bool deviceFound = false;
  bool reportStartup = true;
  bool finger = false;
  bool beat = false;
  bool hrValid = false;
  bool spo2Valid = false;
  uint8_t partId = 0;
  uint32_t ir = 0;
  uint32_t red = 0;
  uint32_t errors = 0;
  int32_t hrBpm = 0;
  int32_t spo2Percent = 0;
  uint32_t sensorSamples = 0;
  uint16_t windowCount = 0;
  uint16_t windowWrite = 0;
  uint16_t samplesSinceCalculation = 0;
  uint32_t lastPollMs = 0;
  uint32_t lastPrintMs = 0;
};

MaxState maxState;
uint32_t irWindow[Config::WINDOW_SIZE] = {};
uint32_t redWindow[Config::WINDOW_SIZE] = {};
uint32_t orderedIr[Config::WINDOW_SIZE] = {};
uint32_t orderedRed[Config::WINDOW_SIZE] = {};

bool maxInit() {
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD, Config::ADDRESS)) {
    ++maxState.errors;
    return false;
  }
  maxState.deviceFound = true;
  maxState.partId = sensor.readRegister8(Config::ADDRESS, Config::PART_ID_REGISTER);
  if (maxState.partId != Config::EXPECTED_PART_ID) {
    ++maxState.errors;
    return false;
  }
  // Average=1 preserves 100 raw samples/s. The Maxim window is downsampled to 25 Hz.
  sensor.setup(Config::LED_BRIGHTNESS, Config::SAMPLE_AVERAGE,
               Config::LED_MODE_RED_AND_IR, Config::SENSOR_RATE_HZ,
               Config::PULSE_WIDTH_US, Config::ADC_RANGE);
  maxState.ready = true;
  return true;
}

void maxResetAlgorithmWindow() {
  maxState.windowCount = 0;
  maxState.windowWrite = 0;
  maxState.samplesSinceCalculation = 0;
  maxState.hrBpm = 0;
  maxState.spo2Percent = 0;
  maxState.hrValid = false;
  maxState.spo2Valid = false;
}

void maxCalculateVitals() {
  if (maxState.windowCount < Config::WINDOW_SIZE ||
      maxState.samplesSinceCalculation < Config::RECOMPUTE_SAMPLES) {
    return;
  }
  maxState.samplesSinceCalculation = 0;
  for (uint16_t index = 0; index < Config::WINDOW_SIZE; ++index) {
    const uint16_t sourceIndex =
        static_cast<uint16_t>((maxState.windowWrite + index) % Config::WINDOW_SIZE);
    orderedIr[index] = irWindow[sourceIndex];
    orderedRed[index] = redWindow[sourceIndex];
  }

  int32_t algorithmHr = 0;
  int32_t algorithmSpo2 = 0;
  int8_t validHr = 0;
  int8_t validSpo2 = 0;
  maxim_heart_rate_and_oxygen_saturation(
      orderedIr, Config::WINDOW_SIZE, orderedRed, &algorithmSpo2, &validSpo2,
      &algorithmHr, &validHr);
  maxState.hrValid = validHr != 0 && algorithmHr >= 30 && algorithmHr <= 240;
  maxState.spo2Valid = validSpo2 != 0 && algorithmSpo2 >= 70 && algorithmSpo2 <= 100;
  maxState.hrBpm = maxState.hrValid ? algorithmHr : 0;
  maxState.spo2Percent = maxState.spo2Valid ? algorithmSpo2 : 0;
}

void maxStoreSample(uint32_t ir, uint32_t red) {
  maxState.ir = ir;
  maxState.red = red;
  maxState.finger = ir >= Config::FINGER_IR_THRESHOLD;
  maxState.beat = checkForBeat(static_cast<int32_t>(ir));
  ++maxState.sensorSamples;
  if (!maxState.finger) {
    maxResetAlgorithmWindow();
    return;
  }

  // The Maxim routine assumes 25 samples/s. Keep one of every four 100 Hz samples.
  if ((maxState.sensorSamples % (Config::SENSOR_RATE_HZ /
                                 Config::ALGORITHM_RATE_HZ)) == 0) {
    irWindow[maxState.windowWrite] = ir;
    redWindow[maxState.windowWrite] = red;
    maxState.windowWrite =
        static_cast<uint16_t>((maxState.windowWrite + 1) % Config::WINDOW_SIZE);
    if (maxState.windowCount < Config::WINDOW_SIZE) {
      ++maxState.windowCount;
    }
    ++maxState.samplesSinceCalculation;
    maxCalculateVitals();
  }
}

void maxUpdate() {
  const uint32_t nowMs = millis();
  if (!maxState.ready || nowMs - maxState.lastPollMs < Config::POLL_INTERVAL_MS) {
    return;
  }
  maxState.lastPollMs = nowMs;
  sensor.check();
  while (sensor.available()) {
    const uint32_t red = sensor.getFIFORed();
    const uint32_t ir = sensor.getFIFOIR();
    sensor.nextSample();
    maxStoreSample(ir, red);
  }
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  Wire.begin(Pins::SDA, Pins::SCL);
  Wire.setClock(Config::I2C_CLOCK_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  maxInit();
}

void loop() {
  if (maxState.reportStartup) {
    maxState.reportStartup = false;
    if (maxState.ready) {
      Serial.printf("MAX30102 ready; part_id=0x%02X sample_rate=%u Hz\n",
                    maxState.partId, Config::SENSOR_RATE_HZ);
      Serial.println("Place a finger steadily on the sensor; allow about 4 s to fill window.");
    } else if (maxState.deviceFound) {
      Serial.printf("ERROR: MAX30102 part ID 0x%02X; expected 0x15. Check sensor/wiring.\n",
                    maxState.partId);
    } else {
      Serial.println("ERROR: MAX30102 not found at 0x57; check power and I2C wiring.");
    }
  }

  maxUpdate();
  const uint32_t nowMs = millis();
  if (maxState.ready && nowMs - maxState.lastPrintMs >= Config::PRINT_INTERVAL_MS) {
    maxState.lastPrintMs = nowMs;
    Serial.printf("ir=%lu red=%lu finger=%s beat=%s hr_bpm=%ld hr_valid=%s "
                  "spo2_percent=%ld spo2_valid=%s\n",
                  static_cast<unsigned long>(maxState.ir),
                  static_cast<unsigned long>(maxState.red),
                  maxState.finger ? "true" : "false",
                  maxState.beat ? "true" : "false",
                  static_cast<long>(maxState.hrBpm),
                  maxState.hrValid ? "true" : "false",
                  static_cast<long>(maxState.spo2Percent),
                  maxState.spo2Valid ? "true" : "false");
  }
}
