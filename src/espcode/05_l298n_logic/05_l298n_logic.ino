// L298N input-logic test only. Keep the motor supply OFF and motors disconnected.
#include <Arduino.h>

namespace Pins {
constexpr uint8_t IN1 = 25;
constexpr uint8_t IN2 = 26;
constexpr uint8_t IN3 = 27;
constexpr uint8_t IN4 = 13;
constexpr uint8_t INPUTS[4] = {IN1, IN2, IN3, IN4};
}

namespace Config {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PWM_FREQUENCY_HZ = 5000;
constexpr uint8_t PWM_RESOLUTION_BITS = 10;
constexpr uint16_t PWM_DUTY[3] = {256, 512, 768};
constexpr uint8_t PWM_PERCENT[3] = {25, 50, 75};
constexpr uint32_t STEP_DURATION_MS = 2000;
constexpr uint8_t INPUT_COUNT = 4;
constexpr uint8_t DUTY_COUNT = 3;
}

uint8_t digitalIndex = 0;
uint8_t pwmIndex = 0;
uint8_t pwmDutyIndex = 0;
uint32_t stepStartedMs = 0;
bool stepActive = false;
bool pwmStage = false;
bool testComplete = false;
bool pwmAttached[Config::INPUT_COUNT] = {false, false, false, false};

void allInputsLow() {
  for (uint8_t index = 0; index < Config::INPUT_COUNT; ++index) {
    if (pwmStage && pwmAttached[index]) {
      ledcWrite(Pins::INPUTS[index], 0);
    } else {
      digitalWrite(Pins::INPUTS[index], LOW);
    }
  }
}

void setup() {
  // Set output latches LOW before changing pin modes; external pulldowns are a second safeguard.
  for (uint8_t index = 0; index < Config::INPUT_COUNT; ++index) {
    digitalWrite(Pins::INPUTS[index], LOW);
  }
  for (uint8_t index = 0; index < Config::INPUT_COUNT; ++index) {
    pinMode(Pins::INPUTS[index], OUTPUT);
    digitalWrite(Pins::INPUTS[index], LOW);
  }
  Serial.begin(Config::SERIAL_BAUD);
}

void loop() {
  if (testComplete) {
    return;
  }
  const uint32_t nowMs = millis();
  if (!pwmStage) {
    if (!stepActive) {
      allInputsLow();
      digitalWrite(Pins::INPUTS[digitalIndex], HIGH);
      Serial.printf("DIGITAL HIGH: GPIO %u for 2 s; all other inputs LOW\n",
                    Pins::INPUTS[digitalIndex]);
      stepStartedMs = nowMs;
      stepActive = true;
    } else if (nowMs - stepStartedMs >= Config::STEP_DURATION_MS) {
      digitalWrite(Pins::INPUTS[digitalIndex], LOW);
      stepActive = false;
      ++digitalIndex;
      if (digitalIndex >= Config::INPUT_COUNT) {
        pwmStage = true;
        for (uint8_t index = 0; index < Config::INPUT_COUNT; ++index) {
          pwmAttached[index] = ledcAttach(
              Pins::INPUTS[index], Config::PWM_FREQUENCY_HZ,
              Config::PWM_RESOLUTION_BITS);
          if (pwmAttached[index]) {
            ledcWrite(Pins::INPUTS[index], 0);
          }
        }
        Serial.println("Starting 5 kHz, 10-bit PWM test.");
      }
    }
    return;
  }

  if (pwmIndex >= Config::INPUT_COUNT) {
    allInputsLow();
    Serial.println("Test complete; all L298N inputs LOW.");
    testComplete = true;
    return;
  }

  if (!stepActive) {
    if (!pwmAttached[pwmIndex]) {
      Serial.printf("ERROR: LEDC attach failed for GPIO %u; skipping it.\n",
                    Pins::INPUTS[pwmIndex]);
      ++pwmIndex;
    } else {
      allInputsLow();
      ledcWrite(Pins::INPUTS[pwmIndex], Config::PWM_DUTY[pwmDutyIndex]);
      Serial.printf("PWM GPIO %u: %u%% duty (%u/1023) for 2 s\n",
                    Pins::INPUTS[pwmIndex], Config::PWM_PERCENT[pwmDutyIndex],
                    Config::PWM_DUTY[pwmDutyIndex]);
      stepStartedMs = nowMs;
      stepActive = true;
    }
  } else if (nowMs - stepStartedMs >= Config::STEP_DURATION_MS) {
    ledcWrite(Pins::INPUTS[pwmIndex], 0);
    stepActive = false;
    ++pwmDutyIndex;
    if (pwmDutyIndex >= Config::DUTY_COUNT) {
      pwmDutyIndex = 0;
      ++pwmIndex;
    }
    if (pwmIndex >= Config::INPUT_COUNT) {
      allInputsLow();
    }
  }
}
