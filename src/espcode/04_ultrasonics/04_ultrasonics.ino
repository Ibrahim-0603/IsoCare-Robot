// Three non-blocking HC-SR04 inputs. Echo capture is done by GPIO interrupts.
#include <Arduino.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

namespace Pins {
constexpr uint8_t FRONT_TRIG = 4;
constexpr uint8_t FRONT_ECHO = 17;
constexpr uint8_t LEFT_TRIG = 14;
constexpr uint8_t LEFT_ECHO = 16;
constexpr uint8_t RIGHT_TRIG = 12;
constexpr uint8_t RIGHT_ECHO = 5;
constexpr uint8_t TRIG[3] = {FRONT_TRIG, LEFT_TRIG, RIGHT_TRIG};
constexpr uint8_t ECHO[3] = {FRONT_ECHO, LEFT_ECHO, RIGHT_ECHO};
}

namespace Config {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t TRIGGER_SLOT_US = 15000;
constexpr uint32_t TRIGGER_PULSE_US = 10;
constexpr uint32_t ECHO_TIMEOUT_US = 12000;
constexpr uint32_t PRINT_INTERVAL_MS = 100;
constexpr uint16_t INVALID_MM = 65535;
constexpr uint32_t SOUND_MM_PER_SECOND = 343000;
}

struct EchoCapture {
  uint8_t echoPin;
  volatile uint32_t riseUs;
  volatile uint32_t durationUs;
  volatile bool rising;
  volatile bool complete;
};

EchoCapture captures[3] = {
  {Pins::FRONT_ECHO, 0, 0, false, false},
  {Pins::LEFT_ECHO, 0, 0, false, false},
  {Pins::RIGHT_ECHO, 0, 0, false, false}
};
portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t distanceMm[3] = {
  Config::INVALID_MM, Config::INVALID_MM, Config::INVALID_MM
};
uint8_t sensorIndex = 2;
uint8_t activeSensor = 0;
uint8_t phase = 0;
uint32_t triggerAtUs = 0;
uint32_t lastTriggerAtUs = 0;

enum Phase : uint8_t {
  IDLE = 0,
  TRIGGER_HIGH = 1,
  WAIT_ECHO = 2
};

void IRAM_ATTR echoChanged(void *argument) {
  EchoCapture *capture = static_cast<EchoCapture *>(argument);
  const uint32_t nowUs = micros();
  portENTER_CRITICAL_ISR(&captureMux);
  if (gpio_get_level(static_cast<gpio_num_t>(capture->echoPin)) != 0) {
    capture->riseUs = nowUs;
    capture->rising = true;
  } else if (capture->rising) {
    capture->durationUs = nowUs - capture->riseUs;
    capture->rising = false;
    capture->complete = true;
  }
  portEXIT_CRITICAL_ISR(&captureMux);
}

void startTrigger(uint8_t index, uint32_t nowUs) {
  activeSensor = index;
  portENTER_CRITICAL(&captureMux);
  captures[index].riseUs = 0;
  captures[index].durationUs = 0;
  captures[index].rising = false;
  captures[index].complete = false;
  portEXIT_CRITICAL(&captureMux);
  digitalWrite(Pins::TRIG[index], HIGH);
  triggerAtUs = nowUs;
  lastTriggerAtUs = nowUs;
  phase = TRIGGER_HIGH;
}

void finishEchoOrTimeout(uint32_t nowUs) {
  uint32_t durationUs = 0;
  bool complete = false;
  portENTER_CRITICAL(&captureMux);
  complete = captures[activeSensor].complete;
  if (complete) {
    durationUs = captures[activeSensor].durationUs;
    captures[activeSensor].complete = false;
  }
  portEXIT_CRITICAL(&captureMux);

  if (complete) {
    if (durationUs > 0 && durationUs <= Config::ECHO_TIMEOUT_US) {
      // Round duration * 343 mm/s / 2, converting microseconds to seconds.
      distanceMm[activeSensor] = static_cast<uint16_t>(
          (durationUs * Config::SOUND_MM_PER_SECOND + 1000000UL) / 2000000UL);
    } else {
      distanceMm[activeSensor] = Config::INVALID_MM;
    }
    phase = IDLE;
  } else if (nowUs - triggerAtUs >= Config::ECHO_TIMEOUT_US) {
    distanceMm[activeSensor] = Config::INVALID_MM;
    phase = IDLE;
  }
}

void setup() {
  for (uint8_t index = 0; index < 3; ++index) {
    pinMode(Pins::TRIG[index], OUTPUT);
    digitalWrite(Pins::TRIG[index], LOW);
    pinMode(Pins::ECHO[index], INPUT);
    attachInterruptArg(Pins::ECHO[index], echoChanged, &captures[index], CHANGE);
  }
  Serial.begin(Config::SERIAL_BAUD);
  lastTriggerAtUs = micros() - Config::TRIGGER_SLOT_US;
}

void loop() {
  const uint32_t nowUs = micros();
  if (phase == TRIGGER_HIGH &&
      nowUs - triggerAtUs >= Config::TRIGGER_PULSE_US) {
    digitalWrite(Pins::TRIG[activeSensor], LOW);
    phase = WAIT_ECHO;
  }
  if (phase == WAIT_ECHO) {
    finishEchoOrTimeout(nowUs);
  }
  if (phase == IDLE && nowUs - lastTriggerAtUs >= Config::TRIGGER_SLOT_US) {
    sensorIndex = static_cast<uint8_t>((sensorIndex + 1) % 3);
    startTrigger(sensorIndex, nowUs);
  }

  static uint32_t lastPrintMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= Config::PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    Serial.printf("front_mm=%u left_mm=%u right_mm=%u (65535=invalid)\n",
                  distanceMm[0], distanceMm[1], distanceMm[2]);
  }
}
