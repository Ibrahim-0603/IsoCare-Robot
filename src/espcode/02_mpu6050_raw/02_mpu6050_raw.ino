// MPU6050 raw-register gyro test; board must stay still during startup bias sampling.
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

namespace Pins {
constexpr uint8_t SDA = 21;
constexpr uint8_t SCL = 22;
}

namespace Config {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 20;
constexpr uint8_t ADDRESS = 0x68;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t WHO_AM_I_EXPECTED = 0x68;
constexpr uint8_t WHO_AM_I_CLONE_A = 0x70;
constexpr uint8_t WHO_AM_I_CLONE_B = 0x72;
constexpr uint8_t WAKE_WITH_PLL_X_GYRO = 0x01;
constexpr uint8_t GYRO_RANGE_500_DPS = 0x08;
constexpr uint8_t DLPF_42_HZ = 0x03;
constexpr uint8_t SAMPLE_DIVIDER_100_HZ = 9;
constexpr uint32_t SAMPLE_PERIOD_MS = 10;
constexpr uint32_t BIAS_TIME_MS = 2000;
constexpr uint32_t BIAS_POLL_DELAY_MS = 1;
constexpr float LSB_PER_DPS = 65.5F;
}

struct MpuState {
  uint8_t whoAmI = 0;
  bool idRead = false;
  bool configured = false;
  bool biasValid = false;
  bool reportStartup = true;
  int32_t biasRaw = 0;
  int32_t gyroZCentidps = 0;
  float yawDeg = 0.0F;
  uint32_t errors = 0;
  uint32_t lastPrintedErrors = 0;
  uint32_t lastSampleMs = 0;
  bool hasSample = false;
};

MpuState mpu;

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(Config::ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  const bool ok = Wire.endTransmission() == 0;
  if (!ok) {
    ++mpu.errors;
  }
  return ok;
}

bool mpuReadRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(Config::ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    ++mpu.errors;
    return false;
  }
  const uint8_t received = Wire.requestFrom(
      static_cast<uint8_t>(Config::ADDRESS), length, static_cast<uint8_t>(true));
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    ++mpu.errors;
    return false;
  }
  for (uint8_t index = 0; index < length; ++index) {
    data[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool mpuReadGyroZ(int16_t &rawZ) {
  uint8_t bytes[2] = {0, 0};
  if (!mpuReadRegisters(Config::REG_GYRO_ZOUT_H, bytes, 2)) {
    return false;
  }
  rawZ = static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
  return true;
}

bool mpuInit() {
  mpu.idRead = mpuReadRegisters(Config::REG_WHO_AM_I, &mpu.whoAmI, 1);
  if (!mpu.idRead) {
    return false;
  }
  // PWR_MGMT_1=1 wakes the chip and selects the X-gyro PLL clock.
  const bool wakeOk = mpuWriteRegister(Config::REG_PWR_MGMT_1, Config::WAKE_WITH_PLL_X_GYRO);
  // FS_SEL=1 selects +/-500 deg/s; DLPF_CFG=3 gives the approx. 42 Hz filter.
  const bool gyroOk = mpuWriteRegister(Config::REG_GYRO_CONFIG, Config::GYRO_RANGE_500_DPS);
  const bool filterOk = mpuWriteRegister(Config::REG_CONFIG, Config::DLPF_42_HZ);
  // With the DLPF enabled, 1 kHz/(1+9) is a 100 Hz output rate.
  const bool rateOk = mpuWriteRegister(Config::REG_SMPLRT_DIV, Config::SAMPLE_DIVIDER_100_HZ);
  mpu.configured = wakeOk && gyroOk && filterOk && rateOk;
  return mpu.configured;
}

bool mpuCalibrateBias() {
  if (!mpu.configured) {
    return false;
  }
  const uint32_t startMs = millis();
  uint32_t nextSampleMs = startMs;
  int64_t sumRaw = 0;
  uint32_t count = 0;
  while (millis() - startMs < Config::BIAS_TIME_MS) {
    const uint32_t nowMs = millis();
    if (nowMs - nextSampleMs >= Config::SAMPLE_PERIOD_MS) {
      nextSampleMs += Config::SAMPLE_PERIOD_MS;
      int16_t rawZ = 0;
      if (mpuReadGyroZ(rawZ)) {
        sumRaw += rawZ;
        ++count;
      }
    }
    delay(Config::BIAS_POLL_DELAY_MS);
  }
  if (count == 0) {
    return false;
  }
  mpu.biasRaw = static_cast<int32_t>(sumRaw / count);
  mpu.biasValid = true;
  return true;
}

void mpuUpdate() {
  const uint32_t nowMs = millis();
  if (nowMs - mpu.lastSampleMs < Config::SAMPLE_PERIOD_MS) {
    return;
  }
  const uint32_t elapsedMs = nowMs - mpu.lastSampleMs;
  mpu.lastSampleMs = nowMs;

  int16_t rawZ = 0;
  if (!mpu.configured || !mpu.biasValid || !mpuReadGyroZ(rawZ)) {
    return;
  }
  const float correctedDps = static_cast<float>(rawZ - mpu.biasRaw) / Config::LSB_PER_DPS;
  mpu.gyroZCentidps = static_cast<int32_t>(lroundf(correctedDps * 100.0F));
  // Positive MPU Z follows the right-hand rule; with +Z up, CCW yaw is positive.
  mpu.yawDeg += correctedDps * static_cast<float>(elapsedMs) / 1000.0F;
  mpu.hasSample = true;
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  Wire.begin(Pins::SDA, Pins::SCL);
  Wire.setClock(Config::I2C_CLOCK_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  mpuInit();
  mpuCalibrateBias();
  mpu.lastSampleMs = millis();
}

void loop() {
  if (mpu.reportStartup) {
    mpu.reportStartup = false;
    if (!mpu.idRead) {
      Serial.println("ERROR: MPU6050 WHO_AM_I read failed; check I2C wiring.");
    } else {
      Serial.printf("WHO_AM_I=0x%02X\n", mpu.whoAmI);
      if (mpu.whoAmI == Config::WHO_AM_I_CLONE_A ||
          mpu.whoAmI == Config::WHO_AM_I_CLONE_B) {
        Serial.println("WARNING: clone-chip ID; attempting raw gyro test.");
      } else if (mpu.whoAmI != Config::WHO_AM_I_EXPECTED) {
        Serial.println("WARNING: unexpected ID; attempting raw gyro test.");
      }
    }
    Serial.printf("config_ok=%s bias_ok=%s bias_raw=%ld errors=%lu\n",
                  mpu.configured ? "true" : "false",
                  mpu.biasValid ? "true" : "false",
                  static_cast<long>(mpu.biasRaw),
                  static_cast<unsigned long>(mpu.errors));
    mpu.lastPrintedErrors = mpu.errors;
  }
  mpuUpdate();
  if (mpu.errors != mpu.lastPrintedErrors) {
    mpu.lastPrintedErrors = mpu.errors;
    Serial.printf("mpu_i2c_errors=%lu\n", static_cast<unsigned long>(mpu.errors));
  }
  if (mpu.hasSample) {
    mpu.hasSample = false;
    Serial.printf("gyro_z_centi_dps=%ld yaw_deg=%.2f errors=%lu\n",
                  static_cast<long>(mpu.gyroZCentidps), mpu.yawDeg,
                  static_cast<unsigned long>(mpu.errors));
  }
}
