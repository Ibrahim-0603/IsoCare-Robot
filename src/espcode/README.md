# IsoCare ESP32 sensor-test sketches

Open each `.ino` from its own folder in Arduino IDE and select the uPesy ESP32 Wroom DevKit.
Use Arduino IDE's ESP32 by Espressif Systems core 3.3.x and the Serial Monitor at 115200 baud.
The sketches are independent breadboard tests; none contains a protocol, ROS, network, or
motor-driving implementation.

## Expected results

1. `01_i2c_scanner`: every five seconds it prints the addresses it found. With only the MPU6050
   and MAX30102 connected, expect `0x57` and `0x68`. Other connected I2C devices will also appear.
2. `02_mpu6050_raw`: expects `WHO_AM_I=0x68`; `0x70` or `0x72` produces a clone warning. Keep the
   board still for its two-second startup bias sample. Then expect `gyro_z_centi_dps` near zero
   while still, and `yaw_deg` to increase for counter-clockwise rotation (with the sensor's +Z
   axis pointing up). Rotate by hand for a rough 360-degree integration check; drift is expected.
3. `03_max30102`: requires the SparkFun **MAX3010x Sensor Library** (`MAX30105.h`,
   `heartRate.h`, and `spo2_algorithm.h`). It should report part ID `0x15` and stream raw IR/red,
   finger/beat flags, and `hr_bpm` / `spo2_percent` with separate validity flags. The pulse-ox
   window needs about four seconds of finger samples before it can return valid results. IR
   threshold 50000 is a common starting point from the SparkFun example; tune it for the actual
   breakout and finger placement. Hobby sensors are not suitable for diagnosis.
4. `04_ultrasonics`: prints front/left/right millimetres about ten times per second. `65535`
   means no valid echo/timeout. Check one sensor at a time first; at 30 cm and 100 cm expect
   approximately 300 mm and 1000 mm, with real HC-SR04 variation.
5. `05_l298n_logic`: prints each GPIO and holds it high for two seconds, then tests 25%, 50%, and
   75% PWM at 5 kHz. Only one L298N input is active at a time. Keep motors disconnected and the
   motor supply off; use the pulldowns and verify with a scope or suitable meter.

All I2C sketches use SDA 21, SCL 22, 100 kHz, and a 20 ms Wire timeout. The ultrasonic sketch
uses the fixed TRIG/ECHO mapping and keeps each sensor's trigger at least 15 ms apart.
