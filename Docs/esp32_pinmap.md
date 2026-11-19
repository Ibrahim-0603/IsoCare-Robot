# IsoCare ESP32 Pin Map

**Status:** draft v0.1 (Nov 17, 2026). Verify against the silkscreen of the actual board before wiring.

Assumes a 38-pin ESP32 dev board with an ESP32-WROOM-32 module. On a WROVER module, GPIO 16 and 17 are not available.

## Pin assignments

| Function | GPIO | Dir | Goes to | Notes |
| --- | --- | --- | --- | --- |
| ENC_L_A | 34 | in | Left encoder A | Input-only pin, no internal pull-up |
| ENC_L_B | 35 | in | Left encoder B | Input-only |
| ENC_R_A | 36 | in | Right encoder A | Input-only |
| ENC_R_B | 39 | in | Right encoder B | Input-only |
| MOT_L_IN1 | 25 | out, PWM | L298N IN1 | Left forward |
| MOT_L_IN2 | 26 | out, PWM | L298N IN2 | Left reverse |
| MOT_R_IN3 | 27 | out, PWM | L298N IN3 | Right forward |
| MOT_R_IN4 | 13 | out, PWM | L298N IN4 | Right reverse |
| SERVO_1 | 18 | out, PWM 50 Hz | Drawer 1 servo | 5 V rail power |
| SERVO_2 | 19 | out, PWM 50 Hz | Drawer 2 servo | |
| SERVO_3 | 23 | out, PWM 50 Hz | Drawer 3 servo | Spare if only 2 drawers |
| I2C_SDA | 21 | i/o | MPU6050, MAX30102, MLX90614, MAX30205 | Start at 100 kHz |
| I2C_SCL | 22 | out | Same four devices | |
| TRIG_F | 4 | out | Front HC-SR04 TRIG | |
| ECHO_F | 17 | in | Front HC-SR04 ECHO | Through a 1 kΩ / 2 kΩ divider |
| TRIG_L | 14 | out | Left HC-SR04 TRIG | Pulses at boot, harmless for a trigger |
| ECHO_L | 16 | in | Left HC-SR04 ECHO | Divider |
| TRIG_R | 5 | out | Right HC-SR04 TRIG | Pulses at boot, harmless for a trigger |
| ECHO_R | 12 | in | Right HC-SR04 ECHO | Strapping pin: the echo idles low, which is what boot needs. Divider |
| ESTOP_SENSE | 33 | in, pull-up | E-stop NO contact to GND | Reads low when the E-stop is pressed |
| BATT_SENSE | 32 | ADC1 | Battery divider (100 kΩ top / 27 kΩ bottom) | Plus a 100 nF to GND. About 2.7 V at a full pack. The ADC is nonlinear, so calibrate |

That is 21 of the usable pins.

## Reserved or avoided

| GPIO | Why |
| --- | --- |
| 1, 3 | USB serial (the link to the laptop). Never use |
| 6 to 11 | Connected to the flash chip. Unusable |
| 0 | Boot-mode strapping. Leave free (BOOT button) |
| 2 | Strapping. Free for the on-board status LED |
| 15 | Strapping, prints a boot log. Avoid for anything important |

Nothing else is free. If you need more pins (lid servo, buttons, LED ring), the options are:
- Share one trigger between the left and right ultrasonics (they point in opposite directions). That frees one pin.
- Use GPIO 2 or 15 for a low-stakes output such as a status LED.
- Drop the third ultrasonic or move to I2C time-of-flight sensors.

## I2C device addresses

| Device | Address |
| --- | --- |
| MPU6050 | 0x68 |
| MAX30102 | 0x57 |
| MLX90614 | 0x5A |
| MAX30205 | 0x48 (default) |

No clashes. Breakout boards bring their own pull-up resistors, and four in parallel can add up to a too-strong pull-up. If the bus misbehaves, remove pull-ups until the total is about 2 kΩ or more. If the vitals sensors end up on a puck at the end of a long coiled cable, keep that cable short, stay at 100 kHz, and expect to move the puck to its own bus.

## Electrical rules

- **Motor pins start safe.** Put a 10 kΩ pulldown on each of the four L298N IN pins so the motors stay off while the ESP32 resets or boots. The firmware drives these four low before anything else.
- **Driving the L298N** (ENA/ENB jumpers on):
  - forward: one IN is PWM, the other is 0
  - reverse: swap them
  - brake: both high
  - The right motor is mounted mirrored, so handle the sign in firmware so that positive means forward for both wheels.
- **Encoders** are powered from 3.3 V. Input-only pins have no internal pull-ups, so add 10 kΩ to 3.3 V on all four if the encoder outputs are open-collector.
- **Ultrasonics** are powered from the 5 V rail. Each ECHO goes through 1 kΩ in series and 2 kΩ to ground, which gives about 3.3 V. Fire them one at a time to avoid crosstalk. With a range cap of about 1.5 to 2 m, each sensor refreshes at roughly 20 to 25 Hz, slower than the 50 Hz tick stream.
- **Servos** run from the 5 V rail with a 470 to 1000 µF capacitor at the servos. Common ground with the ESP32.
- **E-stop:** wired between GPIO 33 and GND using the NO contact (internal pull-up on). The NC contact drives the relay, not this pin.

## Firmware notes

- Use the hardware pulse counters (PCNT) for quadrature decoding. Any GPIO can feed them, including the input-only pins.
- Use LEDC for the four motor PWM channels (20 kHz is a good starting point, above hearing) and the three servo channels (50 Hz).
- The default I2C pins (21, 22) are used as-is.
- ADC: no Wi-Fi is used, but BATT_SENSE is on ADC1 anyway, which keeps it usable in any case.
