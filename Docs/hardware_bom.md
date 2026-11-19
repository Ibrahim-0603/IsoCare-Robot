# IsoCare Hardware Component List

**Status:** draft v0.1 (Nov 17, 2026). Not frozen. Items marked **[confirm]** need a decision or a check before ordering.

Prices are only the local prices seen on Makers Electronics in Oct 2026. A dash means not priced yet.

## Design assumptions

- A Windows laptop (Ryzen 7 5800H, 16 GB) rides on the robot and runs ROS 2 Jazzy and the vision pipeline (Ubuntu in a VM or native). The Pi 5 is dropped for now and stays as the fallback if the laptop test fails.
- The laptop runs on its own battery. The robot's battery does not power it.
- The ESP32 is powered from the laptop's USB port, not from the robot's 5 V rail.
- Estimated robot mass is 6 to 8 kg with the laptop. Design the drive for up to 10 kg. **[confirm by weighing parts]**
- Navigation speed is 0.3 m/s or less.
- Drawers: the servo locks, opens and closes. There are no limit switches.
- The L298N is used because it is the only driver available now.

## 1. Compute

| Item | Qty | Notes | Price (EGP) |
| --- | --- | --- | --- |
| Laptop (owned) | 1 | Ubuntu 24.04 + ROS 2 Jazzy, VM or native | owned |
| ESP32 38-pin dev board (ESP32-WROOM-32) | 1, plus 1 spare | The pin map assumes WROOM, not WROVER | 295 |
| USB data cable, laptop to ESP32 | 1 | Must be a data cable, not charge-only. Short | - |
| USB webcam, fixed focus (C270-class) | 1 | Fixed focus matters: autofocus breaks calibration. Mount low at the front | - |

## 2. Drive

| Item | Qty | Notes | Price (EGP) |
| --- | --- | --- | --- |
| JGB37-520 gear motor with encoder, 12 V, 110 RPM | 2 | Rated 3.9 kg·cm, stall 2.4 A. Comes with bracket. **[confirm]** shaft size (usually 6 mm D) and encoder counts per output revolution | 675 each |
| Wheels, about 100 mm, matching the shaft | 2 | If torque is marginal, 65 mm wheels give about 1.5x the push | - |
| Ball caster | 2 | One front, one rear for stability with the laptop | - |
| L298N dual H-bridge module | 1 | Add a heatsink. See the wiring notes below | - |
| Ceramic capacitor 100 nF | 4 | One across each motor's terminals, reduces encoder noise | - |

L298N notes:
- Leave the ENA/ENB jumpers on. PWM goes on the IN pins.
- Remove the 5V-enable jumper (a full pack is 12.6 V, just over what its onboard regulator is meant for) and feed the +5V pin from the buck.
- It drops about 2 V, so the motors see about 10 V. It is rated 2 A per channel against a 2.4 A stall, so the firmware must cut power fast when PWM is high and the encoders do not move.

## 3. Power and safety

| Item | Qty | Notes | Price (EGP) |
| --- | --- | --- | --- |
| Li-ion pack 3S2P, 5100 mAh, with BMS | 1 | Genuine-cell pack | - |
| Charger 3S, 12.6 V, 2 A | 1 | | - |
| Inline fuse holder + 10 A blade fuse (plus spare) | 1 | Directly after the battery | - |
| XT60 connector pairs | 2 | Main battery and main bus path. Use 5.5x2.1 barrels only for charging (they are usually rated 3 to 5 A) | - |
| KCD4 rocker switch, 16 A | 1 | Main power switch | - |
| AB6-A emergency stop (1 NC + 1 NO) | 1 | **[confirm]** panel hole size | - |
| 12 V automotive relay, 10 A or more (or a relay module rated 10 A+) | 1 | Switches only the L298N motor supply | - |
| Diode 1N4007 | 2 | Flyback across the relay coil (skip if the module has one) | - |
| XL4015 5 A buck converter | 1 | One only. Set to 5.0 to 5.1 V with a multimeter BEFORE connecting anything | - |
| Electrolytic capacitor 470 to 1000 µF, 16 V | 2 | Across the 5 V rail at the servos | - |
| Resistors, 1/4 W | kit | 100 kΩ + 27 kΩ (battery divider); 1 kΩ + 2 kΩ x3 pairs (ultrasonic echo dividers); 10 kΩ x4 (L298N IN pulldowns); 10 kΩ x4 (encoder pull-ups, only if the encoder outputs are open-collector) | - |
| Ceramic capacitor 100 nF | 1 | On the battery-sense ADC pin | - |
| Terminal blocks / Wago connectors | about 6 | Power distribution and common ground | - |
| GX16-2 chassis connector pair | 1 | Optional charge port | - |
| Wire 18 AWG (motor and battery path), 22 to 24 AWG (signals), heat-shrink, crimp/ring terminals, cable ties | - | | - |

## 4. Sensors

| Item | Qty | Notes | Price (EGP) |
| --- | --- | --- | --- |
| MPU6050 (GY-521) | 1 | Gyro z for heading | - |
| HC-SR04 ultrasonic | 3 | Front, left, right. 5 V supply. The echo output needs a divider to 3.3 V | - |
| MAX30102 | 1 | HR + SpO2 | 90 |
| MLX90614 (GY-906, 3.3 V module) | 1 | Forehead temperature (primary) | - |
| MAX30205 | 1 | Contact temperature (cross-check) | - |
| Encoders | - | Built into the motors. Power them from 3.3 V so their outputs are safe for the ESP32 | - |

## 5. Drawers

| Item | Qty | Notes | Price (EGP) |
| --- | --- | --- | --- |
| MG90S micro servo, 180° positional, metal gear | 3 | Buy the 180° version, never the 360° continuous one. One spare is wise | 135 each |
| Cabinet with 2 or 3 drawers and a servo linkage | 1 | Belal designs it. The closed position must be mechanically blocked (hook or over-center), not held only by servo torque | - |

If the linkage needs more force to open a drawer, the MG996R is the next step up. It is bigger and draws about 2 A at stall, which changes the 5 V rail budget.

## 6. Mechanical

- Chassis base plate, standoffs and M3 hardware.
- Laptop tray with straps, mounted low and central, with airflow.
- Mounts for the three ultrasonics and the webcam (front, low).
- Cable strain relief.

## 7. Dropped from the earlier draft

Pi 5, its microSD, its cooler and its XL4015 (laptop replaces it), LiDAR (navigation uses markers), solenoids (servos chosen), limit switches (servo does open/close), GA25-370 motors (35 RPM is too slow for 0.3 m/s), DRV8871 (not available), touchscreen (nurse page on a phone).

**Undecided, not on the list:** patient puck and its coiled cable, Accept/Cancel buttons, LED ring, speaker. They would need ESP32 pins, and almost none are free (see esp32_pinmap.md).

## 8. Power architecture

```
Battery 12.6 V -> fuse -> KCD4 switch -+-> E-stop NC -> relay coil (+ 1N4007) -> GND
                                       +-> relay contacts -> L298N motor supply (VS)
                                       +-> XL4015 -> 5 V rail -> servos, HC-SR04 x3, L298N +5V

Laptop USB -> ESP32 -> 3V3 pin -> MPU6050, MAX30102, MLX90614, MAX30205, encoders
All grounds tied together: battery -, buck -, L298N GND, ESP32 GND
```

- The E-stop cuts only the motor supply. The laptop, ESP32 and servo rail stay on, so the ESP32 can report `ESTOP`.
- The E-stop's NO contact goes to an ESP32 pin to sense it. A broken sense wire is not safety-critical, because the NC contact cuts motor power anyway.
- The relay re-closes as soon as the E-stop is released. So the firmware must keep the motors at zero after an E-stop until the Pi clears the fault (SYSTEM_CMD "clear fault latch").
- Do not connect the 5 V rail to the ESP32's 5V/VIN pin while it is on USB. Two supplies get tied together.
- The ESP32's 3V3 pin has plenty of margin for the sensors and encoders (roughly 100 mA total, estimated). **[verify with a meter]**

## 9. Open items

1. Confirm the motor shaft size, encoder counts per revolution and wheel hubs.
2. Weigh the parts as they arrive. If the robot goes over about 10 kg, change wheels or gearing.
3. Test the laptop path (USB passthrough, webcam, YOLO speed on battery power) before dropping the Pi for good.
4. Decide whether the patient puck, buttons, LED ring and speaker are in scope.
5. Test the ultrasonics against a real bed with a blanket. If unreliable, add one VL53L1X (I2C) at the front.

## 10. Documents that need updating because of these decisions

- `protocol.md`: with no limit switches, DRAWER_STATUS `open_mask` becomes the commanded position plus a timer, not a sensed state. Range sensors are three ultrasonics (they refresh at a slower rate than the ticks).
- Mission manager: "DISPENSE confirmed by limit switch" becomes confirmed by command and timer.
- `README.md`: it still says Raspberry Pi 5.
