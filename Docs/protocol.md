# IsoCare Serial Protocol

## TL;DR: This is how the ROS and ESP will communicate and send messages to each other; What the messages sent will look like

**Status:** draft v0.1 (Oct 21, 2026). Not frozen. Anything marked **[decide]** needs a team decision before firmware and `serial_bridge` are written against it.

This is the single contract between the Raspberry Pi 5 (`serial_bridge` node) and the ESP32 (firmware). If a byte layout changes, change this file, the firmware parser and `serial_bridge` in the same commit.

It reuses the framing from the Sentinel protocol. The type IDs and payloads are new.

## 1. Link

| Item | Value |
| --- | --- |
| Physical | USB cable, Pi to ESP32 dev board (USB-UART bridge on the board) |
| Settings | 115200 baud, 8N1, no flow control (**[decide]** raise to 460800 later if needed; estimated load is about 1.5 KB/s, roughly 13% of 115200) |
| Byte order | Big-endian for all multi-byte fields |
| Signed values | Two's complement |

## 2. Frame format

```
 +------+------+------+---------------+-----+
 | SYNC | TYPE | LEN  | PAYLOAD       | CHK |
 | 0xAA | 1 B  | 1 B  | LEN bytes     | 1 B |
 +------+------+------+---------------+-----+
```

- `SYNC`: always `0xAA`.
- `TYPE`: message ID (section 4).
- `LEN`: payload length in bytes (0 to 32). A `LEN` above 32 means a corrupt frame.
- `CHK`: XOR of every byte from `TYPE` through the last payload byte (so `LEN` is included, `SYNC` is not).
- Total frame size is `LEN + 4` bytes.

### Parser rules (both sides)

1. Discard bytes until `0xAA` is seen.
2. Read `TYPE`, `LEN`. If `LEN` > 32, drop the sync byte and rescan.
3. Read `LEN` payload bytes and `CHK`.
4. If `CHK` is wrong, **do not skip the whole frame**. Throw away only the sync byte and rescan from the next byte, because `0xAA` can legitimately appear inside a payload.
5. If more than 50 ms passes between bytes of one frame, drop the partial frame and go back to step 1.
6. An unknown `TYPE` with a valid checksum is answered with an ACK status `UNKNOWN_TYPE` (section 6) and otherwise ignored.

## 3. Directions and ID ranges

IDs are split by direction so a captured frame can be read at a glance.

| Range | Direction |
| --- | --- |
| `0x01` to `0x7F` | Pi to ESP32 (commands) |
| `0x80` to `0xFF` | ESP32 to Pi (data and replies) |

## 4. Message summary

| ID | Name | Direction | LEN | Rate | ACKed |
| --- | --- | --- | --- | --- | --- |
| `0x01` | DRIVE_CMD | Pi to ESP | 4 | 20 Hz, continuous | No |
| `0x02` | DRAWER_CMD | Pi to ESP | 3 | On event | Yes |
| `0x03` | MEASURE_CMD | Pi to ESP | 2 | On event | Yes |
| `0x04` | SYSTEM_CMD | Pi to ESP | 1 | On event | Yes |
| `0x81` | ACK | ESP to Pi | 2 | Reply | n/a |
| `0x82` | SENSOR_DATA | ESP to Pi | 20 | 50 Hz | n/a |
| `0x83` | VITALS | ESP to Pi | 7 | 1 Hz while measuring | n/a |
| `0x84` | DRAWER_STATUS | ESP to Pi | 2 | On change, and 1 Hz | n/a |
| `0x85` | STATUS | ESP to Pi | 4 | On change, and 2 Hz | n/a |

## 5. Message definitions

### 0x01 DRIVE_CMD (Pi to ESP), LEN 4

| Offset | Field | Type | Unit | Notes |
| --- | --- | --- | --- | --- |
| 0 | left_speed | int16 | mm/s | Positive = wheel rolls forward |
| 2 | right_speed | int16 | mm/s | Positive = wheel rolls forward |

- `serial_bridge` converts `/cmd_vel` (Twist) into wheel speeds using the wheelbase, so the ESP32 never needs the robot's geometry.
- The ESP32 runs a closed speed loop per wheel (encoder feedback, PID).
- **This is also the watchdog.** It must be sent at 20 Hz at all times, including `0, 0` when stopped. If no valid `DRIVE_CMD` arrives for 0.5 s, the ESP32 stops the motors and sets the `WATCHDOG` flag in STATUS. The next valid `DRIVE_CMD` clears it.
- Never ACKed: it is periodic, and a lost one is replaced by the next.
- Local safety overrides the command (section 7).

### 0x02 DRAWER_CMD (Pi to ESP), LEN 3

| Offset | Field | Type | Notes |
| --- | --- | --- | --- |
| 0 | drawer_id | uint8 | 0 to 2 = drawers, 3 = lid |
| 1 | action | uint8 | 0 = lock, 1 = unlock |
| 2 | relock_s | uint8 | Auto-relock after this many seconds once the drawer is closed; 0 = stay unlocked until commanded |

- Commands set a state, they do not toggle. Resending the same command is harmless, which makes retries safe.
- Refused with `REFUSED_SAFETY` if a sensor fault or E-stop is active ("never dispense on a sensor error").

### 0x03 MEASURE_CMD (Pi to ESP), LEN 2

| Offset | Field | Type | Notes |
| --- | --- | --- | --- |
| 0 | action | uint8 | 0 = stop, 1 = start |
| 1 | sensor_mask | uint8 | bit0 = MAX30102 (HR/SpO2), bit1 = MLX90614, bit2 = MAX30205 |

- While started, the ESP32 sends VITALS at 1 Hz and keeps going until stopped. Deciding when a reading is stable enough is the Pi's job (`decision_engine`).
- Sensors not in the mask are left idle (saves power, avoids I2C noise).

### 0x04 SYSTEM_CMD (Pi to ESP), LEN 1

| Value | Command | Notes |
| --- | --- | --- |
| `0x01` | Recalibrate gyro | Robot must be stationary for about 2 s; STATUS shows `GYRO_CAL` while running |
| `0x02` | Clear fault latch | Only works if the E-stop is released and the cause is gone, otherwise `REFUSED_SAFETY` |
| `0x03` | Soft reset | ESP32 reboots; cumulative tick counters restart from 0 |

### 0x81 ACK (ESP to Pi), LEN 2

| Offset | Field | Type | Notes |
| --- | --- | --- | --- |
| 0 | acked_type | uint8 | The `TYPE` of the command being answered |
| 1 | status | uint8 | See below |

| Status | Name | Meaning |
| --- | --- | --- |
| `0x00` | OK | Accepted and executed |
| `0x01` | BAD_PAYLOAD | Wrong `LEN` or a value out of range |
| `0x02` | UNKNOWN_TYPE | `TYPE` not recognised |
| `0x03` | REFUSED_SAFETY | E-stop, latched fault, or sensor error blocks this command |
| `0x04` | BUSY | Cannot accept right now (for example gyro calibration running) |

- A frame with a bad checksum gets **no reply**, since its contents cannot be trusted. The Pi detects this by timeout (section 6).
- `DRIVE_CMD` is never ACKed.

### 0x82 SENSOR_DATA (ESP to Pi), LEN 20 ("navigation data")

| Offset | Field | Type | Unit | Notes |
| --- | --- | --- | --- | --- |
| 0 | left_ticks | int32 | ticks | Cumulative since boot, positive = forward |
| 4 | right_ticks | int32 | ticks | Cumulative since boot, positive = forward |
| 8 | t_ms | uint32 | ms | ESP32 uptime when the sample was taken |
| 12 | gyro_z | int16 | 0.01 deg/s | Bias-corrected, positive = counter-clockwise (ROS convention) |
| 14 | range_front | uint16 | mm | `0xFFFF` = no valid reading |
| 16 | range_left | uint16 | mm | `0xFFFF` = no valid reading |
| 18 | range_right | uint16 | mm | `0xFFFF` = no valid reading |

- Raw data only. The `odometry` node on the Pi turns ticks and gyro into `/odom`, using `t_ms` for dt (do not use Pi arrival time, USB jitter would add error).
- Ticks are cumulative and never reset during a run, so a dropped frame loses no distance.
- **[decide]** the range sensor count and placement (three assumed) once Belal finalises the sensor list.
- **[decide]** whether bias correction happens on the ESP32 (assumed) or the Pi.

### 0x83 VITALS (ESP to Pi), LEN 7

| Offset | Field | Type | Unit | Notes |
| --- | --- | --- | --- | --- |
| 0 | flags | uint8 | | bit0 = HR valid, bit1 = SpO2 valid, bit2 = IR temp valid, bit3 = contact temp valid, bit4 = finger detected |
| 1 | heart_rate | uint8 | bpm | 0 if not valid |
| 2 | spo2 | uint8 | % | 0 if not valid |
| 3 | temp_ir | int16 | 0.01 °C | MLX90614 (forehead, primary) |
| 5 | temp_contact | int16 | 0.01 °C | MAX30205 (cross-check) |

- A value is only usable if its valid flag is set. Zero in a field never means "measured zero".
- **[decide]** HR and SpO2 are computed on the ESP32 from the MAX30102 raw signal (assumed). Streaming raw PPG to the Pi would need a separate message.

### 0x84 DRAWER_STATUS (ESP to Pi), LEN 2

| Offset | Field | Type | Notes |
| --- | --- | --- | --- |
| 0 | unlocked_mask | uint8 | bit0 to bit2 = drawers, bit3 = lid; 1 = unlocked |
| 1 | open_mask | uint8 | Same bit layout; 1 = limit switch reports open |

- Sent immediately on any change, plus once per second.
- "Dispensed" is derived on the Pi: unlock commanded, then `open_mask` bit set, then cleared again.

### 0x85 STATUS (ESP to Pi), LEN 4

| Offset | Field | Type | Notes |
| --- | --- | --- | --- |
| 0 | flags | uint16 | See below |
| 2 | battery_mv | uint16 | Pack voltage in mV, 0 if no voltage sensing is fitted |

| Bit | Flag | Meaning |
| --- | --- | --- |
| 0 | ESTOP | Hardware E-stop is active |
| 1 | WATCHDOG | Motors stopped because DRIVE_CMD went silent for 0.5 s |
| 2 | OBST_SLOW | Something within 40 cm, speed limited |
| 3 | OBST_STOP | Something within 15 cm, motors stopped |
| 4 | IMU_FAULT | Gyro not responding |
| 5 | VITALS_FAULT | A vitals sensor not responding |
| 6 | RANGE_FAULT | A range sensor not responding |
| 7 | GYRO_CAL | Gyro calibration in progress |
| 8 to 15 | reserved | Send as 0 |

- Sent immediately on any flag change, plus at 2 Hz.
- The Pi reads this to know *why* the robot stopped instead of guessing.
- **[decide]** whether battery voltage sensing is worth an ADC divider (recommended, it prevents silent brown-outs).

## 6. Reliability

- Commands that are ACKed (`0x02`, `0x03`, `0x04`): if no ACK within 200 ms, `serial_bridge` resends, up to 3 times, then reports failure to `mission_manager`.
- Because commands are state-setting (not toggles), duplicates are harmless.
- Periodic ESP32 messages need no ACK. The Pi treats `SENSOR_DATA` silence of more than 200 ms as a lost link, stops sending drive commands and alerts the mission manager.
- Counters and timestamps are cumulative, so a dropped frame costs no accumulated data.

## 7. Safety interaction

These behaviours live on the ESP32 and do not depend on the Pi:

| Condition | ESP32 behaviour |
| --- | --- |
| No valid `DRIVE_CMD` for 0.5 s | Motors stop, `WATCHDOG` flag |
| Obstacle within 40 cm | Speed limited, `OBST_SLOW` flag |
| Obstacle within 15 cm | Motors stop, `OBST_STOP` flag |
| E-stop pressed | Motor power cut in hardware, `ESTOP` flag; `DRAWER_CMD` unlock refused |
| Sensor fault | Fault flag set; `DRAWER_CMD` unlock refused |

The Pi's commands never override these. They only apply when the ESP32's own checks allow them.

## 8. ROS mapping (`serial_bridge`)

| Frame | ROS interface |
| --- | --- |
| `DRIVE_CMD` | subscribes `/cmd_vel` (Twist), converts to wheel speeds, sends at 20 Hz |
| `SENSOR_DATA` | publishes `/sensor_data` (custom) |
| `VITALS` | publishes `/vitals` (custom) |
| `DRAWER_CMD` | service `/drawer/unlock` (custom), waits for the ACK |
| `DRAWER_STATUS`, `STATUS` | publish custom status topics |
| `MEASURE_CMD`, `SYSTEM_CMD` | custom services |

## 9. Test vectors

Hand-built frames for testing the parser from a laptop terminal or the fake-ESP32 script.

| Frame | Bytes (hex) |
| --- | --- |
| DRIVE_CMD, both wheels 200 mm/s | `AA 01 04 00 C8 00 C8 05` |
| DRAWER_CMD, unlock drawer 1, relock after 30 s | `AA 02 03 01 01 1E 1F` |
| ACK, DRAWER_CMD, OK | `AA 81 02 02 00 81` |

Checksums: `01^04^00^C8^00^C8 = 05`, `02^03^01^01^1E = 1F`, `81^02^02^00 = 81`.

Also test: a corrupted checksum (expect no reply), a payload containing `0xAA` (expect correct resync), a truncated frame (expect the 50 ms timeout), and `LEN` = 33 (expect rescan).

## 10. Open decisions

1. Baud rate (115200 to start).
2. Number and placement of range sensors.
3. Gyro bias correction on ESP32 or Pi.
4. HR/SpO2 computed on ESP32 or Pi.
5. Battery voltage sensing on the PCB/wiring.
6. Wheel speed unit: mm/s (assumed) or encoder ticks/s. mm/s is easier to read when debugging but the ESP32 then needs one ticks-per-mm constant for its speed loop.
