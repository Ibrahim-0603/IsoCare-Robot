# Raw navigation data from the ESP32 (protocol frame 0x82, 50 Hz).
# Units are the ESP32's own wire units; odometry converts them.
std_msgs/Header header      # stamp = Pi time when the frame was received (do NOT use for dt)
uint32 esp_time_ms          # ESP32 uptime at the sample; odometry uses this for dt
int32 left_ticks            # cumulative since boot, positive = forward
int32 right_ticks           # cumulative since boot, positive = forward
int16 gyro_z_centideg_s     # 0.01 deg/s, bias-corrected, positive = counter-clockwise
float32 range_front_mm      # NaN = no valid reading
float32 range_left_mm       # NaN = no valid reading
float32 range_right_mm      # NaN = no valid reading
