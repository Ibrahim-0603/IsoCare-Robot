# Safety and fault flags from the ESP32 (protocol frame 0x85, on change and 2 Hz).
std_msgs/Header header
bool estop            # hardware E-stop is active
bool watchdog         # motors stopped because DRIVE_CMD went silent for 0.5 s
bool obstacle_slow    # something within 40 cm, speed limited
bool obstacle_stop    # something within 15 cm, motors stopped
bool imu_fault        # gyro not responding
bool vitals_fault     # a vitals sensor not responding
bool range_fault      # a range sensor not responding
bool gyro_cal         # gyro calibration in progress
uint16 battery_mv     # pack voltage in mV, 0 if no voltage sensing is fitted
