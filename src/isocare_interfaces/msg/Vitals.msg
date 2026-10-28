# Vitals sample from the ESP32 (protocol frame 0x83, 1 Hz while measuring).
# A value is only usable if its valid flag is true. Zero never means "measured zero".
std_msgs/Header header
bool hr_valid
bool spo2_valid
bool ir_temp_valid
bool contact_temp_valid
bool finger_detected
uint8 heart_rate_bpm          # only meaningful if hr_valid
uint8 spo2_percent            # only meaningful if spo2_valid
int16 temp_ir_centi_c         # 0.01 degC, MLX90614 forehead (primary); only if ir_temp_valid
int16 temp_contact_centi_c    # 0.01 degC, MAX30205 (cross-check); only if contact_temp_valid
