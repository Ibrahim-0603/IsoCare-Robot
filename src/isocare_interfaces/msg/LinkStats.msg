# Health of the serial link, published by serial_bridge.
std_msgs/Header header
bool link_ok                # false if SENSOR_DATA has been silent for more than link_lost_s
uint32 frames_ok            # valid frames received
uint32 bad_checksum         # candidate frames rejected by checksum
uint32 bad_length           # candidate frames rejected for LEN > 32
uint32 timeouts             # partial frames dropped by the 50 ms rule
uint32 junk_bytes           # bytes discarded before a sync byte
float32 ack_rtt_ms          # last command-to-ACK round trip, NaN if none yet
float32 esp_clock_jitter_ms # delay above the best recent case, NaN until enough data
