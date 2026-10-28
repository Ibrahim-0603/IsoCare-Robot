# Drawer lock and limit-switch state (protocol frame 0x84, on change and 1 Hz).
# Index 0 to 2 = drawers, index 3 = lid.
std_msgs/Header header
bool[4] unlocked    # true = unlocked
bool[4] open        # true = limit switch reports open
