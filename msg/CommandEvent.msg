# 고규범 신규 로직 (2026-10-21). Observation only; NEVER an actuator command.
uint8 COMMAND=1
uint8 TRANSMITTED=2
uint8 DRY_RUN=3
# COMMAND: immediately before command publication.
# TRANSMITTED: first sendto start, published only after all three sends succeed.
# This is PC socket time, NOT board receipt or physical brake onset.
std_msgs/Header header
string command_topic
uint8 stage
float32 steering_raw
float32 speed_raw
bool brake
