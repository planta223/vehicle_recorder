# 고규범 신규 로직 (2026-10-21). Measured state, independent of commanded FSM.
std_msgs/Header header
string command_topic
uint64 stop_id
bool command_active
bool command_fresh
bool speed_valid
float64 speed_mps
builtin_interfaces/Time sample_stamp
bool stopped
# First qualifying GNSS sample, NOT the time confirmation is published.
builtin_interfaces/Time stopped_since
builtin_interfaces/Time confirmed_at
builtin_interfaces/Time command_stamp
builtin_interfaces/Time transmit_stamp
bool command_position_valid
float64 command_x_m
float64 command_y_m
bool transmit_position_valid
float64 transmit_x_m
float64 transmit_y_m
bool stop_position_valid
float64 stop_x_m
float64 stop_y_m
bool distance_valid
float64 distance_m
string reason
