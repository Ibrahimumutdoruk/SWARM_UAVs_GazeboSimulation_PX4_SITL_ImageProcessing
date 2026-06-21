std_msgs/Header header

bool armed
bool offboard_active
uint8 nav_state
uint8 arming_state
bool landed
bool estimator_ready
float32 battery_remaining

float32[3] position_ned
float32[3] velocity_ned
float32 heading
float32 roll
float32 pitch

float64 ref_lat_deg
float64 ref_lon_deg
float32 ref_alt_m
