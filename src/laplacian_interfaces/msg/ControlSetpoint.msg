std_msgs/Header header

uint8 MODE_HOLD = 0
uint8 MODE_POSITION = 1
uint8 MODE_POSITION_VELOCITY = 2

uint8 mode
float32[3] position_ned
float32[3] velocity_ned
float32 yaw_rad
