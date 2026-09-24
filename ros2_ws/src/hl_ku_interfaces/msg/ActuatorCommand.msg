std_msgs/Header header

uint8 MODE_DISABLED=0
uint8 MODE_AUTONOMOUS=1
uint8 MODE_BRAKE=2
uint8 MODE_FAULT=3
uint8 MODE_MANUAL=4

float32 drive_duty
float32 steering_angle_rad
bool brake
bool enable
uint8 mode
uint32 sequence
