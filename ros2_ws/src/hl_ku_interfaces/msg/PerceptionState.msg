std_msgs/Header header

uint8 LIGHT_UNKNOWN=0
uint8 LIGHT_RED=1
uint8 LIGHT_YELLOW=2
uint8 LIGHT_GREEN=3
uint8 LIGHT_LEFT=4

uint8 END_SIGNAL_UNKNOWN=0
uint8 END_SIGNAL_DOWN=1
uint8 END_SIGNAL_X=2

float32 lane_offset_m
float32 lane_heading_error_rad
float32 lane_confidence
bool stop_line_detected
float32 stop_line_distance_m
uint8 traffic_light
float32 traffic_confidence
bool obstacle_in_path
float32 obstacle_distance_m
float32 avoidance_steering_offset_rad
uint8 end_lane_signal
int8 allowed_lane
float32 end_lane_confidence

bool camera_fresh
bool lidar_fresh
bool end_signal_fresh
