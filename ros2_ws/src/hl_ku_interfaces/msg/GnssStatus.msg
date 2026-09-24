std_msgs/Header header

uint8 FIX_NONE=0
uint8 FIX_SINGLE=1
uint8 FIX_DGPS=2
uint8 FIX_RTK_FIXED=4
uint8 FIX_RTK_FLOAT=5

uint8 fix_type
bool heading_valid
bool position_valid
bool velocity_valid
float64 heading_true_deg
float64 ground_speed_mps
float64 track_true_deg
uint16 satellites
float32 hdop
float32 correction_age_sec
float32 position_age_sec
float32 velocity_age_sec
float32 heading_age_sec
bool nmea_checksum_valid
