# PRD §8.1 — a GS-side (cued, uncertain) target track.
std_msgs/Header header
uint32 target_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[9] position_covariance
float64 class_confidence
uint8 source            # 0=internal_sim 1=external 2=onboard
float64 age             # seconds since last real update
bool has_velocity
