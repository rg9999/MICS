# PRD §8.3 — broadcast on /state_sharing_drone_N.
std_msgs/Header header
uint32 drone_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
uint8 state             # IDLE..FAILED enum (see mics.msgs.DroneState)
uint32 current_target
float32 battery_pct
float32 track_quality   # 0..1, onboard estimate quality
