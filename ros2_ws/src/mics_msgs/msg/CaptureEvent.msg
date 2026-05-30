# PRD §8.4 — emitted on a capture attempt.
std_msgs/Header header
uint32 drone_id
uint32 target_id
uint8 result            # 0=attempt 1=success 2=miss
geometry_msgs/Point engagement_point
