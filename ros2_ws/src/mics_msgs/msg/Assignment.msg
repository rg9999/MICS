# PRD §8.2 — a (drone -> target) mapping. target_id == 0 means unassigned/standby.
std_msgs/Header header
uint32 drone_id
uint32 target_id
uint8 role              # 0=primary 1=standby
