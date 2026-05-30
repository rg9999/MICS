# Continuous run status published on /sim_run_status (PRD viewer §6.7).
uint8 IDLE=0
uint8 LAUNCHING=1
uint8 RUNNING=2
uint8 STOPPING=3
uint8 STOPPED=4
uint8 ERROR=5

std_msgs/Header header
uint8   state
string  scenario_id
string  run_id                 # unique per run; "" when IDLE
float64 elapsed_s
uint8   drones_up
uint8   targets_up
string  recording_session_id   # "" if not recording
float64 requested_rtf          # requested real-time factor (0 = max)
float64 actual_rtf             # achieved RTF (may be < requested when compute-bound)
bool    rtf_controllable       # false on HITL/real hardware (locked at 1.0)
string  message
