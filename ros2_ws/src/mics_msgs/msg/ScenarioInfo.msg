# Catalog descriptor for one vetted scenario (PRD viewer §6.7).
# The server-side `path` is gateway-internal and never exposed raw to clients.
string scenario_id
string name
string description
string target_source       # internal | external
uint8  defender_count
uint8  attacker_count
string sensor_mode         # ideal | stub | full
string path                # server-side path; not forwarded to the browser
