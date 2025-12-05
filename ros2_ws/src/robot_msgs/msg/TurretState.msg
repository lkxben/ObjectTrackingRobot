string mode            # IDLE | AUTONOMOUS | MANUAL
string status          # IDLE | SEARCHING | DETECTING | TRACKING | LOST | MANUAL_CONTROL | ERROR

float32 target_id      # currently tracked target ID, -1 if none
bool has_target        # true if a valid target exists

builtin_interfaces/Time stamp