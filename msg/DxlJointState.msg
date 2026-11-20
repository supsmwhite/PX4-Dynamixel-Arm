# Joint position feedback from a completed DYNAMIXEL Sync Read, ordered by servo ID 1, 2, 3.

uint64 timestamp # [us] Time since system start when the flight controller finished reading the group
int32[3] raw_position # [count] Single-turn Present Position register values
float32[3] position_rad # [rad] Relative to the 180 degree center; negative up, positive down
