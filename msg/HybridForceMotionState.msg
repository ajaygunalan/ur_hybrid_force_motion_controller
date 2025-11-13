uint8 STATE_WAITING_FOR_START = 0
uint8 STATE_READY = 1
uint8 STATE_RUNNING = 2
uint8 STATE_PAUSED = 3
uint8 STATE_COMPLETED = 4
uint8 STATE_ABORTED = 5
uint8 STATE_FAULT = 6

uint8 PHASE_SEEK = 0
uint8 PHASE_DWELL = 1
uint8 PHASE_TANGENTIAL = 2

builtin_interfaces/Time stamp
uint8 state
uint8 phase
float32 normal_force
float32 normal_force_target
float32 normal_force_error
float32 tangential_distance
float32 tangential_target
bool dwell_active
bool paused
bool fault_active
string fault_reason
