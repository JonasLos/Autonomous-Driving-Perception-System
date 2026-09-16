# Delphi ESR 2.5 single track report (decoded from CAN ID 0x500-0x53F)
# Track slot  0 = CAN ID 0x500, slot 63 = CAN ID 0x53F
std_msgs/Header header

uint8   track_id          # 0-63, slot index

float32 range             # [m] 0-204.7, res 0.1 m
float32 range_rate        # [m/s] approx +/-81.9, res 0.01 m/s (signed, positive=receding)
float32 angle             # [deg] approx +/-51.2, res 0.1 deg (positive=left of boresight)
float32 amplitude         # Track motion-power estimate from CAN ID 0x540 (power field, approx -10..21)

uint8   track_status      # 3-bit status from CAN track frame
bool    is_cipv           # not provided directly in ESR track frame; currently false
bool    is_merged         # derived heuristic (grouping_changed OR bridge_object)
bool    is_oncoming       # oncoming bit from track frame
bool    is_new_target     # true when track_status indicates new target
bool    range_rate_ambiguous  # not directly decoded yet

float32 lat_rate          # [m/s] approx +/-8.0, res 0.25 m/s
uint8   update_count      # rolling_count bit from track frame
