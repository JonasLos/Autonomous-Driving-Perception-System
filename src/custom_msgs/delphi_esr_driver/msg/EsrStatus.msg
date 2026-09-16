# Delphi ESR 2.5 radar status (decoded from CAN IDs 0x4E0-0x4E3)
std_msgs/Header header

# From 0x4E0 - ESR_Status1
uint16  scan_count            # radar scan counter, wraps at 65535
uint8   rolling_count         # 2-bit rolling count from status message
bool    comm_error            # CAN communication error flag
bool    xmit_power_mode       # transmit power mode
uint8   max_tracks_ack        # number of track slots being reported
bool    xcvr_operational      # transceiver operational

# From 0x4E2 - ESR_Status2
uint8   raw_byte0_4e2         # raw byte 0 (sw_version_dsp / config bytes)

# From 0x4E0 - ESR_Status1 host vehicle motion fields
float32 host_speed            # [m/s] radar-reported host vehicle speed
float32 host_yaw_rate         # [deg/s] radar-reported host yaw rate

# All active tracks in this scan cycle
EsrTrack[] tracks
