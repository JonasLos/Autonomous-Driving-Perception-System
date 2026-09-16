# One perceived object, carrying camera+LiDAR geometry and, where radar matched it,
# radar-derived range and velocity.
#
# This is a superset of what /fused_bbox's yolo_msgs/Detection carries for the obstacle
# path. It exists as a separate message rather than as extra fields on Detection because
# adding a field changes that message's type hash, which would silently stop every
# existing /fused_bbox subscriber from matching until it was rebuilt.

# ---- identity, copied verbatim from the fused detection ----
int32 class_id
string class_name
float64 score

# Track ID from tracking_node. Empty for a radar-only object: the radar's own track_id
# lives in radar_track_id and is NOT interchangeable with the visual tracker's IDs.
string id

# ---- geometry ----
# Position of the object centre and its extent. frame_id is carried explicitly as well as
# in the array header because a consumer that copies a single object out of the array
# would otherwise lose it.
#
# WARNING: this is the LiDAR frame, lidar_tc, and lidar_tc is yawed about -5.35 deg from
# the vehicle axis while /tf_static publishes lidar_tc -> base_link as identity. Radar and
# LiDAR agree with each other in this frame, so association is sound, but a consumer that
# treats these coordinates as vehicle-frame inherits a 9.4 cm lateral error per metre of
# range. See CHANGELOG 2026-09-02.
geometry_msgs/Pose center
geometry_msgs/Vector3 size
string frame_id

# ---- velocity ----
# RADIAL COMPONENT ONLY, expressed in frame_id. The radar measures range rate along the
# sensor-to-target ray and nothing else, so a vehicle crossing the field of view at 20 m/s
# reports approximately zero here. Do not read this as a full velocity vector.
# All zeros and velocity_valid=false unless source is RADAR_MATCHED or RADAR_ONLY.
geometry_msgs/Vector3 velocity
bool velocity_valid

# ---- provenance ----
uint8 SOURCE_LIDAR_ONLY=0     # camera+LiDAR fusion only; no radar contribution
uint8 SOURCE_RADAR_MATCHED=1  # camera+LiDAR object that a radar track was associated to
uint8 SOURCE_RADAR_ONLY=2     # originated by radar alone; no visual detection
uint8 source

# ---- radar measurements ----
# All NaN / zero when source is LIDAR_ONLY.
float32 radar_range           # [m] measured slant range
float32 radar_range_rate      # [m/s] positive = receding
float32 radar_azimuth_deg     # [deg] in the radar frame, positive = left of boresight
float32 radar_amplitude       # track motion-power estimate, approx -10..21
uint8 radar_track_id          # 0-63 ESR slot index; only meaningful with a radar match

# ---- range cross-check ----
# A 2D box around a distant vehicle also contains the road in front of it, and the fused
# range can latch onto those nearer road returns: measured 14.3 m median (22.2 m worst)
# short, at 76-91 m. Radar range is accurate to ~0.1 m out to 175 m, so a large positive
# disagreement is the signature of that failure.
#
# Only meaningful when source is RADAR_MATCHED. False / NaN otherwise -- absence of a
# dispute is NOT evidence the range is good.
bool range_disputed
float32 range_disagreement    # [m] fused range minus radar range; NaN when unmatched
