# One tracked object from the multi-sensor aggregator.
#
# Frame: this is the `ego` frame -- a perception-owned child of lidar_tc, rotated by the
# measured -5.35 deg so it is genuinely vehicle-longitudinal. It is NOT base_link: tf_static
# publishes lidar_tc -> base_link as identity, which is an uncalibrated placeholder, and
# reading lidar_tc coordinates as vehicle coordinates costs 9.4 cm of lateral error per metre
# of range. The correction is confirmed by three independent methods agreeing to 0.19 deg.
uint32 track_id                    # stable across frames; NOT the ByteTrack id
float64 age                        # [s] since birth
uint32 update_count                # measurements applied over this track's life

int32 class_id
string class_name
float64 score                      # fused class confidence, majority-voted over the track

# ---- geometry ----
geometry_msgs/Pose pose            # centroid, with a real yaw
geometry_msgs/Vector3 size
bool size_measured
uint8 YAW_FROM_SHAPE_FIT=0
uint8 YAW_FROM_VELOCITY=1
uint8 YAW_FROM_RAY_DEFAULT=2
uint8 yaw_source

# Row-major 4x4 over the filter state [x, y, vx, vy], in frame_id.
# All-NaN means "not filtered" (passthrough mode). It is deliberately NOT all-zero, which
# would read as infinite confidence.
float64[16] covariance

# ---- velocity ----
# FULL 2D ground-referenced velocity, not the radial-only component TrackedObject carries.
# velocity_covariance is what says how much of it is real: a track updated only by radar has
# a rank-deficient covariance -- small along the ray, huge across it.
geometry_msgs/Vector3 velocity
float64[4] velocity_covariance     # row-major 2x2
bool velocity_valid                # false when odometry was unavailable

# ---- confidence and provenance ----
float32 existence_probability      # [0,1]
uint8 STATUS_TENTATIVE=0
uint8 STATUS_CONFIRMED=1
uint8 STATUS_COASTING=2
uint8 track_status

uint8 CONTRIB_CAMERA=1
uint8 CONTRIB_LIDAR=2
uint8 CONTRIB_RADAR=4
uint8 contributions                # sensors that have EVER updated this track
uint8 contributions_this_frame
uint8 missed_updates               # consecutive cycles coasting

# ---- range quality ----
# Camera range is dropped entirely beyond ~80 m, where it is not noisy but WRONG: measured
# -7.59 m median at 80-100 m and -32.10 m at 100-175 m against radar. True here means this
# object's position is carried by radar range and camera lateral only.
bool camera_range_dropped

# Camera updates rejected by the innovation gate over this track's life. The along-ray error
# is a mixture, not a Gaussian, so outliers are REJECTED rather than absorbed; 36% of updates
# were rejected on the reference replay. A track with a high rejection count has a position
# worth distrusting even though every accepted update looked fine.
uint32 camera_updates_rejected

string frame_id
