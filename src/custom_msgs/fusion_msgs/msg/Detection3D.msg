# One sensor's observation of one object, in a form every detector can emit and the
# aggregator can consume without knowing which sensor produced it.
#
# header.stamp is the CAPTURE time of the sensor data, never the publish time. The aggregator
# releases measurements in capture order across sensors, so a wrong stamp here reorders the
# filter's updates rather than merely delaying them.
std_msgs/Header header

uint8 SENSOR_CAMERA_LIDAR=0   # 2D detection lifted by LiDAR returns inside the box
uint8 SENSOR_LIDAR=1          # standalone cluster, no camera involved
uint8 SENSOR_RADAR=2          # Delphi ESR track
uint8 SENSOR_CAMERA_ONLY=3    # box geometry only; no LiDAR return landed in the box
uint8 sensor

# ---- Cartesian measurement, in header.frame_id ----
# Radar populates this for convenience but carries its NATIVE polar measurement below; the
# aggregator uses the polar form, because converting to Cartesian smears the radar's 0.1 m
# range accuracy into the lateral direction through the conversion's cross-covariance.
geometry_msgs/Pose pose
float64[9] position_covariance     # row-major 3x3, in header.frame_id

# ---- extent and heading ----
# size_measured FALSE means these are a class prior, not an observation. Past ~40 m the LiDAR
# puts only ~3 rings on a 1.5 m body and neither length nor yaw is recoverable from shape.
geometry_msgs/Vector3 size
bool size_measured
float32 yaw                        # heading in header.frame_id, radians
uint8 YAW_FROM_SHAPE_FIT=0
uint8 YAW_FROM_VELOCITY=1
uint8 YAW_FROM_RAY_DEFAULT=2       # fabricated: the object is assumed to face the sensor
uint8 yaw_source
bool yaw_measured

# ---- identity ----
int32 class_id
string class_name
float64 score
string tracker_id                  # ByteTrack id; empty for lidar and radar detectors

# ---- native radar measurement. NaN for every other sensor. ----
float32 radar_range                # [m]
float32 radar_range_rate           # [m/s] positive = RECEDING
float32 radar_azimuth_deg          # [deg] positive = LEFT of boresight
float32 radar_amplitude
uint8   radar_track_id             # 0-63 ESR slot index, RECYCLED -- not an identity

int32 point_count                  # LiDAR returns behind this detection; 0 for radar
