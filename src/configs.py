import numpy as np

# Camera intrinsic parameters
# If rectified image is subscribed to this is obsolete, only use if using unrectified images

# Old intrinsics (pre-2025-04-16 calibration)
# PROJ = np.array(
#     [
#         [3391.5379517746969, 0, 1022.9778357081916, 0],
#         [0, 3391.537035878745, 771.55254940421378, 0],
#         [0, 0, 1, 0],
#     ]
# )

# Intrinsics from camera_fl.intrinsics.yaml (TAMU.Jeep_GC.White_Jeep.2025-04-16.17-20-11.01)
PROJ = np.array(
    [
        [3461.179235458374, 0, 1031.3118348121643, 0],
        [0, 3461.1789894104004, 771.39050769805908, 0],
        [0, 0, 1, 0],
    ]
)

# Camera to lidar extrinsic transformation matrix
# Old extrinsics (pre-2025-04-16 calibration)
# T1 = np.array(
#     [
#         [ 0.0327348, -0.0104690,  0.9994093, 1.931740],
#         [-0.9980448, -0.0536130,  0.0321285, 0.257106],
#         [ 0.0532450, -0.9985069, -0.0122035, -0.880199],
#         [ 0.0,        0.0,        0.0,        1.0],
#     ]
# )

# Extrinsics from extrinsics.yaml camera_fl entry (TAMU.Jeep_GC.White_Jeep.2025-04-16.17-20-11.01)
# Derived from: [x=1.208815, y=0.273073, z=-0.842512, qx=-0.491530, qy=0.531214, qz=-0.511528, qw=0.463194]
T1 = np.array(
    [
        [-0.0876984, -0.0483418,  0.9949730, 1.208815],
        [-0.9960886, -0.0065253, -0.0881142, 0.273073],
        [ 0.0107524, -0.9988092, -0.0475801, -0.842512],
        [ 0.0,        0.0,        0.0,        1.0],
    ]
)
