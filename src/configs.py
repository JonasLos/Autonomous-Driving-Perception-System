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

# Intrinsics from camera_fl.intrinsics.yaml (TAMU.Jeep_GC.Blue_Jeep.2025-06-24.14-27-11.01)
PROJ = np.array(
    [
        [3411.259361395108, 0, 1021.73025726041, 0],
        [0, 3411.2588644613816, 771.21438165632549, 0],
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

# Extrinsics from extrinsics.yaml camera_fl entry (TAMU.Jeep_GC.Blue_Jeep.2025-06-24.14-27-11.01)
# Using https://www.andre-gaschler.com/rotationconverter/ Derived from: [1.760872, 0.289978, -0.956039, -0.523534, 0.480752, -0.502847, 0.491868]
T1 = np.array(
    [
        [0.0320444, -0.0087113,  0.9994485, 1.760872],
        [-0.9980491, -0.0538864,  0.0315299, 0.289978],
        [ 0.0535820, -0.9985090, -0.0104211, -0.956039],
        [ 0.0,        0.0,        0.0,        1.0],
    ]
)

