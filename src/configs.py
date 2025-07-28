import numpy as np

# Camera intrinsic parameters
# If rectified image is subscribed to this is obsolete, only use if using unrectified images

PROJ = np.array(
    [
        [3391.5379517746969, 0, 1022.9778357081916, 0],
        [0, 3391.537035878745, 771.55254940421378, 0],
        [0, 0, 1, 0],
    ]
)

# Camera to lidar extrinsic transformation matrix
T1 = np.array(
    [
        [
            0.0327348,
            -0.0104690,
            0.9994093,
            1.931740,
        ],
        [
            -0.9980448,
            -0.0536130,
            0.0321285,
            0.257106,
        ],
        [
            0.0532450,
            -0.9985069,
            -0.0122035,
            -0.880199,
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ]
)
