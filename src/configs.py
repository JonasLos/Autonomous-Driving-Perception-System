import numpy as np

# Raw Sensor topics
CAMERA_TOPIC = "/camera_fl/image_color"
LIDAR_TOPIC = "/lidar_tc/velodyne_points"
RADAR_TRACKS_TOPIC = "/radar_fc/as_tx/radar_tracks"

# YOLO output topics
YOLO_IMAGE_TOPIC = "yolo_published_image"
YOLO_BBOX_TOPIC = "/yolo_bboxInfo"
FUSED_BBOX_TOPIC = "/fused_bbox"

# SAM output topics
SAM_SEGMENTATION_MASK_TOPIC = "/sam_road_segmentation"
SAM_LEFT_CONTOUR_TOPIC = "/sam_left_contour"
SAM_RIGHT_CONTOUR_TOPIC = "/sam_right_contour"
SAM_LEFT_BOUNDARY = "/sam_left_boundary"
SAM_RIGHT_BOUNDARY = "/sam_right_boundary"

# SPHEREFORMER output topics
SPHEREFORMER_SEGMENTATION_TOPIC = "/sphereformer_road_segmentation"
SPHEREFORMER_LEFT_BOUNDARY = "/sphereformer_left_boundary"
SPHEREFORMER_RIGHT_BOUNDARY = "/sphereformer_right_boundary"
SPHEREFORMER_CENTER_LINE_POINTS = "/sphereformer_centerline_points"
LIDAR_BBOX_TOPIC = "/bounding_boxes"

# ULTRAFAST output topics
LANE_DETECTION_MASK_TOPIC = "/lane_detection/output"
LEFT_LANE_TOPIC = "/lane_detection/current_lane_left_boundary"
RIGHT_LANE_TOPIC = "/lane_detection/current_lane_right_boundary"
LEFT_LANE_BOUNDARY_TOPIC = "/Left_Line3dPoints"
RIGHT_LANE_BOUNDARY_TOPIC = "/Right_Line3dPoints"

# 2D-3D Transformation
LIDAR_2D_PROJ_TOPIC = "/lidar_2d_projection"

# Calibration Parameters
# Camera intrinsic parameters
#if rectified image is subscribed to this is obsolete, only use if using unrectified images
'''
PROJ = np.array(
    [
        [3.5612204509314029e03 / 2, 0.0, 9.9143998670769213e02 / 2, 0.0],
        [0, 3.5572532571086072e03 / 2, 7.8349772942764150e02 / 2, 0.0],
        [0, 0.0, 1.0, 0],
    ]
)

# Camera to lidar extrinsic transformation matrix
'''
T1 = np.array(
    [[ -4.8076040039157775e-03, 1.1565175070195832e-02, 9.9992156375854679e-01, 1.3626313209533691e00,],
     [ -9.9997444266988167e-01, -5.3469003551928074e-03, -4.7460155553246119e-03, 2.0700573921203613e-02,],
     [ 5.2915924636425249e-03, -9.9991882539643562e-01, 1.1590585274754983e-02, -9.1730421781539917e-01,],
     [ 0.0, 0.0, 0.0, 1.0],]
)
'''


T1 = np.array( [[-0.08769843, -0.04834693,  0.99497315,  1.20882,],
 [-0.99608904, -0.00652521, -0.08811386,  0.273073,  ],
 [ 0.01075245, -0.99880929, -0.0475856,  -0.842512,  ],
 [ 0.,          0.,          0.,          1.,        ],])
'''

T1 = np.array( [[-0.08769842, 0.04646302, 0.9950629, 1.20882,],
 [ 0.99608908,  0.0148725,   0.08709439, 0.273073,],
 [-0.01075241,  0.99880928, -0.04758558 ,-0.842512,],
[ 0.,          0.,          0.,          1.,        ],] )
'''

