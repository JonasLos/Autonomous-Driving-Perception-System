#!/bin/bash

# List of ROS topics to check
TOPICS=(
  "resized/camera_fl/image_color"
  "/lidar_tc/velodyne_points"
  "/radar_fc/as_tx/radar_tracks"
  "yolo_published_image"
  "/yolo_bboxInfo"
  "/fused_bbox"
  "/sam_road_segmentation"
  "/sam_left_contour"
  "/sam_right_contour"
  "/sam_left_boundary"
  "/sam_right_boundary"
  "/sphereformer_road_segmentation"
  "/sphereformer_left_boundary"
  "/sphereformer_right_boundary"
  "/sphereformer_centerline_points"
  "/bounding_boxes"
  "/lane_detection/output"
  "/lane_detection/current_lane_left_boundary"
  "/lane_detection/current_lane_right_boundary"
  "/Left_Line3dPoints"
  "/Right_Line3dPoints"
  "/lidar_2d_projection"
)

echo "Gathering topic frequencies..."
echo "------------------------------"

for topic in "${TOPICS[@]}"; do
  echo "Topic: $topic"
  timeout 3s rostopic hz "$topic" 2>/dev/null | grep -i "average rate" || echo "  No data or not publishing"
  echo "------------------------------"
done
