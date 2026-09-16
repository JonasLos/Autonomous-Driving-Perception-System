# Header is copied from the /fused_bbox message the objects were built from, so the stamp
# remains the LiDAR sweep capture time and the frame remains lidar_tc. Radar messages are
# matched to that stamp rather than driving it.
std_msgs/Header header
TrackedObject[] objects
