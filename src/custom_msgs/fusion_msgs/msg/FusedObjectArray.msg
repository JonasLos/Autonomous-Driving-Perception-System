# Header carries the CAPTURE stamp of the measurement that produced this update, never
# wall-clock now. The aggregator publishes one fixed lag behind real time by design; hiding
# that by predicting forward would break the invariant that an output describes the moment it
# is stamped with.
std_msgs/Header header
FusedObject[] objects
