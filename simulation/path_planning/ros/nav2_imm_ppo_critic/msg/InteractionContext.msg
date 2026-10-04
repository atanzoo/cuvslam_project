# SIMULATION ONLY, v1. This is the flattened typed form of the accepted G1 JSON.
# header.stamp is the query ROS time bound to the received pose_seq by the
# disarmed/arming bridge. It is NOT source_sim_time_s or observation_sim_time_s.
# header.frame_id must equal frame_id. The bridge verifies clock/pose alignment.
std_msgs/Header header
uint32 schema_version
string<=128 epoch_id
string<=128 scene_id
string<=128 frame_id
uint64 packet_seq
uint64 pose_seq
uint64 observation_seq
float64 source_sim_time_s
float64 observation_sim_time_s
float64 ttl_s
bool sensing_valid
float64 controller_horizon_s
string<=128 prediction_semantics
string<=128 action_id
bool waitable
float64 wait_elapsed_s
float64 wait_duration_limit_s

# Actor i metadata uses index i; every metadata array has identical length <=8.
# Zero actors means every array below is empty and sensing_valid is still true.
string<=128[<=8] actor_id
float64[<=8] confidence
float64[<=8] body_radius_m
float64[<=8] observation_age_s
# Triples for actor i are [CV, CA, CTRV] at model_weights[3*i : 3*i+3].
# Diagnostics only: these values never scale or cancel human collision costs.
float64[<=24] model_weights
uint8[<=8] sample_counts
# Concatenate each actor's samples in actor order, with offsets given by the
# prefix sum of sample_counts. Each actor has 2..64 samples, begins at t=0,
# strictly increases, and covers >=3.5 s. Offsets are from the query ROS stamp
# when evaluated by the critic; do NOT add track observation age a second time.
# G1 trajectory_world_xy_m[i][j] maps to the matching x_m/y_m entries here.
float64[<=512] prediction_times_s
float64[<=512] trajectory_world_x_m
float64[<=512] trajectory_world_y_m
float64[<=512] uncertainty_radius_m
