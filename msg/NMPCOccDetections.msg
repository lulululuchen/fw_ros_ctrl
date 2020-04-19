# ROS message containing NMPC occlusion detections / attributes
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

int32 len_occ_data
int32 len_occ_window
int32 len_occ_buffer
int32 ray_casting_interval
int32 rtd_node_interval
float32 surfel_radius

int8[101] fwd_detections
float32[101] fwd_pos_0
float32[101] fwd_pos_1
float32[101] fwd_pos_2
float32[101] fwd_normal_0
float32[101] fwd_normal_1
float32[101] fwd_normal_2

int8[101] right_detections
float32[101] right_pos_0
float32[101] right_pos_1
float32[101] right_pos_2
float32[101] right_normal_0
float32[101] right_normal_1
float32[101] right_normal_2

int8[101] left_detections
float32[101] left_pos_0
float32[101] left_pos_1
float32[101] left_pos_2
float32[101] left_normal_0
float32[101] left_normal_1
float32[101] left_normal_2
