# ROS message containing NMPC auxiliary outputs
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32 path_error_lat # lateral-directional path error [m]
float32 path_error_lon # longitudinal path error [m]
float32 err_v_n_unit   # unit northing ground velocity error [~]
float32 err_v_e_unit   # unit easting ground velocity error [~]
float32 err_v_d_unit   # unit downing ground velocity error [~]
