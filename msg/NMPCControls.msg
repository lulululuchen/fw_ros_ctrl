# ROS message containing NMPC control horizons
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[50] u_T       # throttle input [~]
float32[50] phi_ref   # roll angle reference [rad]
float32[50] theta_ref # pitch angle reference [rad]
