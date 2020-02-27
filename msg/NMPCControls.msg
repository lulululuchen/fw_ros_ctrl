# ROS message containing NMPC control horizons
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[50] throt       # throttle input [~]
float32[50] roll_ref    # roll angle reference [rad]
float32[50] pitch_ref   # pitch angle reference [rad]
