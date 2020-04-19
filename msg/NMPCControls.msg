# ROS message containing NMPC control horizons
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[50] throt       # throttle input [~]
float32[50] roll_ref    # roll angle reference [rad]
float32[50] pitch_ref   # pitch angle reference [rad]

float32[50] throt_ub       # throttle upper bound [~]
float32[50] roll_ref_ub    # roll angle reference upper bound [rad]
float32[50] pitch_ref_ub   # pitch angle reference upper bound [rad]

float32[50] throt_lb       # throttle lower bound [~]
float32[50] roll_ref_lb    # roll angle reference lower bound [rad]
float32[50] pitch_ref_lb   # pitch angle reference lower bound [rad]
