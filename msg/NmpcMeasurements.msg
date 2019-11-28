# ROS message containing NMPC state measurements
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32 n      # northing position [m]
float32 e      # easting position [m]
float32 d      # down position [m]
float32 v      # airspeed [m/s]
float32 gamma  # flight path angle [rad]
float32 xi     # heading angle [rad]
float32 phi    # roll angle [rad]
float32 theta  # pitch angle [rad]
float32 n_prop # propeller speed [rps]
