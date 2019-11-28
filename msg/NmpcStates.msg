# ROS message containing NMPC state horizons
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[51] n      # northing position [m]
float32[51] e      # easting position [m]
float32[51] d      # down position [m]
float32[51] v      # airspeed [m/s]
float32[51] gamma  # flight path angle [rad]
float32[51] xi     # heading angle [rad]
float32[51] phi    # roll angle [rad]
float32[51] theta  # pitch angle [rad]
float32[51] n_prop # propeller speed [rps]
