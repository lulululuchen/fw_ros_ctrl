# ROS message containing NMPC state horizons
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[51] pos_n       # northing position [m]
float32[51] pos_e       # easting position [m]
float32[51] pos_d       # down position [m]
float32[51] airsp       # airspeed [m/s]
float32[51] fpa         # flight path angle [rad]
float32[51] heading     # heading angle [rad]
float32[51] roll        # roll angle [rad]
float32[51] pitch       # pitch angle [rad]
float32[51] prop_speed  # propeller speed [rps]
