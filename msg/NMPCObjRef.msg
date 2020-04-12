# ROS message containing NMPC objective references
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[50] pos_n       # position north [m]
float32[50] pos_e       # position east [m]
float32[50] airsp       # airspeed [m/s]
float32[50] fpa         # flight path angle [rad]
float32[50] heading     # heading angle [rad]
float32[50] soft_airsp  # soft airspeed [~]
float32[50] soft_aoa    # soft angle of attack (AoA) [~]
float32[50] soft_hagl   # soft height above ground level (HAGL) [~]
float32[50] soft_rtd    # soft radial terrain distance (RTD) [~]


float32[50] throt       # throttle input [~]
float32[50] roll_ref    # roll angle reference [rad]
float32[50] pitch_ref   # pitch angle reference [rad]
