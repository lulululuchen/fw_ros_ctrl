# ROS message containing NMPC end term objective references
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32 pos_n       # position north [m]
float32 pos_e       # position east [m]
float32 pos_d       # position down [m]
float32 airsp       # airspeed [m/s]
float32 fpa         # flight path angle [rad]
float32 heading     # heading angle [rad]
float32 soft_airsp  # soft airspeed [~]
float32 soft_aoa    # soft angle of attack (AoA) [~]
float32 soft_hagl   # soft height above ground level (HAGL) [~]
float32 soft_rtd    # soft radial terrain distance (RTD) [~]
