# ROS message containing NMPC end term objective references
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32 vn       # unit ground speed, north [~]
float32 ve       # unit ground speed, east [~]
float32 vd       # unit ground speed, down [~]
float32 v        # airspeed [m/s]
float32 phi      # roll angle [rad]
float32 theta    # pitch angle [rad]
float32 soft_aoa # soft angle of attack [~]
float32 soft_h   # soft nadir terrain clearance [~]
float32 soft_r   # soft radial terrain clearance [~]
