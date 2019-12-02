# ROS message containing NMPC objective references
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32[50] vn       # unit ground speed, north [~]
float32[50] ve       # unit ground speed, east [~]
float32[50] vd       # unit ground speed, down [~]
float32[50] v        # airspeed [m/s]
float32[50] phi      # roll angle [rad]
float32[50] theta    # pitch angle [rad]
float32[50] soft_aoa # soft angle of attack [~]
float32[50] soft_h   # soft nadir terrain clearance [~]
float32[50] soft_r   # soft radial terrain clearance [~]

float32[50] u_T       # throttle input [~]
float32[50] phi_ref   # roll angle reference [rad]
float32[50] theta_ref # pitch angle reference [rad]
