# ROS message containing wind ground truth information
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# wind ground truth from gazebo

# cartesian coordinates
float32 north
float32 east
float32 down

# spherical coordinates
float32 speed
float32 azimuth
float32 inclination

# wind estimate from EKF2 -- spherical coordinates
float32 ekf2_speed
float32 ekf2_azimuth
float32 ekf2_inclination
