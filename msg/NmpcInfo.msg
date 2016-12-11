# ROS message containing NMPC information
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# solver info
float64 kkt
float64 obj

# timing
uint64 t_ctrl
uint64 t_solve
uint64 t_update 
uint64 t_wp_man
uint64 t_iter