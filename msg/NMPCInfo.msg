# ROS message containing NMPC information
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# offboard control status
int32 status
int32 err_code

# solver info
float64 kkt
float64 obj

# timing
uint64 t_ctrl
uint64 t_preeval
uint64 t_prep
uint64 t_fb
uint64 t_iter
