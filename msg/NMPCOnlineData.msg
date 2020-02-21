# ROS message containing NMPC online data
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

float32 rho       # air density [kg/m^3]
float32 wn        # northing wind [m/s]
float32 we        # easting wind [m/s]
float32 wd        # down wind [m/s]
float32 tau_phi   # control augmented roll response time constant [s]
float32 tau_theta # control augmented pitch response time constant [s]
float32 k_phi     # control augmented roll response gain [~]
float32 k_theta   # control augmented pitch response gain [~]
float32 tau_n     # propeller speed time constant [s]
float32 delta_F   # symmetric flap deflection [rad]

float32[51] soft_aoa        # soft angle of attack
float32[51] jac_soft_aoa_0  # jacobian 0 of soft angle of attack
float32[51] jac_soft_aoa_1  # jacobian 1 of soft angle of attack
float32[51] soft_hagl       # soft height above ground level (HAGL)
float32[51] jac_soft_hagl_0 # jacobian 0 of soft height above ground level (HAGL)
float32[51] jac_soft_hagl_1 # jacobian 1 of soft height above ground level (HAGL)
float32[51] jac_soft_hagl_2 # jacobian 2 of soft height above ground level (HAGL)
float32[51] jac_soft_hagl_3 # jacobian 3 of soft height above ground level (HAGL)
float32[51] soft_rtd        # soft radial terrain distance (RTD)
float32[51] jac_soft_rtd_0  # jacobian 0 of soft radial terrain distance (RTD)
float32[51] jac_soft_rtd_1  # jacobian 1 of soft radial terrain distance (RTD)
float32[51] jac_soft_rtd_2  # jacobian 2 of soft radial terrain distance (RTD)
float32[51] jac_soft_rtd_3  # jacobian 3 of soft radial terrain distance (RTD)
float32[51] jac_soft_rtd_4  # jacobian 4 of soft radial terrain distance (RTD)
float32[51] jac_soft_rtd_5  # jacobian 5 of soft radial terrain distance (RTD)
