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

float32[51] soft_aoa       # soft angle of attack
float32[51] jac_soft_aoa_0 # nonzero jacobian 0 of soft angle of attack
float32[51] jac_soft_aoa_1 # nonzero jacobian 1 of soft angle of attack
float32[51] soft_h         # soft nadir terrain clearance
float32[51] jac_soft_h_0   # nonzero jacobian 0 of soft nadir terrain clearance
float32[51] jac_soft_h_1   # nonzero jacobian 1 of soft nadir terrain clearance
float32[51] jac_soft_h_2   # nonzero jacobian 2 of soft nadir terrain clearance
float32[51] jac_soft_h_3   # nonzero jacobian 3 of soft nadir terrain clearance
float32[51] soft_r         # soft radial terrain clearance
float32[51] jac_soft_r_0   # nonzero jacobian 0 of soft radial terrain clearance
float32[51] jac_soft_r_1   # nonzero jacobian 1 of soft radial terrain clearance
float32[51] jac_soft_r_2   # nonzero jacobian 2 of soft radial terrain clearance
float32[51] jac_soft_r_3   # nonzero jacobian 3 of soft radial terrain clearance
float32[51] jac_soft_r_4   # nonzero jacobian 4 of soft radial terrain clearance
float32[51] jac_soft_r_5   # nonzero jacobian 5 of soft radial terrain clearance
