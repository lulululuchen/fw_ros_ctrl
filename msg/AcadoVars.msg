# ROS message containing ACADO variables
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# measurements
float32[13] x0 # measurement vector

# state horizons
float32[71] n			# northing [m]
float32[71] e			# easting [m]
float32[71] d			# down [m]
float32[71] V			# airspeed [m/s]
float32[71] gamma # flight path angle [rad]
float32[71] xi		# heading angle [rad]
float32[71] phi		# roll angle [rad]
float32[71] theta # pitch angle [rad]
float32[71] p			# roll rate [rad/s]
float32[71] q			# pitch rate [rad/s]
float32[71] r			# yaw rate [rad/s]
float32[71] throt	# throttle setting [%]
float32[71] xsw		# switching state [~]

# controls
float32[70] uT			  # throttle input
float32[70] phi_ref		# roll angle reference
float32[70] theta_ref	# pitch angle reference

# online data
uint8 pparam1   		   # type=0 type=1 type=2
float32 pparam2     	 # b_n    c_n    c_n
float32 pparam3     	 # b_e    c_e    c_e
float32 pparam4     	 # b_d    c_d    c_d
float32 pparam5     	 # N/A		 +/-R 	+/-R
float32 pparam6     	 # Chi    Chi    N/A
float32 pparam7     	 # Gam    Gam    N/A
uint8 pparam1_next
float32 pparam2_next
float32 pparam3_next
float32 pparam4_next
float32 pparam5_next
float32 pparam6_next
float32 pparam7_next
float32 R_acpt			   # track switch acceptance radius (proximity condition)
float32 ceta_acpt      # cosine of track switch acceptance angular error (bearing condition)
float32 wn				     # north wind component
float32 we				     # east wind component
float32 wd				     # down wind component
float32 alpha_p_co     # angle of attack upper cutoff [rad]
float32 alpha_m_co     # angle of attack lower cutoff [rad]
float32 alpha_delta_co # angle of attack cutoff transition length [rad]
float32 T_b_lat  		   # lateral-directional track error boundary constant [m]
float32 T_b_lon  		   # longitudinal track error boundary constant [m]

# references
float32 y_V
float32 y_uT
float32 y_theta_ref
