# ROS message containing ACADO variables
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# states (note: this only records the "measured" state values, not the horizon states)
float32[41] n			# northing [m]
float32[41] e			# easting [m]
float32[41] d			# down [m]
float32[41] V			# airspeed [m/s]
float32[41] gamma		# flight path angle [rad]
float32[41] xi			# heading angle [rad]
float32[41] phi			# roll angle [rad]
float32[41] theta		# pitch angle [rad]
float32[41] p			# roll rate [rad/s]
float32[41] q			# pitch rate [rad/s]
float32[41] r			# yaw rate [rad/s]
float32[41] throt		# throttle setting [%]
float32[41] xsw			# switching state [~]

# controls
float32[40] uT			# throttle input
float32[40] phi_ref		# roll angle reference
float32[40] theta_ref	# pitch angle reference

# online data
uint8 pparam1   	# type(=0)	type(=1)
float32 pparam2     # aa_n    	cc_n
float32 pparam3     # aa_e    	cc_e
float32 pparam4     # aa_d    	cc_d
float32 pparam5     # bb_n    	R
float32 pparam6     # bb_e    	dir
float32 pparam7     # bb_d    	gam
float32 pparam8     # --      	xi0
float32 pparam9     # --      	dxi
uint8 pparam1_next  
float32 pparam2_next 
float32 pparam3_next 
float32 pparam4_next
float32 pparam5_next
float32 pparam6_next
float32 pparam7_next
float32 pparam8_next
float32 pparam9_next
float32 R_acpt
float32 ceta_acpt

float32 wn
float32 we
float32 wd

float32 k_t_d 			# longitudinal logistic gain
float32 e_d_co     		# longitudinal logistic cutoff [m]
float32 k_t_ne      	# lateral logistic gain
float32 e_ne_co     	# lateral logistic cutoff [m]
float32 eps_v       	# unit ground speed threshold [~]
float32 alpha_p_co    	# angle of attack upper cutoff [rad]
float32 alpha_m_co    	# angle of attack lower cutoff [rad]
float32 alpha_delta_co  # angle of attack cutoff transition length [rad]

# references
float32 y_V
float32 y_uT0
float32 y_phi_ref0
float32 y_theta_ref0