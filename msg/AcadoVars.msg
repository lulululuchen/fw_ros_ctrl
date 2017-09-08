# ROS message containing ACADO variables
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# measurements
float32[13] x0			# measurement vector

# state horizons
float32[81] n			# northing [m]
float32[81] e			# easting [m]
float32[81] d			# down [m]
float32[81] V			# airspeed [m/s]
float32[81] gamma		# flight path angle [rad]
float32[81] xi			# heading angle [rad]
float32[81] phi			# roll angle [rad]
float32[81] theta		# pitch angle [rad]
float32[81] p			# roll rate [rad/s]
float32[81] q			# pitch rate [rad/s]
float32[81] r			# yaw rate [rad/s]
float32[81] throt		# throttle setting [%]
float32[81] xsw			# switching state [~]

# controls
float32[80] uT			# throttle input
float32[80] phi_ref		# roll angle reference
float32[80] theta_ref	# pitch angle reference

# online data
uint8 pparam1   		# type(=0)	type(=1)
float32 pparam2     	# aa_n    	cc_n
float32 pparam3     	# aa_e    	cc_e
float32 pparam4     	# aa_d    	cc_d
float32 pparam5     	# bb_n    	R
float32 pparam6     	# bb_e    	dir
float32 pparam7     	# bb_d    	gam
float32 pparam8     	# --      	xi0
float32 pparam9     	# --      	dxi
uint8 pparam1_next  	# type(=0)	type(=1)
float32 pparam2_next 	# aa_n    	cc_n
float32 pparam3_next 	# aa_e    	cc_e
float32 pparam4_next	# aa_d    	cc_d
float32 pparam5_next	# bb_n    	R
float32 pparam6_next	# bb_e    	dir
float32 pparam7_next	# bb_d    	gam
float32 pparam8_next	# --      	xi0
float32 pparam9_next	# --      	dxi
float32 R_acpt			# track switch acceptance radius (proximity condition)
float32 ceta_acpt		# cosine of track switch acceptance angular error (bearing condition)
float32 wn				# north wind component
float32 we				# east wind component
float32 wd				# down wind component
float32 alpha_p_co    	# angle of attack upper cutoff [rad]
float32 alpha_m_co    	# angle of attack lower cutoff [rad]
float32 alpha_delta_co  # angle of attack cutoff transition length [rad]
float32 T_b_lat  		# lateral-directional track error boundary constant [m]
float32 T_b_lon  		# longitudinal track error boundary constant [m]

# references
float32 y_V
float32 y_uT
float32 y_theta_ref
