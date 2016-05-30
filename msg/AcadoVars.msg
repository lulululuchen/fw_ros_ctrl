# ROS message containing ACADO variables
# Thomas Stastny | thomas.stastny@mavt.ethz.ch

# states (note: this only records the "measured" state values, not the horizon states)
float32 n			# northing [m]
float32 e			# easting [m]
float32 xi			# heading angle [rad]
float32 intg_e_t	# integral of track position error [m-s]
float32 intg_e_chi	# integral of course angle error [rad-s]
float32 sw			# track switching state [~]

# controls
float32 mu_r		# bank angle reference [rad]

# online data
float32 V			# airspeed reference
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
float32 wn			# northing wind speed
float32 we			# easting wind speed
float32 k_chi		# course angle gradient gain
float32 mu_r_prev	# previous control input

# references
float32 y_e_t
float32 y_e_chi
float32 y_intg_e_t
float32 y_intg_e_chi
float32 y_mu_r
float32 y_delta_mu_r