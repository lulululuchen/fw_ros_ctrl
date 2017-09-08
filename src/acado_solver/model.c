#include "acado_common.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

bool check_line_seg( const double *pos, const double *vel, const double *params );
bool check_curve_seg( const double *pos, const double *vel, const double *params );

void rhs( const real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);
const double t22 = sin(in[4]);

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[36]+Vsafe*t2*cos(in[5]);
const double e_dot = in[37]+Vsafe*t2*sin(in[5]);
const double d_dot = in[38]-Vsafe*t22;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t23 = in[11]*in[11];
const double t24 = 1.0/Vsafe;
const double t25 = Vsafe*Vsafe;
const double t26 = alpha*alpha;
const double t27 = in[11]*8.61861E1;
const double t28 = in[11]*t23*2.501023E2;
const double t36 = t23*3.05322E1;
const double t29 = t27+t28-t36;
const double t30 = alpha*5.0996;
const double t31 = t26*(-5.343)+t30+1.7E1/3.2E1;
const double t32 = t25*t31*2.38875E-1;
const double t33 = cos(alpha);
const double t34 = 1.0/t33;
const double t35 = sin(alpha);
const double t37 = t24*t29*t34*t35;
const double t38 = t32+t37;
const double t39 = cos(in[6]);
const double t40 = sin(in[6]);

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t22*(-9.81E2/1.0E2)+t24*t29*(2.0E1/5.3E1)-t25*(alpha*2.5491E-1+t26*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t24*(t2*(9.81E2/1.0E2)-t38*t39*(2.0E1/5.3E1));
out[5] = (t24*t38*t40*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t39-in[10]*t40;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[14]*1.24716E1;
out[9] = -t25*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[15]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[14]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[13]*4.143016944939305;
out[12] = sw_dot;

}

void rhs_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);
const double t22 = sin(in[4]);

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[36]+Vsafe*t2*cos(in[5]);
const double e_dot = in[37]+Vsafe*t2*sin(in[5]);
const double d_dot = in[38]-Vsafe*t22;

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t23 = in[11]*in[11];
const double t24 = 1.0/Vsafe;
const double t25 = Vsafe*Vsafe;
const double t26 = alpha*alpha;
const double t27 = in[11]*8.61861E1;
const double t28 = in[11]*t23*2.501023E2;
const double t36 = t23*3.05322E1;
const double t29 = t27+t28-t36;
const double t30 = alpha*5.0996;
const double t31 = t26*(-5.343)+t30+1.7E1/3.2E1;
const double t32 = t25*t31*2.38875E-1;
const double t33 = cos(alpha);
const double t34 = 1.0/t33;
const double t35 = sin(alpha);
const double t37 = t24*t29*t34*t35;
const double t38 = t32+t37;
const double t39 = cos(in[6]);
const double t40 = sin(in[6]);

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t22*(-9.81E2/1.0E2)+t24*t29*(2.0E1/5.3E1)-t25*(alpha*2.5491E-1+t26*2.7337+6.4105E-2)*9.014150943396226E-2;
out[4] = -t24*(t2*(9.81E2/1.0E2)-t38*t39*(2.0E1/5.3E1));
out[5] = (t24*t38*t40*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t39-in[10]*t40;
out[8] = in[6]*(-1.24716E1)-in[8]*7.4252+in[10]*1.0069+in[14]*1.24716E1;
out[9] = -t25*(alpha*1.9303E-1+in[7]*1.8359E-1+in[9]*4.6239E-2-in[15]*1.8359E-1-9.4955E-4);
out[10] = in[6]*5.7996-in[10]*9.5153+in[14]*1.5967;
out[11] = in[11]*(-4.143016944939305)+in[13]*4.143016944939305;
out[12] = sw_dot;

}

void rhs_jac( const real_t *in, real_t *out ){

/* rhs_jac */
 
double f_Delta_m[ACADO_NX];
double f_Delta_p[ACADO_NX];
double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];
memcpy(in_Delta, in, sizeof(in_Delta));
const double Delta = 0.00001;
const double Delta2 = 2.0 * Delta;
 
int i;
int j;
for (i = 0; i < (ACADO_NX+ACADO_NU); i=i+1) {
 
    in_Delta[i] = in[i] - Delta;
    rhs_eval( in_Delta, f_Delta_m );
    in_Delta[i] = in[i] + Delta;
    rhs_eval( in_Delta, f_Delta_p );
    in_Delta[i] = in[i];
 
    for (j = 0; j < ACADO_NX; j=j+1) {
        out[j*(ACADO_NX+ACADO_NU)+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
    }
 
}

}

void lsq_obj_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[36]+Vsafe*t2*cos(in[5]);
const double e_dot = in[37]+Vsafe*t2*sin(in[5]);
const double d_dot = in[38]-Vsafe*sin(in[4]);

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double p_n = 0.0;
double p_e = 0.0;
double p_d = 0.0;
double tP_n = 1.0;
double tP_e = 0.0;
double tP_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        tP_n = abn / norm_ab;
        tP_e = abe / norm_ab;
        tP_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = tP_n*(in[0] - pparam_aa_n) + tP_e*(in[1] - pparam_aa_e) + tP_d*(in[2] - pparam_aa_d);
    
    // point on track
    p_n = pparam_aa_n + dot_abunit_ap * tP_n;
    p_e = pparam_aa_e + dot_abunit_ap * tP_e;
    p_d = pparam_aa_d + dot_abunit_ap * tP_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cr = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cr>0.1) {
        cp_n_unit = cp_n / norm_cr;
        cp_e_unit = cp_e / norm_cr;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    p_n = pparam_R * cp_n_unit + pparam_cc_n;
    p_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    tP_n = pparam_ldir * -cp_e_unit;
    tP_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        p_d = pparam_cc_d;
        tP_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        p_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* p (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((p_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            p_d = pparam_cc_d + delta_d_sp_end;
            tP_d = 0.0;
        }
        else {
            tP_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
    }
    
    // Renormalize tP
    const double normtP = sqrt(tP_n*tP_n+tP_e*tP_e+tP_d*tP_d);
    tP_n = tP_n / normtP;
    tP_e = tP_e / normtP;
    tP_d = tP_d / normtP;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double alpha = -in[4]+in[7];

const double t3 = in[1]-p_e;
const double t4 = in[0]-p_n;
const double norm_rp_ne = sqrt(t3*t3+t4*t4);

const double t5 = 1.0/norm_rp_ne;
const double t6 = -in[2]+p_d;

double sgn_rp = 0.0;
if (t6>0.0) {
    sgn_rp = 1.0;
} else if (t6<0.0) {
    sgn_rp = -1.0;
}

const double t16 = e_dot*e_dot;
const double t17 = n_dot*n_dot;
const double t18 = t16+t17;

const double e_lat = t4*tP_e-t3*tP_n;

double e_b_lat;
if (t18>1.0) {
    e_b_lat = sqrt(t18)*in[42];                               
} else {
    e_b_lat = in[42]*(1.0/2.0)+in[42]*t18*(1.0/2.0);
}
double sat_e_lat = fabs(e_lat)/e_b_lat;
if (sat_e_lat>1.0) sat_e_lat = 1.0;

const double t7 = sat_e_lat-1.0;
const double t8 = t7*t7;
const double t9 = 3.141592653589793*t8*(1.0/2.0);
const double t10 = cos(t9);
const double t11 = sin(t9);

const double e_lon = t6;

double e_b_lon;
if (fabs(d_dot)>1.0) {
    e_b_lon = fabs(d_dot)*in[43];                               
} else {
    e_b_lon = in[43]*(1.0/2.0)+in[43]*fabs(d_dot)*(1.0/2.0);
}
double sat_e_lon = fabs(e_lon)/e_b_lon;
if (sat_e_lon>1.0) sat_e_lon = 1.0;

const double t12 = sat_e_lon-1.0;
const double t13 = t12*t12;
const double t14 = 3.141592653589793*t13*(1.0/2.0);
const double t15 = sin(t14);

const double rp_n_unit = -t4*t5;
const double rp_e_unit = -t3*t5;

const double atan2_01 = atan2(rp_e_unit*t10+t11*tP_e, rp_n_unit*t10+t11*tP_n);
const double atan2_02 = atan2(e_dot, n_dot);
double eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793) {
    eta_lat = eta_lat - 6.283185307179586;
}
else if (eta_lat<-3.141592653589793) {
    eta_lat = eta_lat + 6.283185307179586;
}

const double atan2_03 = atan2(-t15*tP_d-sgn_rp*cos(t14), t15*sqrt(tP_e*tP_e+tP_n*tP_n));
const double atan2_04 = atan2(-d_dot, sqrt(t18));
double eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793) {
    eta_lon = eta_lon - 6.283185307179586;
}
else if (eta_lon<-3.141592653589793) {
    eta_lon = eta_lon + 6.283185307179586;
}

const double t19 = alpha-in[39]+in[41];
const double t20 = 1.0/(in[41]*in[41]);
const double t21 = -alpha+in[40]+in[41];

double a_soft;
if (alpha>(in[39]-in[41])) {
    a_soft=(t19*t19)*t20;
}
else if (alpha>(in[40]+in[41])) {
    a_soft=0.0;
}
else {
    a_soft=t20*(t21*t21);
}

/* outputs */

out[0] = eta_lat;
out[1] = eta_lon;
out[2] = Vsafe;
out[3] = in[8];
out[4] = in[9];
out[5] = in[10];
out[6] = a_soft;
out[7] = in[11]*(-4.143016944939305)+in[13]*4.143016944939305;
out[8] = in[13];
out[9] = in[14];
out[10] = in[15];

}

void evaluateLSQ( const real_t *in, real_t *out ){

double in_Delta[ACADO_NX+ACADO_NU+ACADO_NOD];
memcpy(in_Delta, in, sizeof(in_Delta));
lsq_obj_eval( in_Delta, out );
 
/* lsq_obj jacobians */
 
double f_Delta_m[ACADO_NY];
double f_Delta_p[ACADO_NY];
const double Delta = 0.00001;
const double Delta2 = 2.0 * Delta;
 
int i;
int j;
for (i = 0; i < ACADO_NX; i=i+1) {
 
    in_Delta[i] = in[i] - Delta;
    lsq_obj_eval( in_Delta, f_Delta_m );
    in_Delta[i] = in[i] + Delta;
    lsq_obj_eval( in_Delta, f_Delta_p );
    in_Delta[i] = in[i];
 
    for (j = 0; j < ACADO_NY; j=j+1) {
        out[ACADO_NY+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
    }
 
}
 
for (i = 0; i < ACADO_NU; i=i+1) {
 
    in_Delta[i+ACADO_NX] = in[i+ACADO_NX] - Delta;
    lsq_obj_eval( in_Delta, f_Delta_m );
    in_Delta[i+ACADO_NX] = in[i+ACADO_NX] + Delta;
    lsq_obj_eval( in_Delta, f_Delta_p );
    in_Delta[i+ACADO_NX] = in[i+ACADO_NX];
 
    for (j = 0; j < ACADO_NY; j=j+1) {
        out[ACADO_NY+ACADO_NY*ACADO_NX+j*ACADO_NU+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
    }
 
}

}

void lsq_objN_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = ACADO_NU;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[33]+Vsafe*t2*cos(in[5]);
const double e_dot = in[34]+Vsafe*t2*sin(in[5]);
const double d_dot = in[35]-Vsafe*sin(in[4]);

/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
bool b_switch_segment = false;
int pparam_sel = 0;
double sw_dot = 0.0;
if ( in[ACADO_NX-1] < 0.05 ) { // check x_sw
    const double vel[3] = {n_dot,e_dot,d_dot};
    if ( in[idx_OD_0] < 0.5 ) { // path type
        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
    }
} else {
    b_switch_segment = true;
}
if (b_switch_segment) {
    pparam_sel = 9;
    sw_dot = 1.0;
} 

double p_n = 0.0;
double p_e = 0.0;
double p_d = 0.0;
double tP_n = 1.0;
double tP_e = 0.0;
double tP_d = 0.0;

const double pparam_type = in[idx_OD_0+pparam_sel];

// LINE SEGMENT
if ( pparam_type < 0.5 ) {

    // variable definitions
    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

    // calculate vector from waypoint a to b
    const double abn = pparam_bb_n - pparam_aa_n;
    const double abe = pparam_bb_e - pparam_aa_e;
    const double abd = pparam_bb_d - pparam_aa_d;
    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

    // calculate tangent
    if (norm_ab>0.1) {
        tP_n = abn / norm_ab;
        tP_e = abe / norm_ab;
        tP_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = tP_n*(in[0] - pparam_aa_n) + tP_e*(in[1] - pparam_aa_e) + tP_d*(in[2] - pparam_aa_d);
    
    // point on track
    p_n = pparam_aa_n + dot_abunit_ap * tP_n;
    p_e = pparam_aa_e + dot_abunit_ap * tP_e;
    p_d = pparam_aa_d + dot_abunit_ap * tP_d;
    
// CURVE SEGMENT
} else if ( pparam_type < 1.5 ) {

    // variable definitions
    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
    const double pparam_R = in[idx_OD_0+pparam_sel+4];
    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

    // calculate closest point on loiter circle
    const double cp_n = in[0] - pparam_cc_n;
    const double cp_e = in[1] - pparam_cc_e;
    const double norm_cr = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cr>0.1) {
        cp_n_unit = cp_n / norm_cr;
        cp_e_unit = cp_e / norm_cr;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    p_n = pparam_R * cp_n_unit + pparam_cc_n;
    p_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    tP_n = pparam_ldir * -cp_e_unit;
    tP_e = pparam_ldir * cp_n_unit;
    
    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi_p = xi_sp-pparam_xi0;
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi_p = delta_xi_p + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi_p = delta_xi_p - 6.28318530718;

    }

    // closest point on nearest spiral leg and tangent down component
    if (fabs(pparam_gam_sp) < 0.001) {

        p_d = pparam_cc_d;
        tP_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi_p * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // closest point on nearest spiral leg
        p_d = pparam_cc_d + delta_d_k + delta_d_xi;

        /* p (on spiral) = (start height) + (revolution height increment) +
         * (lateral-direcitonal angular position increment)
         */
        
        // cap end point
        if ((p_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
            p_d = pparam_cc_d + delta_d_sp_end;
            tP_d = 0.0;
        }
        else {
            tP_d = -sin(pparam_gam_sp);
        }
        
    }
    
    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        tP_n=1.0;
    }
    
    // Renormalize tP
    const double normtP = sqrt(tP_n*tP_n+tP_e*tP_e+tP_d*tP_d);
    tP_n = tP_n / normtP;
    tP_e = tP_e / normtP;
    tP_d = tP_d / normtP;
        
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double alpha = -in[4]+in[7];

const double t3 = in[1]-p_e;
const double t4 = in[0]-p_n;
const double norm_rp_ne = sqrt(t3*t3+t4*t4);

const double t5 = 1.0/norm_rp_ne;
const double t6 = -in[2]+p_d;

double sgn_rp = 0.0;
if (t6>0.0) {
    sgn_rp = 1.0;
} else if (t6<0.0) {
    sgn_rp = -1.0;
}

const double t16 = e_dot*e_dot;
const double t17 = n_dot*n_dot;
const double t18 = t16+t17;

const double e_lat = t4*tP_e-t3*tP_n;

double e_b_lat;
if (t18>1.0) {
    e_b_lat = sqrt(t18)*in[39];                               
} else {
    e_b_lat = in[39]*(1.0/2.0)+in[39]*t18*(1.0/2.0);
}
double sat_e_lat = fabs(e_lat)/e_b_lat;
if (sat_e_lat>1.0) sat_e_lat = 1.0;

const double t7 = sat_e_lat-1.0;
const double t8 = t7*t7;
const double t9 = 3.141592653589793*t8*(1.0/2.0);
const double t10 = cos(t9);
const double t11 = sin(t9);

const double e_lon = t6;

double e_b_lon;
if (fabs(d_dot)>1.0) {
    e_b_lon = fabs(d_dot)*in[40];                               
} else {
    e_b_lon = in[40]*(1.0/2.0)+in[40]*fabs(d_dot)*(1.0/2.0);
}
double sat_e_lon = fabs(e_lon)/e_b_lon;
if (sat_e_lon>1.0) sat_e_lon = 1.0;

const double t12 = sat_e_lon-1.0;
const double t13 = t12*t12;
const double t14 = 3.141592653589793*t13*(1.0/2.0);
const double t15 = sin(t14);

const double t19 = alpha-in[36]+in[38];
const double t20 = 1.0/(in[38]*in[38]);
const double t21 = -alpha+in[37]+in[38];

const double rp_n_unit = -t4*t5;
const double rp_e_unit = -t3*t5;

const double atan2_01 = atan2(rp_e_unit*t10+t11*tP_e, rp_n_unit*t10+t11*tP_n);
const double atan2_02 = atan2(e_dot, n_dot);
double eta_lat = atan2_01-atan2_02;
if (eta_lat>3.141592653589793) {
    eta_lat = eta_lat - 6.283185307179586;
}
else if (eta_lat<-3.141592653589793) {
    eta_lat = eta_lat + 6.283185307179586;
}

const double atan2_03 = atan2(-t15*tP_d-sgn_rp*cos(t14), t15*sqrt(tP_e*tP_e+tP_n*tP_n));
const double atan2_04 = atan2(-d_dot, sqrt(t18));
double eta_lon = atan2_03-atan2_04;
if (eta_lon>3.141592653589793) {
    eta_lon = eta_lon - 6.283185307179586;
}
else if (eta_lon<-3.141592653589793) {
    eta_lon = eta_lon + 6.283185307179586;
}

double a_soft;
if (alpha>(in[36]-in[38])) {
    a_soft=(t19*t19)*t20;
}
else if (alpha>(in[37]+in[38])) {
    a_soft=0.0;
}
else {
    a_soft=t20*(t21*t21);
}

/* outputs */

out[0] = eta_lat;
out[1] = eta_lon;
out[2] = Vsafe;
out[3] = in[8];
out[4] = in[9];
out[5] = in[10];
out[6] = a_soft;

}

void evaluateLSQEndTerm( const real_t *in, real_t *out ){

double in_Delta[ACADO_NX+ACADO_NOD];
memcpy(in_Delta, in, sizeof(in_Delta));
lsq_objN_eval( in_Delta, out );
 
/* lsq_objN jacobians */
 
double f_Delta_m[ACADO_NYN];
double f_Delta_p[ACADO_NYN];
const double Delta = 0.00001;
const double Delta2 = 2.0 * Delta;
 
int i;
int j;
for (i = 0; i < ACADO_NX; i=i+1) {
 
    in_Delta[i] = in[i] - Delta;
    lsq_objN_eval( in_Delta, f_Delta_m );
    in_Delta[i] = in[i] + Delta;
    lsq_objN_eval( in_Delta, f_Delta_p );
    in_Delta[i] = in[i];
 
    for (j = 0; j < ACADO_NYN; j=j+1) {
        out[ACADO_NYN+j*ACADO_NX+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
    }
 
}

}

/* begin inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

bool check_line_seg( const double *pos, const double *vel, const double *params ) {
    
    // waypoint a to b
    const double ab_n = params[3] - params[0];
    const double ab_e = params[4] - params[1];
    const double ab_d = params[5] - params[2];
    
    const double norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d );
    
    // tB
    double tB_n = 1.0;
    double tB_e = 0.0;
    double tB_d = 0.0;
    if (norm_ab > 0.1) {
        tB_n = ab_n / norm_ab;
        tB_e = ab_e / norm_ab;
        tB_d = ab_d / norm_ab;
    }

    // p - b
    const double bp_n = pos[0] - params[3];
    const double bp_e = pos[1] - params[4];
    const double bp_d = pos[2] - params[5];
    
    // dot( (p-r) , tB )
    const double dot_brtB = bp_n * tB_n + bp_e * tB_e + bp_d * tB_d;
    
    // check travel 
    return ( dot_brtB > 0.0 );
    
}

bool check_curve_seg( const double *pos, const double *vel, const double *params ) {
    
    // chi_B
    const double chi_B = params[6] + params[4] * (params[7] + 1.570796326794897);
        
    // tB
    const double tB_n = cos(chi_B)*cos(params[5]);
    const double tB_e = sin(chi_B)*cos(params[5]);
    const double tB_d = -sin(params[5]);

    // p - b
    const double bp_n = pos[0] - ( params[0] + params[3] * ( cos(params[6] + params[4] * params[7]) ) );
    const double bp_e = pos[1] - ( params[1] + params[3] * ( sin(params[6] + params[4] * params[7]) ) );
    const double bp_d = pos[2] - ( params[2] - params[3] * tan(params[5]) * params[7] );
    
    // dot( v , tB )
    const double dot_vtB = vel[0] * tB_n + vel[1] * tB_e + vel[2] * tB_d;
    
    // dot( (p-r) , tB )
    const double dot_brtB = bp_n * tB_n + bp_e * tB_e + bp_d * tB_d;
    
    // norm( p-r )
    const double norm_br = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( norm_br < params[17] && dot_vtB > params[18] && dot_brtB > 0.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */