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

const double alpha = -in[4]+in[7];

const double t3 = alpha-in[45]+in[47];
const double t4 = 1.0/(in[47]*in[47]);
const double t5 = -alpha+in[46]+in[47];
const double t6 = sin(in[4]);

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[37]+Vsafe*t2*cos(in[5]);
const double e_dot = in[38]+Vsafe*t2*sin(in[5]);
const double d_dot = in[39]-Vsafe*t6;

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

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

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
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
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
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // check
        if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

            delta_d_k = 0.0;

        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {

            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);

        }

        // closest point on nearest spiral leg
        const double delta_d_sp = delta_d_k + delta_d_xi;
        d_d = pparam_cc_d + delta_d_sp;
        Td_d = -sin(pparam_gam_sp);
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t7 = in[11]*in[11];
const double t8 = 1.0/Vsafe;
const double t9 = Vsafe*Vsafe;
const double t10 = alpha*alpha;
const double t11 = in[11]*4.65309685298437E1;
const double t12 = t7*1.33121145483364E2;
const double t13 = in[11]*t7*1.94400407803984E2;
const double t14 = t11+t12+t13;
const double t15 = alpha*6.8709446964706;
const double t16 = t10*(-2.11069893359182E1)+t15+4.70333508888944E-1;
const double t17 = t9*t16*2.38875E-1;
const double t18 = cos(alpha);
const double t19 = 1.0/t18;
const double t20 = sin(alpha);
const double t21 = t8*t14*t19*t20;
const double t22 = t17+t21;
const double t23 = cos(in[6]);
const double t24 = sin(in[6]);

const double pd_n = d_n-in[0];
const double pd_e = d_e-in[1];
const double pd_d = d_d-in[2];
const double e_t_ne = -Td_e*pd_n+Td_n*pd_e;
const double e_t_d = pd_d;

const double e_t = sqrt(e_t_d*e_t_d+e_t_ne*e_t_ne);
double i_e_t_dot = 0.0;
if (e_t < in[48]) i_e_t_dot = e_t/in[48];

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t6*(-9.81E2/1.0E2)+t8*t14*(2.0E1/5.3E1)-t9*(alpha*3.78620335309777E-1+t10*1.62033558854341+6.23204432055247E-2)*9.014150943396226E-2;
out[4] = -t8*(t2*(9.81E2/1.0E2)-t22*t23*(2.0E1/5.3E1));
out[5] = (t8*t22*t24*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t23-in[10]*t24;
out[8] = in[6]*(-1.11467593207791E1)-in[8]*6.7357877396904+in[10]*8.78182329679164E-1+in[15]*1.11467593207791E1;
out[9] = -t9*(alpha*1.64663711873934E-1+in[7]*1.58960374141736E-1+in[9]*4.10704191099466E-2-in[16]*1.58960374141736E-1-8.48173994434413E-4);
out[10] = in[6]*5.98887765504684-in[10]*9.76492309398636+in[15]*1.58960918118893;
out[11] = in[11]*(-5.583695191098388)+in[14]*5.583695191098388;
out[12] = i_e_t_dot;
out[13] = sw_dot;

}

void rhs_eval( real_t *in, real_t *out ){

/* for manual input indexing ... */

const int minus_NU = 0;

/* optimized intermediate calculations */

const double t2 = cos(in[4]);

const double alpha = -in[4]+in[7];

const double t3 = alpha-in[45]+in[47];
const double t4 = 1.0/(in[47]*in[47]);
const double t5 = -alpha+in[46]+in[47];
const double t6 = sin(in[4]);

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[37]+Vsafe*t2*cos(in[5]);
const double e_dot = in[38]+Vsafe*t2*sin(in[5]);
const double d_dot = in[39]-Vsafe*t6;

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

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

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
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
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
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // check
        if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

            delta_d_k = 0.0;

        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {

            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);

        }

        // closest point on nearest spiral leg
        const double delta_d_sp = delta_d_k + delta_d_xi;
        d_d = pparam_cc_d + delta_d_sp;
        Td_d = -sin(pparam_gam_sp);
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

const double t7 = in[11]*in[11];
const double t8 = 1.0/Vsafe;
const double t9 = Vsafe*Vsafe;
const double t10 = alpha*alpha;
const double t11 = in[11]*4.65309685298437E1;
const double t12 = t7*1.33121145483364E2;
const double t13 = in[11]*t7*1.94400407803984E2;
const double t14 = t11+t12+t13;
const double t15 = alpha*6.8709446964706;
const double t16 = t10*(-2.11069893359182E1)+t15+4.70333508888944E-1;
const double t17 = t9*t16*2.38875E-1;
const double t18 = cos(alpha);
const double t19 = 1.0/t18;
const double t20 = sin(alpha);
const double t21 = t8*t14*t19*t20;
const double t22 = t17+t21;
const double t23 = cos(in[6]);
const double t24 = sin(in[6]);

const double pd_n = d_n-in[0];
const double pd_e = d_e-in[1];
const double pd_d = d_d-in[2];
const double e_t_ne = -Td_e*pd_n+Td_n*pd_e;
const double e_t_d = pd_d;

const double e_t = sqrt(e_t_d*e_t_d+e_t_ne*e_t_ne);
double i_e_t_dot = 0.0;
if (e_t < in[48]) i_e_t_dot = e_t/in[48];

/* rhs */

out[0] = n_dot;
out[1] = e_dot;
out[2] = d_dot;
out[3] = t6*(-9.81E2/1.0E2)+t8*t14*(2.0E1/5.3E1)-t9*(alpha*3.78620335309777E-1+t10*1.62033558854341+6.23204432055247E-2)*9.014150943396226E-2;
out[4] = -t8*(t2*(9.81E2/1.0E2)-t22*t23*(2.0E1/5.3E1));
out[5] = (t8*t22*t24*(2.0E1/5.3E1))/t2;
out[6] = in[8];
out[7] = in[9]*t23-in[10]*t24;
out[8] = in[6]*(-1.11467593207791E1)-in[8]*6.7357877396904+in[10]*8.78182329679164E-1+in[15]*1.11467593207791E1;
out[9] = -t9*(alpha*1.64663711873934E-1+in[7]*1.58960374141736E-1+in[9]*4.10704191099466E-2-in[16]*1.58960374141736E-1-8.48173994434413E-4);
out[10] = in[6]*5.98887765504684-in[10]*9.76492309398636+in[15]*1.58960918118893;
out[11] = in[11]*(-5.583695191098388)+in[14]*5.583695191098388;
out[12] = i_e_t_dot;
out[13] = sw_dot;

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

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[37]+Vsafe*t2*cos(in[5]);
const double e_dot = in[38]+Vsafe*t2*sin(in[5]);
const double d_dot = in[39]-Vsafe*sin(in[4]);

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

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

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
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
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
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // check
        if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

            delta_d_k = 0.0;

        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {

            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);

        }

        // closest point on nearest spiral leg
        const double delta_d_sp = delta_d_k + delta_d_xi;
        d_d = pparam_cc_d + delta_d_sp;
        Td_d = -sin(pparam_gam_sp);
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

/* tracked expressions */

const double pd_n = d_n-in[0];
const double pd_e = d_e-in[1];
const double pd_d = d_d-in[2];
const double e_t_ne = -Td_e*pd_n+Td_n*pd_e;
const double e_t_d = pd_d;

double e_t_1_ne;
if (e_t_ne>in[43]) {
    e_t_1_ne = 1.0;
}
else if (e_t_ne>-in[43]) {
    e_t_1_ne = sin((3.141592653589793*e_t_ne*(1.0/2.0))/in[43]);
}
else {
    e_t_1_ne = -1.0;
}

double e_t_1_d;
if (e_t_d>in[41]) {
    e_t_1_d = 1.0;
}
else if (e_t_d>-in[41]) {
    e_t_1_d = sin((3.141592653589793*e_t_d*(1.0/2.0))/in[41]);
}
else {
    e_t_1_d = -1.0;
}

// negative unit normal vector
double Tpd_n;
double Tpd_e;
double Tpd_d;
const double norm_pd = sqrt(e_t_ne * e_t_ne + e_t_d * e_t_d);
if (norm_pd<1.0) {
    Tpd_n = 0.0;
    Tpd_e = 0.0;
    Tpd_d = 0.0;
}
else {
    Tpd_n = pd_n/norm_pd;
    Tpd_e = pd_e/norm_pd;
    Tpd_d = pd_d/norm_pd;
}

// unit velocity vector
const double norm_v = sqrt(d_dot*d_dot+e_dot*e_dot+n_dot*n_dot);
double vbar_n;
double vbar_e;
double vbar_d;
if (norm_v<0.05) {
    vbar_n=0.0;
    vbar_e=0.0;
    vbar_d=0.0;
}
else if (norm_v<in[44]) {
    vbar_n=((sin(2*norm_v/in[44]/3.141592653589793)-1)+1)*n_dot;
    vbar_e=((sin(2*norm_v/in[44]/3.141592653589793)-1)+1)*e_dot;
    vbar_d=((sin(2*norm_v/in[44]/3.141592653589793)-1)+1)*d_dot;
}
else {
    vbar_n=n_dot/norm_v;
    vbar_e=e_dot/norm_v;
    vbar_d=d_dot/norm_v;
}

const double t3 = alpha-in[45]+in[47];
const double t4 = 1.0/(in[47]*in[47]);
const double t5 = -alpha+in[46]+in[47];
const double t6 = e_t_ne+in[43];
const double t7 = in[42]*t6;
const double t8 = exp(t7);
const double t9 = t8+1.0;
const double t10 = 1.0/t9;
const double t11 = e_t_ne-in[43];
const double t15 = in[42]*t11;
const double t12 = exp(-t15);
const double t13 = t12+1.0;
const double t14 = 1.0/t13;
const double t16 = t10+t14-1.0;
const double t17 = t10+t14;
const double t18 = e_t_d+in[41];
const double t19 = in[40]*t18;
const double t20 = exp(t19);
const double t21 = t20+1.0;
const double t22 = 1.0/t21;
const double t23 = e_t_d-in[41];
const double t24 = exp(-in[40]*t23);
const double t25 = t24+1.0;
const double t26 = 1.0/t25;

// soft constraints
double a_soft;
if (alpha>(in[45]-in[47])) {
    a_soft=(t3*t3)*t4;
}
else if (alpha>(in[46]+in[47])) {
    a_soft=0.0;
}
else {
    a_soft=t4*(t5*t5);
}

/* outputs */

out[0] = e_t_1_ne;
out[1] = e_t_1_d;
out[2] = in[12];
out[3] = -vbar_n-Td_n*t16+Tpd_n*t17;
out[4] = -vbar_e-Td_e*t16+Tpd_e*t17;
out[5] = -vbar_d-Td_d*(t22+t26-1.0)+Tpd_d*(t22+t26);
out[6] = Vsafe;
out[7] = in[8];
out[8] = in[9];
out[9] = in[10];
out[10] = a_soft;
out[11] = in[11]*(-5.583695191098388)+in[14]*5.583695191098388;
out[12] = in[14];
out[13] = in[15];
out[14] = in[16];
out[15] = in[14];
out[16] = in[15];
out[17] = in[16];

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

const double alpha = -in[4]+in[7];

double Vsafe = in[3];
if (Vsafe<1.0) Vsafe = 1.0;

const double n_dot = in[34]+Vsafe*t2*cos(in[5]);
const double e_dot = in[35]+Vsafe*t2*sin(in[5]);
const double d_dot = in[36]-Vsafe*sin(in[4]);

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

double d_n = 0.0;
double d_e = 0.0;
double d_d = 0.0;
double Td_n = 1.0;
double Td_e = 0.0;
double Td_d = 0.0;

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
        Td_n = abn / norm_ab;
        Td_e = abe / norm_ab;
        Td_d = abd / norm_ab;
    }
    
    // dot product
    const double dot_abunit_ap = Td_n*(in[0] - pparam_aa_n) + Td_e*(in[1] - pparam_aa_e) + Td_d*(in[2] - pparam_aa_d);
    
    // point on track
    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
    d_d = pparam_aa_d + dot_abunit_ap * Td_d;
    
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
    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
    double cp_n_unit;
    double cp_e_unit;
    if (norm_cp>0.1) {
        cp_n_unit = cp_n / norm_cp;
        cp_e_unit = cp_e / norm_cp;
    }
    else {
        cp_n_unit = 0.0;
        cp_e_unit = 0.0;
    }
    d_n = pparam_R * cp_n_unit + pparam_cc_n;
    d_e = pparam_R * cp_e_unit + pparam_cc_e;

    // calculate tangent
    Td_n = pparam_ldir * -cp_e_unit;
    Td_e = pparam_ldir * cp_n_unit;

    // spiral angular position: [0,2*pi)
    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
    double delta_xi = xi_sp-pparam_xi0;

    // closest point on nearest spiral leg and tangent down component
    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

        delta_xi = delta_xi + 6.28318530718;

    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

        delta_xi = delta_xi - 6.28318530718;

    }

    if (fabs(pparam_gam_sp) < 0.001) {

        d_d = pparam_cc_d;
        Td_d = 0.0;

    } else {

        const double Rtangam = pparam_R * tan(pparam_gam_sp);

        // spiral height delta for current angle
        const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

        // end spiral altitude change
        const double delta_d_sp_end = -pparam_dxi * Rtangam;

        // nearest spiral leg
        double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
        const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

        // check
        if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

            delta_d_k = 0.0;

        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {

            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);

        }

        // closest point on nearest spiral leg
        const double delta_d_sp = delta_d_k + delta_d_xi;
        d_d = pparam_cc_d + delta_d_sp;
        Td_d = -sin(pparam_gam_sp);
    }
    
    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
        Td_n=1.0;
    }
}

/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

/* tracked expressions */

const double pd_n = d_n-in[0];
const double pd_e = d_e-in[1];
const double pd_d = d_d-in[2];
const double e_t_ne = -Td_e*pd_n+Td_n*pd_e;
const double e_t_d = pd_d;

double e_t_1_ne;
if (e_t_ne>in[40]) {
    e_t_1_ne = 1.0;
}
else if (e_t_ne>-in[40]) {
    e_t_1_ne = sin((3.141592653589793*e_t_ne*(1.0/2.0))/in[40]);
}
else {
    e_t_1_ne = -1.0;
}

double e_t_1_d;
if (e_t_d>in[38]) {
    e_t_1_d = 1.0;
}
else if (e_t_d>-in[38]) {
    e_t_1_d = sin((3.141592653589793*e_t_d*(1.0/2.0))/in[38]);
}
else {
    e_t_1_d = -1.0;
}

// negative unit normal vector
double Tpd_n;
double Tpd_e;
double Tpd_d;
const double norm_pd = sqrt(e_t_ne * e_t_ne + e_t_d * e_t_d);
if (norm_pd<1.0) {
    Tpd_n = 0.0;
    Tpd_e = 0.0;
    Tpd_d = 0.0;
}
else {
    Tpd_n = pd_n/norm_pd;
    Tpd_e = pd_e/norm_pd;
    Tpd_d = pd_d/norm_pd;
}

// unit velocity vector
const double norm_v = sqrt(d_dot*d_dot+e_dot*e_dot+n_dot*n_dot);
double vbar_n;
double vbar_e;
double vbar_d;
if (norm_v<0.05) {
    vbar_n=0.0;
    vbar_e=0.0;
    vbar_d=0.0;
}
else if (norm_v<in[41]) {
    vbar_n=((sin(2*norm_v/in[41]/3.141592653589793)-1)+1)*n_dot;
    vbar_e=((sin(2*norm_v/in[41]/3.141592653589793)-1)+1)*e_dot;
    vbar_d=((sin(2*norm_v/in[41]/3.141592653589793)-1)+1)*d_dot;
}
else {
    vbar_n=n_dot/norm_v;
    vbar_e=e_dot/norm_v;
    vbar_d=d_dot/norm_v;
}

const double t3 = alpha-in[42]+in[44];
const double t4 = 1.0/(in[44]*in[44]);
const double t5 = -alpha+in[43]+in[44];
const double t6 = e_t_ne+in[40];
const double t7 = in[39]*t6;
const double t8 = exp(t7);
const double t9 = t8+1.0;
const double t10 = 1.0/t9;
const double t11 = e_t_ne-in[40];
const double t15 = in[39]*t11;
const double t12 = exp(-t15);
const double t13 = t12+1.0;
const double t14 = 1.0/t13;
const double t16 = t10+t14-1.0;
const double t17 = t10+t14;
const double t18 = e_t_d+in[38];
const double t19 = in[37]*t18;
const double t20 = exp(t19);
const double t21 = t20+1.0;
const double t22 = 1.0/t21;
const double t23 = e_t_d-in[38];
const double t24 = exp(-in[37]*t23);
const double t25 = t24+1.0;
const double t26 = 1.0/t25;

// soft constraints
double a_soft;
if (alpha>(in[42]-in[44])) {
    a_soft=(t3*t3)*t4;
}
else if (alpha>(in[43]+in[44])) {
    a_soft=0.0;
}
else {
    a_soft=t4*(t5*t5);
}

/* outputs */

out[0] = e_t_1_ne;
out[1] = e_t_1_d;
out[2] = in[12];
out[3] = -vbar_n-Td_n*t16+Tpd_n*t17;
out[4] = -vbar_e-Td_e*t16+Tpd_e*t17;
out[5] = -vbar_d-Td_d*(t22+t26-1.0)+Tpd_d*(t22+t26);
out[6] = Vsafe;
out[7] = in[8];
out[8] = in[9];
out[9] = in[10];
out[10] = a_soft;

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
    
    // Tb
    double Tb_n = 1.0;
    double Tb_e = 0.0;
    double Tb_d = 0.0;
    if (norm_ab > 0.1) {
        Tb_n = ab_n / norm_ab;
        Tb_e = ab_e / norm_ab;
        Tb_d = ab_d / norm_ab;
    }

    // p - b
    const double bp_n = pos[0] - params[3];
    const double bp_e = pos[1] - params[4];
    const double bp_d = pos[2] - params[5];
    
    // dot( v , Tb )
    const double dot_vTb = vel[0] * Tb_n + vel[1] * Tb_e + vel[2] * Tb_d;
    
    // dot( (p-b) , Tb )
    const double dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;
    
    // norm( p-b )
    const double norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( (norm_bp < params[17] && dot_vTb > params[18]) || dot_bpTb > 0.0 );
    
}

bool check_curve_seg( const double *pos, const double *vel, const double *params ) {
    
    // chi_b
    const double chi_b = params[6] + params[4] * (params[7] + 1.570796326794897);
        
    // Tb
    const double Tb_n = cos(chi_b)*cos(params[5]);
    const double Tb_e = sin(chi_b)*cos(params[5]);
    const double Tb_d = -sin(params[5]);

    // p - b
    const double bp_n = pos[0] - ( params[0] + params[3] * ( cos(params[6] + params[4] * params[7]) ) );
    const double bp_e = pos[1] - ( params[1] + params[3] * ( sin(params[6] + params[4] * params[7]) ) );
    const double bp_d = pos[2] - ( params[2] - params[3] * tan(params[5]) * params[7] );
    
    // dot( v , Tb )
    const double dot_vTb = vel[0] * Tb_n + vel[1] * Tb_e + vel[2] * Tb_d;
    
    // dot( (p-b) , Tb )
    const double dot_bpTb = bp_n * Tb_n + bp_e * Tb_e + bp_d * Tb_d;
    
    // norm( p-b )
    const double norm_bp = sqrt( bp_n*bp_n + bp_e*bp_e + bp_d*bp_d );
    
    // check (1) proximity, (2) bearing, (3) travel 
    return ( norm_bp < params[17] && dot_vTb > params[18] && dot_bpTb > 0.0 );
}

/* end inline functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
