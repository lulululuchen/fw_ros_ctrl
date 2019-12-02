#include "acado_common.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define EPSILON 0.000001

/* math functions / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
int constrain_int(int x, int xmin, int xmax) {
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
}

double constrain_double(double x, double xmin, double xmax) {
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
}

void cross(double *v, const double v1[3], const double v2[3]) {
    v[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

double dot(const double v1[3], const double v2[3]) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}
/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

/* objective helper functions / / / / / / / / / / / / / / / / / / / / / /*/

/* unit ground speed jacobian */
void jacobian_vg_unit(double *jac_vn, double *jac_ve, double *jac_vd,
        const double gamma, const double one_over_vG_norm, const double v,
        const double vG_d, const double vG_d_unit, const double vG_e,
        const double vG_e_unit, const double vG_n, const double vG_n_unit,
        const double v_d, const double v_e, const double v_n, const double xi)
{

    /* v_n, w.r.t.:
     * v
     * gamma
     * xi
     *
     * v_e, w.r.t.:
     * v
     * gamma
     * xi
     *
     * v_d, w.r.t.:
     * v
     * gamma
     * xi
     */

    const double t2 = cos(gamma);
    const double t3 = cos(xi);
    const double t4 = one_over_vG_norm*one_over_vG_norm;
    const double t5 = sin(gamma);
    const double t6 = sin(xi);
    const double t7 = t2*t3*vG_n*2.0;
    const double t8 = t2*t6*vG_e*2.0;
    const double t16 = t5*vG_d*2.0;
    const double t9 = t7+t8-t16;
    const double t10 = t2*v*vG_d*2.0;
    const double t11 = t3*t5*v*vG_n*2.0;
    const double t12 = t5*t6*v*vG_e*2.0;
    const double t13 = t10+t11+t12;
    const double t14 = vG_n*v_e*2.0;
    const double t17 = vG_e*v_n*2.0;
    const double t15 = t14-t17;

    jac_vn[0] = one_over_vG_norm*t2*t3-t4*t9*vG_n_unit*0.5;
    jac_vn[1] = one_over_vG_norm*t3*v_d+t4*t13*vG_n_unit*0.5;
    jac_vn[2] = -one_over_vG_norm*v_e+t4*t15*vG_n_unit*0.5;

    jac_ve[0] = one_over_vG_norm*t2*t6-t4*t9*vG_e_unit*0.5;
    jac_ve[1] = one_over_vG_norm*t6*v_d+t4*t13*vG_e_unit*0.5;
    jac_ve[2] = one_over_vG_norm*v_n+t4*t15*vG_e_unit*0.5;

    jac_vd[0] = -one_over_vG_norm*t5-t4*t9*vG_d_unit*0.5;
    jac_vd[1] = -one_over_vG_norm*t2*v+t4*t13*vG_d_unit*0.5;
    jac_vd[2] = t4*t15*vG_d_unit*0.5;
}

/* calculate speed states */
void calculate_speed_states(double *speed_states,
        const double v, const double gamma, const double xi,
        const double w_n, const double w_e, const double w_d)
{
    const double v_cos_gamma = v*cos(gamma);
    const double cos_xi = cos(xi);
    const double sin_xi = sin(xi);

    /* airspeed */
    speed_states[0] = v_cos_gamma*cos_xi;       /* v_n */
    speed_states[1] = v_cos_gamma*sin_xi;       /* v_e */
    speed_states[2] = -v*sin(gamma);            /* v_d */

    /* ground speed */
    speed_states[3] = speed_states[0] + w_n;    /* vG_n */
    speed_states[4] = speed_states[1] + w_e;    /* vG_e */
    speed_states[5] = speed_states[2] + w_d;    /* vG_d */
    speed_states[6] = speed_states[3]*speed_states[3] + speed_states[4]*speed_states[4] + speed_states[5]*speed_states[5]; /* vG_sq */
    speed_states[7] = sqrt(speed_states[6]);    /* vG_norm */

    /* unit ground speed */
    speed_states[8] = (speed_states[7] < 0.01) ? 100.0 : 1.0 / speed_states[7];
    speed_states[9] = speed_states[3] * speed_states[8];    /* vG_n_unit */
    speed_states[10] = speed_states[4] * speed_states[8];	/* vG_e_unit */
    speed_states[11] = speed_states[5] * speed_states[8];   /* vG_d_unit */
}
/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

/* evaluation functions / / / / / / / / / / / / / / / / / / / / / / / / /*/
void lsq_obj_eval( const real_t *in, real_t *out, bool eval_end_term )
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* shift inputs by ACADO_NU if evaluating the end term */
    int idx_shift = 0;
    if (eval_end_term) idx_shift = ACADO_NU;

    /* states */
    const double r_n = in[0];
    const double r_e = in[1];
    const double r_d = in[2];
    const double v = in[3];
    const double gamma = in[4];
    const double xi = in[5];
    const double phi = in[6];
    const double theta = in[7];
    const double n_p = in[8];

    /* controls */ /* NOTE: these just dont get used if in end term eval */
    const double u_T = in[9];
    const double phi_ref = in[10];
    const double theta_ref = in[11];

    /* online data */

    /* disturbances */
    const double w_n = in[13-idx_shift];
    const double w_e = in[14-idx_shift];
    const double w_d = in[15-idx_shift];

    /* soft aoa */
    const double sig_aoa = in[22-idx_shift];
    double jac_sig_aoa[2];
    jac_sig_aoa[0] = in[23-idx_shift];
    jac_sig_aoa[1] = in[24-idx_shift];

    /* soft height */
    const double sig_h = in[25-idx_shift];
    double jac_sig_h[4];
    jac_sig_h[0] = in[26-idx_shift];
    jac_sig_h[1] = in[27-idx_shift];
    jac_sig_h[2] = in[28-idx_shift];
    jac_sig_h[3] = in[29-idx_shift];

    /* soft radial */
    const double sig_r = in[30-idx_shift];
    double jac_sig_r[6];
    jac_sig_r[0] = in[31-idx_shift];
    jac_sig_r[1] = in[32-idx_shift];
    jac_sig_r[2] = in[33-idx_shift];
    jac_sig_r[3] = in[34-idx_shift];
    jac_sig_r[4] = in[35-idx_shift];
    jac_sig_r[5] = in[36-idx_shift];


    /* INTERMEDIATE CALCULATIONS - - - - - - - - - - - - - - - - - - - - */

    /* speed states */
    double speed_states[12];
    calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);
    const double v_n = speed_states[0];
    const double v_e = speed_states[1];
    const double v_d = speed_states[2];
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    const double one_over_vG_norm = speed_states[8];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11];


    /* OBJECTIVES - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

    /* state output */
    out[0] = vG_n_unit;
    out[1] = vG_e_unit;
    out[2] = vG_d_unit;
    out[3] = v;
    out[4] = phi;
    out[5] = theta;
    out[6] = sig_aoa;
    out[7] = sig_h;
    out[8] = sig_r;

    /* control output */
    if (!eval_end_term) {
        out[9] = u_T;
        out[10] = phi_ref;
        out[11] = theta_ref;
    }


    /* JACOBIANS - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    if (eval_end_term) {

        /* lsq end term non-zero jacobian evals */
        double jac_v_n[3];
        double jac_v_e[3];
        double jac_v_d[3];
        jacobian_vg_unit(jac_v_n, jac_v_e, jac_v_d,
            gamma, one_over_vG_norm, v, vG_d, vG_d_unit, vG_e, vG_e_unit, vG_n, vG_n_unit, v_d, v_e, v_n, xi);
        double jac_v = 1.0;
        double jac_phi = 1.0;
        double jac_theta = 1.0;

        /* lsq end term jacobian w.r.t. states */
        out[9] = 0.0;
        out[10] = 0.0;
        out[11] = 0.0;
        out[12] = jac_v_n[0];
        out[13] = jac_v_n[1];
        out[14] = jac_v_n[2];
        out[15] = 0.0;
        out[16] = 0.0;
        out[17] = 0.0;
        out[18] = 0.0;
        out[19] = 0.0;
        out[20] = 0.0;
        out[21] = jac_v_e[0];
        out[22] = jac_v_e[1];
        out[23] = jac_v_e[2];
        out[24] = 0.0;
        out[25] = 0.0;
        out[26] = 0.0;
        out[27] = 0.0;
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = jac_v_d[0];
        out[31] = jac_v_d[1];
        out[32] = jac_v_d[2];
        out[33] = 0.0;
        out[34] = 0.0;
        out[35] = 0.0;
        out[36] = 0.0;
        out[37] = 0.0;
        out[38] = 0.0;
        out[39] = jac_v;
        out[40] = 0.0;
        out[41] = 0.0;
        out[42] = 0.0;
        out[43] = 0.0;
        out[44] = 0.0;
        out[45] = 0.0;
        out[46] = 0.0;
        out[47] = 0.0;
        out[48] = 0.0;
        out[49] = 0.0;
        out[50] = 0.0;
        out[51] = jac_phi;
        out[52] = 0.0;
        out[53] = 0.0;
        out[54] = 0.0;
        out[55] = 0.0;
        out[56] = 0.0;
        out[57] = 0.0;
        out[58] = 0.0;
        out[59] = 0.0;
        out[60] = 0.0;
        out[61] = jac_theta;
        out[62] = 0.0;
        out[63] = 0.0;
        out[64] = 0.0;
        out[65] = 0.0;
        out[66] = 0.0;
        out[67] = jac_sig_aoa[0];
        out[68] = 0.0;
        out[69] = 0.0;
        out[70] = jac_sig_aoa[1];
        out[71] = 0.0;
        out[72] = jac_sig_h[0];
        out[73] = jac_sig_h[1];
        out[74] = jac_sig_h[2];
        out[75] = 0.0;
        out[76] = 0.0;
        out[77] = jac_sig_h[3];
        out[78] = 0.0;
        out[79] = 0.0;
        out[80] = 0.0;
        out[81] = jac_sig_r[0];
        out[82] = jac_sig_r[1];
        out[83] = jac_sig_r[2];
        out[84] = jac_sig_r[3];
        out[85] = jac_sig_r[4];
        out[86] = jac_sig_r[5];
        out[87] = 0.0;
        out[88] = 0.0;
        out[89] = 0.0;
    }
    else {

        /* lsq non-zero jacobian evals */
        double jac_v_n[3];
        double jac_v_e[3];
        double jac_v_d[3];
        jacobian_vg_unit(jac_v_n, jac_v_e, jac_v_d,
            gamma, one_over_vG_norm, v, vG_d, vG_d_unit, vG_e, vG_e_unit, vG_n, vG_n_unit, v_d, v_e, v_n, xi);
        double jac_v = 1.0;
        double jac_phi = 1.0;
        double jac_theta = 1.0;
        double jac_uT = 1.0;
        double jac_phi_ref = 1.0;
        double jac_theta_ref = 1.0;

        /* lsq jacobian w.r.t. states */
        out[12] = 0.0;
        out[13] = 0.0;
        out[14] = 0.0;
        out[15] = jac_v_n[0];
        out[16] = jac_v_n[1];
        out[17] = jac_v_n[2];
        out[18] = 0.0;
        out[19] = 0.0;
        out[20] = 0.0;
        out[21] = 0.0;
        out[22] = 0.0;
        out[23] = 0.0;
        out[24] = jac_v_e[0];
        out[25] = jac_v_e[1];
        out[26] = jac_v_e[2];
        out[27] = 0.0;
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = 0.0;
        out[31] = 0.0;
        out[32] = 0.0;
        out[33] = jac_v_d[0];
        out[34] = jac_v_d[1];
        out[35] = jac_v_d[2];
        out[36] = 0.0;
        out[37] = 0.0;
        out[38] = 0.0;
        out[39] = 0.0;
        out[40] = 0.0;
        out[41] = 0.0;
        out[42] = jac_v;
        out[43] = 0.0;
        out[44] = 0.0;
        out[45] = 0.0;
        out[46] = 0.0;
        out[47] = 0.0;
        out[48] = 0.0;
        out[49] = 0.0;
        out[50] = 0.0;
        out[51] = 0.0;
        out[52] = 0.0;
        out[53] = 0.0;
        out[54] = jac_phi;
        out[55] = 0.0;
        out[56] = 0.0;
        out[57] = 0.0;
        out[58] = 0.0;
        out[59] = 0.0;
        out[60] = 0.0;
        out[61] = 0.0;
        out[62] = 0.0;
        out[63] = 0.0;
        out[64] = jac_theta;
        out[65] = 0.0;
        out[66] = 0.0;
        out[67] = 0.0;
        out[68] = 0.0;
        out[69] = 0.0;
        out[70] = jac_sig_aoa[0];
        out[71] = 0.0;
        out[72] = 0.0;
        out[73] = jac_sig_aoa[1];
        out[74] = 0.0;
        out[75] = jac_sig_h[0];
        out[76] = jac_sig_h[1];
        out[77] = jac_sig_h[2];
        out[78] = 0.0;
        out[79] = 0.0;
        out[80] = jac_sig_h[3];
        out[81] = 0.0;
        out[82] = 0.0;
        out[83] = 0.0;
        out[84] = jac_sig_r[0];
        out[85] = jac_sig_r[1];
        out[86] = jac_sig_r[2];
        out[87] = jac_sig_r[3];
        out[88] = jac_sig_r[4];
        out[89] = jac_sig_r[5];
        out[90] = 0.0;
        out[91] = 0.0;
        out[92] = 0.0;
        out[93] = 0.0;
        out[94] = 0.0;
        out[95] = 0.0;
        out[96] = 0.0;
        out[97] = 0.0;
        out[98] = 0.0;
        out[99] = 0.0;
        out[100] = 0.0;
        out[101] = 0.0;
        out[102] = 0.0;
        out[103] = 0.0;
        out[104] = 0.0;
        out[105] = 0.0;
        out[106] = 0.0;
        out[107] = 0.0;
        out[108] = 0.0;
        out[109] = 0.0;
        out[110] = 0.0;
        out[111] = 0.0;
        out[112] = 0.0;
        out[113] = 0.0;
        out[114] = 0.0;
        out[115] = 0.0;
        out[116] = 0.0;
        out[117] = 0.0;
        out[118] = 0.0;
        out[119] = 0.0;

        /* lsq jacobian w.r.t. controls */
        out[120] = 0.0;
        out[121] = 0.0;
        out[122] = 0.0;
        out[123] = 0.0;
        out[124] = 0.0;
        out[125] = 0.0;
        out[126] = 0.0;
        out[127] = 0.0;
        out[128] = 0.0;
        out[129] = 0.0;
        out[130] = 0.0;
        out[131] = 0.0;
        out[132] = 0.0;
        out[133] = 0.0;
        out[134] = 0.0;
        out[135] = 0.0;
        out[136] = 0.0;
        out[137] = 0.0;
        out[138] = 0.0;
        out[139] = 0.0;
        out[140] = 0.0;
        out[141] = 0.0;
        out[142] = 0.0;
        out[143] = 0.0;
        out[144] = 0.0;
        out[145] = 0.0;
        out[146] = 0.0;
        out[147] = jac_uT;
        out[148] = 0.0;
        out[149] = 0.0;
        out[150] = 0.0;
        out[151] = jac_phi_ref;
        out[152] = 0.0;
        out[153] = 0.0;
        out[154] = 0.0;
        out[155] = jac_theta_ref;
    }
}

void acado_evaluateLSQ( const real_t *in, real_t *out )
{
    lsq_obj_eval(in, out, false);
}

void acado_evaluateLSQEndTerm( const real_t *in, real_t *out )
{
	lsq_obj_eval(in, out, true);
}
