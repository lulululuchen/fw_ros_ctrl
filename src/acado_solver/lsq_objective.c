#include "acado_common.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

double wrapPi(double angle)
{
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0) {
        angle += 2*M_PI;
    }

    return angle - M_PI;
} /* wrapPi */

void lsq_obj_eval( const real_t *in, real_t *out, bool eval_end_term )
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* shift inputs by ACADO_NU if evaluating the end term */
    int idx_shift = 0;
    if (eval_end_term) idx_shift = ACADO_NU;

    /* online data */

    /* heading reference */
    const double heading_ref = in[22-idx_shift];

    /* soft airsp */
    const double soft_airsp = in[23-idx_shift];
    const double jac_soft_airsp = in[24-idx_shift];

    /* soft aoa */
    const double soft_aoa = in[25-idx_shift];
    double jac_soft_aoa[2];
    jac_soft_aoa[0] = in[26-idx_shift];
    jac_soft_aoa[1] = in[27-idx_shift];

    /* soft height */
    const double soft_hagl = in[28-idx_shift];
    double jac_soft_hagl[4];
    jac_soft_hagl[0] = in[29-idx_shift];
    jac_soft_hagl[1] = in[30-idx_shift];
    jac_soft_hagl[2] = in[31-idx_shift];
    jac_soft_hagl[3] = in[32-idx_shift];

    /* soft radial */
    const double soft_rtd = in[33-idx_shift];
    double jac_soft_rtd[6];
    jac_soft_rtd[0] = in[34-idx_shift];
    jac_soft_rtd[1] = in[35-idx_shift];
    jac_soft_rtd[2] = in[36-idx_shift];
    jac_soft_rtd[3] = in[37-idx_shift];
    jac_soft_rtd[4] = in[38-idx_shift];
    jac_soft_rtd[5] = in[39-idx_shift];

    /* OBJECTIVES - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

    /* state output */
    out[0] = in[0]; /* position north */
    out[1] = in[1]; /* position east */
    out[2] = in[3]; /* airspeed */
    out[3] = in[4]; /* flight path angle */
    out[4] = wrapPi(in[5] - heading_ref); /* heading error */
    out[5] = soft_airsp;
    out[6] = soft_aoa;
    out[7] = soft_hagl;
    out[8] = soft_rtd;

    /* control output */
    if (!eval_end_term) {
        out[9] = in[9]; /* throttle input */
        out[10] = in[10]; /* roll_ref */
        out[11] = in[11]; /* pitch_ref */
    }


    /* JACOBIANS - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    if (eval_end_term) {

        /* lsq end term non-zero jacobian evals */
        const double jac_pos_n = 1.0;
        const double jac_pos_e = 1.0;
        const double jac_airsp = 1.0;
        const double jac_fpa = 1.0;
        const double jac_heading = 1.0;

        /* lsq end term jacobian w.r.t. states */
        out[9] = jac_pos_n; /* - - - - - - - - - - - - - - d(pos_n)/d(x) */
        out[10] = 0.0;
        out[11] = 0.0;
        out[12] = 0.0;
        out[13] = 0.0;
        out[14] = 0.0;
        out[15] = 0.0;
        out[16] = 0.0;
        out[17] = 0.0;
        out[18] = 0.0; /* - - - - - - - - - - - - - - - - -d(pos_e)/d(x) */
        out[19] = jac_pos_e;
        out[20] = 0.0;
        out[21] = 0.0;
        out[22] = 0.0;
        out[23] = 0.0;
        out[24] = 0.0;
        out[25] = 0.0;
        out[26] = 0.0;
        out[27] = 0.0; /* - - - - - - - - - - - - - - - - -d(airsp)/d(x) */
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = jac_airsp;
        out[31] = 0.0;
        out[32] = 0.0;
        out[33] = 0.0;
        out[34] = 0.0;
        out[35] = 0.0;
        out[36] = 0.0; /* - - - - - - - - - - - - - - - - - -d(fpa)/d(x) */
        out[37] = 0.0;
        out[38] = 0.0;
        out[39] = 0.0;
        out[40] = jac_fpa;
        out[41] = 0.0;
        out[42] = 0.0;
        out[43] = 0.0;
        out[44] = 0.0;
        out[45] = 0.0; /* - - - - - - - - - - - - - - - -d(heading)/d(x) */
        out[46] = 0.0;
        out[47] = 0.0;
        out[48] = 0.0;
        out[49] = 0.0;
        out[50] = jac_heading;
        out[51] = 0.0;
        out[52] = 0.0;
        out[53] = 0.0;
        out[54] = 0.0; /* - - - - - - - - - - - - - - d(soft_airsp)/d(x) */
        out[55] = 0.0;
        out[56] = 0.0;
        out[57] = jac_soft_airsp;
        out[58] = 0.0;
        out[59] = 0.0;
        out[60] = 0.0;
        out[61] = 0.0;
        out[62] = 0.0;
        out[63] = 0.0; /* - - - - - - - - - - - - - - - d(soft_aoa)/d(x) */
        out[64] = 0.0;
        out[65] = 0.0;
        out[66] = 0.0;
        out[67] = jac_soft_aoa[0];
        out[68] = 0.0;
        out[69] = 0.0;
        out[70] = jac_soft_aoa[1];
        out[71] = 0.0;
        out[72] = jac_soft_hagl[0]; /* - - - - - - - - d(soft_hagl)/d(x) */
        out[73] = jac_soft_hagl[1];
        out[74] = jac_soft_hagl[2];
        out[75] = 0.0;
        out[76] = 0.0;
        out[77] = jac_soft_hagl[3];
        out[78] = 0.0;
        out[79] = 0.0;
        out[80] = 0.0;
        out[81] = jac_soft_rtd[0]; /* - - - - - - - - - d(soft_rtd)/d(x) */
        out[82] = jac_soft_rtd[1];
        out[83] = jac_soft_rtd[2];
        out[84] = jac_soft_rtd[3];
        out[85] = jac_soft_rtd[4];
        out[86] = jac_soft_rtd[5];
        out[87] = 0.0;
        out[88] = 0.0;
        out[89] = 0.0;
    }
    else {

        /* lsq non-zero jacobian evals */
        const double jac_pos_n = 1.0;
        const double jac_pos_e = 1.0;
        const double jac_airsp = 1.0;
        const double jac_fpa = 1.0;
        const double jac_heading = 1.0;
        const double jac_throt = 1.0;
        const double jac_pitch_ref = 1.0;
        const double jac_roll_ref = 1.0;

        /* lsq jacobian w.r.t. states */
        out[12] = jac_pos_n; /* - - - - - - - - - - - - - -d(pos_n)/d(x) */
        out[13] = 0.0;
        out[14] = 0.0;
        out[15] = 0.0;
        out[16] = 0.0;
        out[17] = 0.0;
        out[18] = 0.0;
        out[19] = 0.0;
        out[20] = 0.0;
        out[21] = 0.0; /* - - - - - - - - - - - - - - - - -d(pos_e)/d(x) */
        out[22] = jac_pos_e;
        out[23] = 0.0;
        out[24] = 0.0;
        out[25] = 0.0;
        out[26] = 0.0;
        out[27] = 0.0;
        out[28] = 0.0;
        out[29] = 0.0;
        out[30] = 0.0; /* - - - - - - - - - - - - - - - - -d(airsp)/d(x) */
        out[31] = 0.0;
        out[32] = 0.0;
        out[33] = jac_airsp;
        out[34] = 0.0;
        out[35] = 0.0;
        out[36] = 0.0;
        out[37] = 0.0;
        out[38] = 0.0;
        out[39] = 0.0; /* - - - - - - - - - - - - - - - - - -d(fpa)/d(x) */
        out[40] = 0.0;
        out[41] = 0.0;
        out[42] = 0.0;
        out[43] = jac_fpa;
        out[44] = 0.0;
        out[45] = 0.0;
        out[46] = 0.0;
        out[47] = 0.0;
        out[48] = 0.0; /* - - - - - - - - - - - - - - - -d(heading)/d(x) */
        out[49] = 0.0;
        out[50] = 0.0;
        out[51] = 0.0;
        out[52] = 0.0;
        out[53] = jac_heading;
        out[54] = 0.0;
        out[55] = 0.0;
        out[56] = 0.0;
        out[57] = 0.0; /* - - - - - - - - - - - - - - d(soft_airsp)/d(x) */
        out[58] = 0.0;
        out[59] = 0.0;
        out[60] = jac_soft_airsp;
        out[61] = 0.0;
        out[62] = 0.0;
        out[63] = 0.0;
        out[64] = 0.0;
        out[65] = 0.0;
        out[66] = 0.0; /* - - - - - - - - - - - - - - - d(soft_aoa)/d(x) */
        out[67] = 0.0;
        out[68] = 0.0;
        out[69] = 0.0;
        out[70] = jac_soft_aoa[0];
        out[71] = 0.0;
        out[72] = 0.0;
        out[73] = jac_soft_aoa[1];
        out[74] = 0.0;
        out[75] = jac_soft_hagl[0]; /* - - - - - - - - d(soft_hagl)/d(x) */
        out[76] = jac_soft_hagl[1];
        out[77] = jac_soft_hagl[2];
        out[78] = 0.0;
        out[79] = 0.0;
        out[80] = jac_soft_hagl[3];
        out[81] = 0.0;
        out[82] = 0.0;
        out[83] = 0.0;
        out[84] = jac_soft_rtd[0]; /* - - - - - - - - - d(soft_rtd)/d(x) */
        out[85] = jac_soft_rtd[1];
        out[86] = jac_soft_rtd[2];
        out[87] = jac_soft_rtd[3];
        out[88] = jac_soft_rtd[4];
        out[89] = jac_soft_rtd[5];
        out[90] = 0.0;
        out[91] = 0.0;
        out[92] = 0.0;
        out[93] = 0.0; /* - - - - - - - - - - - - - - - - -d(throt)/d(x) */
        out[94] = 0.0;
        out[95] = 0.0;
        out[96] = 0.0;
        out[97] = 0.0;
        out[98] = 0.0;
        out[99] = 0.0;
        out[100] = 0.0;
        out[101] = 0.0;
        out[102] = 0.0; /* - - - - - - - - - - - - - - -d(roll_ref)/d(x) */
        out[103] = 0.0;
        out[104] = 0.0;
        out[105] = 0.0;
        out[106] = 0.0;
        out[107] = 0.0;
        out[108] = 0.0;
        out[109] = 0.0;
        out[110] = 0.0;
        out[111] = 0.0; /* - - - - - - - - - - - - - - d(pitch_ref)/d(x) */
        out[112] = 0.0;
        out[113] = 0.0;
        out[114] = 0.0;
        out[115] = 0.0;
        out[116] = 0.0;
        out[117] = 0.0;
        out[118] = 0.0;
        out[119] = 0.0;

        /* lsq jacobian w.r.t. controls */
        out[120] = 0.0; /* - - - - - - - - - - - - - - - - d(pos_n)/d(u) */
        out[121] = 0.0;
        out[122] = 0.0;
        out[123] = 0.0; /* - - - - - - - - - - - - - - - - d(pos_e)/d(u) */
        out[124] = 0.0;
        out[125] = 0.0;
        out[126] = 0.0; /* - - - - - - - - - - - - - - - - d(airsp)/d(u) */
        out[127] = 0.0;
        out[128] = 0.0;
        out[129] = 0.0; /* - - - - - - - - - - - - - - - - - d(fpa)/d(u) */
        out[130] = 0.0;
        out[131] = 0.0;
        out[132] = 0.0; /* - - - - - - - - - - - - - - - d(heading)/d(u) */
        out[133] = 0.0;
        out[134] = 0.0;
        out[135] = 0.0; /* - - - - - - - - - - - - - -d(soft_airsp)/d(u) */
        out[136] = 0.0;
        out[137] = 0.0;
        out[138] = 0.0; /* - - - - - - - - - - - - - - -d(soft_aoa)/d(u) */
        out[139] = 0.0;
        out[140] = 0.0;
        out[141] = 0.0; /* - - - - - - - - - - - - - - d(soft_hagl)/d(u) */
        out[142] = 0.0;
        out[143] = 0.0;
        out[144] = 0.0; /* - - - - - - - - - - - - - - -d(soft_rtd)/d(u) */
        out[145] = 0.0;
        out[146] = 0.0;
        out[147] = jac_throt; /* - - - - - - - - - - - - - d(throt)/d(u) */
        out[148] = 0.0;
        out[149] = 0.0;
        out[150] = 0.0; /* - - - - - - - - - - - - - - -d(roll_ref)/d(u) */
        out[151] = jac_roll_ref;
        out[152] = 0.0;
        out[153] = 0.0; /* - - - - - - - - - - - - - - -(pitch_ref)/d(u) */
        out[154] = 0.0;
        out[155] = jac_pitch_ref;
    }
} /* lsq_obj_eval */

void acado_evaluateLSQ( const real_t *in, real_t *out )
{
    lsq_obj_eval(in, out, false);
} /* acado_evaluateLSQ */

void acado_evaluateLSQEndTerm( const real_t *in, real_t *out )
{
    lsq_obj_eval(in, out, true);
} /* acado_evaluateLSQEndTerm */
