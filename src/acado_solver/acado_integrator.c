/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 26. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[5]));
a[2] = (cos(xd[4]));
a[3] = (sin(xd[5]));
a[4] = (sin(xd[4]));
a[5] = ((od[9])*(od[9]));
a[6] = (pow((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7]),2));
a[7] = ((xd[3])*(xd[3]));
a[8] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[9] = (cos((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[10] = ((xd[8])*(xd[8]));
a[11] = (sin(xd[4]));
a[12] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[13] = ((xd[8])*(xd[8]));
a[14] = (sin((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[15] = ((xd[3])*(xd[3]));
a[16] = (cos(xd[6]));
a[17] = (cos(xd[4]));
a[18] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[19] = ((xd[8])*(xd[8]));
a[20] = (sin((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[21] = ((xd[3])*(xd[3]));
a[22] = (cos(xd[4]));
a[23] = (sin(xd[6]));
a[24] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[25] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));

/* Compute outputs: */
out[0] = (((xd[3]*a[0])*a[1])+od[1]);
out[1] = (((xd[3]*a[2])*a[3])+od[2]);
out[2] = ((((real_t)(0.0000000000000000e+00)-xd[3])*a[4])+od[3]);
out[3] = (((((((((real_t)(0.0000000000000000e+00)-(((((((real_t)(-8.9899999999999994e-02)*od[9])+((real_t)(5.0763999999999998e-01)*a[5]))*(real_t)(2.0000000000000000e+00))+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(3.4738000000000002e-01)))+((real_t)(2.5991999999999998e-01)*a[6]))+(real_t)(6.0314000000000000e-02)))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[7])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[8])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[9])*a[10]))*(real_t)(3.2010243277848910e-01))-((real_t)(9.8100000000000005e+00)*a[11]));
out[4] = ((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[12])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[13])*a[14])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[15]))*a[16])-((real_t)(3.0646440000000002e+01)*a[17]))*(real_t)(3.2010243277848910e-01))/xd[3]);
out[5] = ((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[18])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[19])*a[20])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[21]))/(real_t)(3.1240000000000001e+00))/xd[3])/a[22])*a[23]);
out[6] = (((od[6]*u[1])-xd[6])/od[4]);
out[7] = (((od[7]*u[2])-xd[7])/od[5]);
out[8] = ((((((((real_t)(0.0000000000000000e+00)-((real_t)(-1.0000000000000000e+01)+(xd[3]*a[24])))/(real_t)(1.5000000000000000e+01))+(real_t)(1.0000000000000000e+00))*(((real_t)(1.2276762909764899e+02)*u[0])+(real_t)(4.0899037569017665e+01)))+((((real_t)(-1.0000000000000000e+01)+(xd[3]*a[25]))*((real_t)(1.0269363533329036e+02)+((real_t)(6.0973031333376298e+01)*u[0])))/(real_t)(1.5000000000000000e+01)))-xd[8])/od[8]);
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 81. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[5]));
a[2] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[3] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[4] = (cos(xd[4]));
a[5] = (sin(xd[5]));
a[6] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[7] = (cos(xd[5]));
a[8] = (sin(xd[4]));
a[9] = (cos(xd[4]));
a[10] = ((od[9])*(od[9]));
a[11] = (pow((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7]),2));
a[12] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[13] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[14] = ((real_t)(1.0000000000000000e+00)/xd[8]);
a[15] = (cos((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[16] = ((xd[8])*(xd[8]));
a[17] = ((real_t)(2.0000000000000000e+00)*(((real_t)(0.0000000000000000e+00)-xd[4])+xd[7]));
a[18] = (a[17]*((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00)));
a[19] = ((xd[3])*(xd[3]));
a[20] = ((real_t)(-1.0000000000000000e+00)*(sin((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7]))));
a[21] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[20]);
a[22] = ((real_t)(-1.0000000000000000e+00)*(sin((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7]))));
a[23] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[22]);
a[24] = (cos(xd[4]));
a[25] = (a[14]*a[14]);
a[26] = ((real_t)(2.0000000000000000e+00)*xd[8]);
a[27] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[28] = ((real_t)(1.0000000000000000e+00)/xd[8]);
a[29] = ((xd[8])*(xd[8]));
a[30] = (sin((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[31] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[32] = (cos(xd[6]));
a[33] = ((real_t)(1.0000000000000000e+00)/xd[3]);
a[34] = ((xd[3])*(xd[3]));
a[35] = (cos(xd[4]));
a[36] = (a[33]*a[33]);
a[37] = ((real_t)(-1.0000000000000000e+00)*(sin((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7]))));
a[38] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[37]);
a[39] = (cos((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[40] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[39]);
a[41] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[42] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[43] = (a[28]*a[28]);
a[44] = ((real_t)(2.0000000000000000e+00)*xd[8]);
a[45] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[46] = ((real_t)(1.0000000000000000e+00)/xd[8]);
a[47] = ((xd[8])*(xd[8]));
a[48] = (sin((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[49] = ((real_t)(2.0000000000000000e+00)*xd[3]);
a[50] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.1240000000000001e+00));
a[51] = ((real_t)(1.0000000000000000e+00)/xd[3]);
a[52] = ((xd[3])*(xd[3]));
a[53] = (a[51]*a[51]);
a[54] = (cos(xd[4]));
a[55] = ((real_t)(1.0000000000000000e+00)/a[54]);
a[56] = (sin(xd[6]));
a[57] = ((real_t)(-1.0000000000000000e+00)*(sin((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7]))));
a[58] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[57]);
a[59] = (cos((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])));
a[60] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[59]);
a[61] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[62] = (a[55]*a[55]);
a[63] = (cos(xd[6]));
a[64] = (a[46]*a[46]);
a[65] = ((real_t)(2.0000000000000000e+00)*xd[8]);
a[66] = ((real_t)(1.0000000000000000e+00)/od[4]);
a[67] = ((real_t)(1.0000000000000000e+00)/od[4]);
a[68] = ((real_t)(1.0000000000000000e+00)/od[5]);
a[69] = ((real_t)(1.0000000000000000e+00)/od[5]);
a[70] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[71] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.5000000000000000e+01));
a[72] = (cos((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7])));
a[73] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.5000000000000000e+01));
a[74] = ((real_t)(1.0000000000000000e+00)/od[8]);
a[75] = ((real_t)(-1.0000000000000000e+00)*(sin((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7]))));
a[76] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[75]);
a[77] = ((real_t)(-1.0000000000000000e+00)*(sin((((real_t)(-3.4906585039886591e-02)-xd[4])+xd[7]))));
a[78] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[77]);
a[79] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.5000000000000000e+01));
a[80] = ((real_t)(1.0000000000000000e+00)/od[8]);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (a[0]*a[1]);
out[4] = ((xd[3]*a[2])*a[1]);
out[5] = ((xd[3]*a[0])*a[3]);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (a[4]*a[5]);
out[16] = ((xd[3]*a[6])*a[5]);
out[17] = ((xd[3]*a[4])*a[7]);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[8]);
out[28] = (((real_t)(0.0000000000000000e+00)-xd[3])*a[9]);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = ((((((((real_t)(0.0000000000000000e+00)-(((((((real_t)(-8.9899999999999994e-02)*od[9])+((real_t)(5.0763999999999998e-01)*a[10]))*(real_t)(2.0000000000000000e+00))+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(3.4738000000000002e-01)))+((real_t)(2.5991999999999998e-01)*a[11]))+(real_t)(6.0314000000000000e-02)))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[12])+(((((((real_t)(-4.7364285714285703e-01)*a[13])*a[14])*(real_t)(6.1465600000000023e-03))*od[0])*a[15])*a[16]))*(real_t)(3.2010243277848910e-01));
out[40] = (((((((((real_t)(0.0000000000000000e+00)-(((real_t)(-3.4738000000000002e-01))+((real_t)(2.5991999999999998e-01)*a[18])))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[19])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[21])*a[14])*(real_t)(6.1465600000000023e-03))*od[0])*a[15])+((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[13])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[23]))*a[16]))*(real_t)(3.2010243277848910e-01))-((real_t)(9.8100000000000005e+00)*a[24]));
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = ((((((((real_t)(0.0000000000000000e+00)-((real_t)(3.4738000000000002e-01)+((real_t)(2.5991999999999998e-01)*a[17])))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[19])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[20])*a[14])*(real_t)(6.1465600000000023e-03))*od[0])*a[15])+((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[13])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[22]))*a[16]))*(real_t)(3.2010243277848910e-01));
out[44] = ((((((((real_t)(0.0000000000000000e+00)-((((real_t)(-4.7364285714285703e-01)*xd[3])*a[13])*a[25]))*(real_t)(6.1465600000000023e-03))*od[0])*a[15])*a[16])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[13])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[15])*a[26]))*(real_t)(3.2010243277848910e-01));
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = ((((((((((((real_t)(-4.7364285714285703e-01)*a[27])*a[28])*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[30])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[31]))*a[32])*(real_t)(3.2010243277848910e-01))*a[33])-((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[27])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[30])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[34]))*a[32])-((real_t)(3.0646440000000002e+01)*a[35]))*(real_t)(3.2010243277848910e-01))*a[36]));
out[52] = ((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[38])*a[28])*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[30])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[27])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[40]))+((((real_t)(-5.9567089999999989e-01))*od[0])*a[34]))*a[32])-((real_t)(3.0646440000000002e+01)*a[41]))*(real_t)(3.2010243277848910e-01))*a[33]);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[27])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[30])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[34]))*a[42])*(real_t)(3.2010243277848910e-01))*a[33]);
out[55] = (((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[37])*a[28])*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[30])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[27])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[29])*a[39]))+((((real_t)(5.9567089999999989e-01))*od[0])*a[34]))*a[32])*(real_t)(3.2010243277848910e-01))*a[33]);
out[56] = ((((((((((real_t)(0.0000000000000000e+00)-((((real_t)(-4.7364285714285703e-01)*xd[3])*a[27])*a[43]))*(real_t)(6.1465600000000023e-03))*od[0])*a[29])+((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[27])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[44]))*a[30])*a[32])*(real_t)(3.2010243277848910e-01))*a[33]);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (((((((((((((real_t)(-4.7364285714285703e-01)*a[45])*a[46])*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[48])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[49]))*a[50])*a[51])-((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[48])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[52]))/(real_t)(3.1240000000000001e+00))*a[53]))*a[55])*a[56]);
out[64] = (((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[58])*a[46])*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[48])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[60]))+((((real_t)(-5.9567089999999989e-01))*od[0])*a[52]))*a[50])*a[51])*a[55])-((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[48])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[52]))/(real_t)(3.1240000000000001e+00))/xd[3])*a[61])*a[62]))*a[56]);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = ((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[48])+((((((((real_t)(-2.0379999999999998e+00)*od[9])+((((real_t)(0.0000000000000000e+00)-xd[4])+xd[7])*(real_t)(2.8500999999999999e+00)))+(real_t)(5.2154999999999996e-01))*(real_t)(4.1799999999999998e-01))*(real_t)(5.0000000000000000e-01))*od[0])*a[52]))/(real_t)(3.1240000000000001e+00))/xd[3])/a[54])*a[63]);
out[67] = ((((((((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[57])*a[46])*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[48])+(((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[47])*a[59]))+((((real_t)(5.9567089999999989e-01))*od[0])*a[52]))*a[50])*a[51])*a[55])*a[56]);
out[68] = (((((((((((real_t)(0.0000000000000000e+00)-((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])*a[64]))*(real_t)(6.1465600000000023e-03))*od[0])*a[47])+((((((((real_t)(-4.7364285714285703e-01)*xd[3])*a[45])/xd[8])+(real_t)(1.1521000000000001e-01))*(real_t)(6.1465600000000023e-03))*od[0])*a[65]))*a[48])*a[50])*a[51])*a[55])*a[56]);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[66]);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (od[6]*a[67]);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[68]);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (od[7]*a[69]);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = ((((((real_t)(0.0000000000000000e+00)-a[70])*a[71])*(((real_t)(1.2276762909764899e+02)*u[0])+(real_t)(4.0899037569017665e+01)))+((a[72]*((real_t)(1.0269363533329036e+02)+((real_t)(6.0973031333376298e+01)*u[0])))*a[73]))*a[74]);
out[100] = ((((((real_t)(0.0000000000000000e+00)-(xd[3]*a[76]))*a[71])*(((real_t)(1.2276762909764899e+02)*u[0])+(real_t)(4.0899037569017665e+01)))+(((xd[3]*a[78])*((real_t)(1.0269363533329036e+02)+((real_t)(6.0973031333376298e+01)*u[0])))*a[73]))*a[74]);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = ((((((real_t)(0.0000000000000000e+00)-(xd[3]*a[75]))*a[71])*(((real_t)(1.2276762909764899e+02)*u[0])+(real_t)(4.0899037569017665e+01)))+(((xd[3]*a[77])*((real_t)(1.0269363533329036e+02)+((real_t)(6.0973031333376298e+01)*u[0])))*a[73]))*a[74]);
out[104] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[74]);
out[105] = (((((((real_t)(0.0000000000000000e+00)-((real_t)(-1.0000000000000000e+01)+(xd[3]*a[70])))/(real_t)(1.5000000000000000e+01))+(real_t)(1.0000000000000000e+00))*(real_t)(1.2276762909764899e+02))+((((real_t)(-1.0000000000000000e+01)+(xd[3]*a[72]))*(real_t)(6.0973031333376298e+01))*a[79]))*a[80]);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim18_triangular( real_t* const A, real_t* const b )
{

b[17] = b[17]/A[323];
b[16] -= + A[305]*b[17];
b[16] = b[16]/A[304];
b[15] -= + A[287]*b[17];
b[15] -= + A[286]*b[16];
b[15] = b[15]/A[285];
b[14] -= + A[269]*b[17];
b[14] -= + A[268]*b[16];
b[14] -= + A[267]*b[15];
b[14] = b[14]/A[266];
b[13] -= + A[251]*b[17];
b[13] -= + A[250]*b[16];
b[13] -= + A[249]*b[15];
b[13] -= + A[248]*b[14];
b[13] = b[13]/A[247];
b[12] -= + A[233]*b[17];
b[12] -= + A[232]*b[16];
b[12] -= + A[231]*b[15];
b[12] -= + A[230]*b[14];
b[12] -= + A[229]*b[13];
b[12] = b[12]/A[228];
b[11] -= + A[215]*b[17];
b[11] -= + A[214]*b[16];
b[11] -= + A[213]*b[15];
b[11] -= + A[212]*b[14];
b[11] -= + A[211]*b[13];
b[11] -= + A[210]*b[12];
b[11] = b[11]/A[209];
b[10] -= + A[197]*b[17];
b[10] -= + A[196]*b[16];
b[10] -= + A[195]*b[15];
b[10] -= + A[194]*b[14];
b[10] -= + A[193]*b[13];
b[10] -= + A[192]*b[12];
b[10] -= + A[191]*b[11];
b[10] = b[10]/A[190];
b[9] -= + A[179]*b[17];
b[9] -= + A[178]*b[16];
b[9] -= + A[177]*b[15];
b[9] -= + A[176]*b[14];
b[9] -= + A[175]*b[13];
b[9] -= + A[174]*b[12];
b[9] -= + A[173]*b[11];
b[9] -= + A[172]*b[10];
b[9] = b[9]/A[171];
b[8] -= + A[161]*b[17];
b[8] -= + A[160]*b[16];
b[8] -= + A[159]*b[15];
b[8] -= + A[158]*b[14];
b[8] -= + A[157]*b[13];
b[8] -= + A[156]*b[12];
b[8] -= + A[155]*b[11];
b[8] -= + A[154]*b[10];
b[8] -= + A[153]*b[9];
b[8] = b[8]/A[152];
b[7] -= + A[143]*b[17];
b[7] -= + A[142]*b[16];
b[7] -= + A[141]*b[15];
b[7] -= + A[140]*b[14];
b[7] -= + A[139]*b[13];
b[7] -= + A[138]*b[12];
b[7] -= + A[137]*b[11];
b[7] -= + A[136]*b[10];
b[7] -= + A[135]*b[9];
b[7] -= + A[134]*b[8];
b[7] = b[7]/A[133];
b[6] -= + A[125]*b[17];
b[6] -= + A[124]*b[16];
b[6] -= + A[123]*b[15];
b[6] -= + A[122]*b[14];
b[6] -= + A[121]*b[13];
b[6] -= + A[120]*b[12];
b[6] -= + A[119]*b[11];
b[6] -= + A[118]*b[10];
b[6] -= + A[117]*b[9];
b[6] -= + A[116]*b[8];
b[6] -= + A[115]*b[7];
b[6] = b[6]/A[114];
b[5] -= + A[107]*b[17];
b[5] -= + A[106]*b[16];
b[5] -= + A[105]*b[15];
b[5] -= + A[104]*b[14];
b[5] -= + A[103]*b[13];
b[5] -= + A[102]*b[12];
b[5] -= + A[101]*b[11];
b[5] -= + A[100]*b[10];
b[5] -= + A[99]*b[9];
b[5] -= + A[98]*b[8];
b[5] -= + A[97]*b[7];
b[5] -= + A[96]*b[6];
b[5] = b[5]/A[95];
b[4] -= + A[89]*b[17];
b[4] -= + A[88]*b[16];
b[4] -= + A[87]*b[15];
b[4] -= + A[86]*b[14];
b[4] -= + A[85]*b[13];
b[4] -= + A[84]*b[12];
b[4] -= + A[83]*b[11];
b[4] -= + A[82]*b[10];
b[4] -= + A[81]*b[9];
b[4] -= + A[80]*b[8];
b[4] -= + A[79]*b[7];
b[4] -= + A[78]*b[6];
b[4] -= + A[77]*b[5];
b[4] = b[4]/A[76];
b[3] -= + A[71]*b[17];
b[3] -= + A[70]*b[16];
b[3] -= + A[69]*b[15];
b[3] -= + A[68]*b[14];
b[3] -= + A[67]*b[13];
b[3] -= + A[66]*b[12];
b[3] -= + A[65]*b[11];
b[3] -= + A[64]*b[10];
b[3] -= + A[63]*b[9];
b[3] -= + A[62]*b[8];
b[3] -= + A[61]*b[7];
b[3] -= + A[60]*b[6];
b[3] -= + A[59]*b[5];
b[3] -= + A[58]*b[4];
b[3] = b[3]/A[57];
b[2] -= + A[53]*b[17];
b[2] -= + A[52]*b[16];
b[2] -= + A[51]*b[15];
b[2] -= + A[50]*b[14];
b[2] -= + A[49]*b[13];
b[2] -= + A[48]*b[12];
b[2] -= + A[47]*b[11];
b[2] -= + A[46]*b[10];
b[2] -= + A[45]*b[9];
b[2] -= + A[44]*b[8];
b[2] -= + A[43]*b[7];
b[2] -= + A[42]*b[6];
b[2] -= + A[41]*b[5];
b[2] -= + A[40]*b[4];
b[2] -= + A[39]*b[3];
b[2] = b[2]/A[38];
b[1] -= + A[35]*b[17];
b[1] -= + A[34]*b[16];
b[1] -= + A[33]*b[15];
b[1] -= + A[32]*b[14];
b[1] -= + A[31]*b[13];
b[1] -= + A[30]*b[12];
b[1] -= + A[29]*b[11];
b[1] -= + A[28]*b[10];
b[1] -= + A[27]*b[9];
b[1] -= + A[26]*b[8];
b[1] -= + A[25]*b[7];
b[1] -= + A[24]*b[6];
b[1] -= + A[23]*b[5];
b[1] -= + A[22]*b[4];
b[1] -= + A[21]*b[3];
b[1] -= + A[20]*b[2];
b[1] = b[1]/A[19];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim18_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 18; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (17); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*18+i]);
	for( j=(i+1); j < 18; j++ ) {
		temp = fabs(A[j*18+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 18; ++k)
{
	acadoWorkspace.rk_dim18_swap = A[i*18+k];
	A[i*18+k] = A[indexMax*18+k];
	A[indexMax*18+k] = acadoWorkspace.rk_dim18_swap;
}
	acadoWorkspace.rk_dim18_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim18_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*18+i];
	for( j=i+1; j < 18; j++ ) {
		A[j*18+i] = -A[j*18+i]/A[i*18+i];
		for( k=i+1; k < 18; k++ ) {
			A[j*18+k] += A[j*18+i] * A[i*18+k];
		}
		b[j] += A[j*18+i] * b[i];
	}
}
det *= A[323];
det = fabs(det);
acado_solve_dim18_triangular( A, b );
return det;
}

void acado_solve_dim18_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim18_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim18_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim18_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim18_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim18_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim18_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim18_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim18_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim18_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim18_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim18_bPerm[10] = b[rk_perm[10]];
acadoWorkspace.rk_dim18_bPerm[11] = b[rk_perm[11]];
acadoWorkspace.rk_dim18_bPerm[12] = b[rk_perm[12]];
acadoWorkspace.rk_dim18_bPerm[13] = b[rk_perm[13]];
acadoWorkspace.rk_dim18_bPerm[14] = b[rk_perm[14]];
acadoWorkspace.rk_dim18_bPerm[15] = b[rk_perm[15]];
acadoWorkspace.rk_dim18_bPerm[16] = b[rk_perm[16]];
acadoWorkspace.rk_dim18_bPerm[17] = b[rk_perm[17]];
acadoWorkspace.rk_dim18_bPerm[1] += A[18]*acadoWorkspace.rk_dim18_bPerm[0];

acadoWorkspace.rk_dim18_bPerm[2] += A[36]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[2] += A[37]*acadoWorkspace.rk_dim18_bPerm[1];

acadoWorkspace.rk_dim18_bPerm[3] += A[54]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[3] += A[55]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[3] += A[56]*acadoWorkspace.rk_dim18_bPerm[2];

acadoWorkspace.rk_dim18_bPerm[4] += A[72]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[4] += A[73]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[4] += A[74]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[4] += A[75]*acadoWorkspace.rk_dim18_bPerm[3];

acadoWorkspace.rk_dim18_bPerm[5] += A[90]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[5] += A[91]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[5] += A[92]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[5] += A[93]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[5] += A[94]*acadoWorkspace.rk_dim18_bPerm[4];

acadoWorkspace.rk_dim18_bPerm[6] += A[108]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[6] += A[109]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[6] += A[110]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[6] += A[111]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[6] += A[112]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[6] += A[113]*acadoWorkspace.rk_dim18_bPerm[5];

acadoWorkspace.rk_dim18_bPerm[7] += A[126]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[7] += A[127]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[7] += A[128]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[7] += A[129]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[7] += A[130]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[7] += A[131]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[7] += A[132]*acadoWorkspace.rk_dim18_bPerm[6];

acadoWorkspace.rk_dim18_bPerm[8] += A[144]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[8] += A[145]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[8] += A[146]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[8] += A[147]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[8] += A[148]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[8] += A[149]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[8] += A[150]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[8] += A[151]*acadoWorkspace.rk_dim18_bPerm[7];

acadoWorkspace.rk_dim18_bPerm[9] += A[162]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[9] += A[163]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[9] += A[164]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[9] += A[165]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[9] += A[166]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[9] += A[167]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[9] += A[168]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[9] += A[169]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[9] += A[170]*acadoWorkspace.rk_dim18_bPerm[8];

acadoWorkspace.rk_dim18_bPerm[10] += A[180]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[10] += A[181]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[10] += A[182]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[10] += A[183]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[10] += A[184]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[10] += A[185]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[10] += A[186]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[10] += A[187]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[10] += A[188]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[10] += A[189]*acadoWorkspace.rk_dim18_bPerm[9];

acadoWorkspace.rk_dim18_bPerm[11] += A[198]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[11] += A[199]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[11] += A[200]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[11] += A[201]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[11] += A[202]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[11] += A[203]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[11] += A[204]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[11] += A[205]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[11] += A[206]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[11] += A[207]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[11] += A[208]*acadoWorkspace.rk_dim18_bPerm[10];

acadoWorkspace.rk_dim18_bPerm[12] += A[216]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[12] += A[217]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[12] += A[218]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[12] += A[219]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[12] += A[220]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[12] += A[221]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[12] += A[222]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[12] += A[223]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[12] += A[224]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[12] += A[225]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[12] += A[226]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[12] += A[227]*acadoWorkspace.rk_dim18_bPerm[11];

acadoWorkspace.rk_dim18_bPerm[13] += A[234]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[13] += A[235]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[13] += A[236]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[13] += A[237]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[13] += A[238]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[13] += A[239]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[13] += A[240]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[13] += A[241]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[13] += A[242]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[13] += A[243]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[13] += A[244]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[13] += A[245]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[13] += A[246]*acadoWorkspace.rk_dim18_bPerm[12];

acadoWorkspace.rk_dim18_bPerm[14] += A[252]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[14] += A[253]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[14] += A[254]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[14] += A[255]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[14] += A[256]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[14] += A[257]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[14] += A[258]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[14] += A[259]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[14] += A[260]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[14] += A[261]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[14] += A[262]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[14] += A[263]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[14] += A[264]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[14] += A[265]*acadoWorkspace.rk_dim18_bPerm[13];

acadoWorkspace.rk_dim18_bPerm[15] += A[270]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[15] += A[271]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[15] += A[272]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[15] += A[273]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[15] += A[274]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[15] += A[275]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[15] += A[276]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[15] += A[277]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[15] += A[278]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[15] += A[279]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[15] += A[280]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[15] += A[281]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[15] += A[282]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[15] += A[283]*acadoWorkspace.rk_dim18_bPerm[13];
acadoWorkspace.rk_dim18_bPerm[15] += A[284]*acadoWorkspace.rk_dim18_bPerm[14];

acadoWorkspace.rk_dim18_bPerm[16] += A[288]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[16] += A[289]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[16] += A[290]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[16] += A[291]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[16] += A[292]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[16] += A[293]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[16] += A[294]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[16] += A[295]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[16] += A[296]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[16] += A[297]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[16] += A[298]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[16] += A[299]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[16] += A[300]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[16] += A[301]*acadoWorkspace.rk_dim18_bPerm[13];
acadoWorkspace.rk_dim18_bPerm[16] += A[302]*acadoWorkspace.rk_dim18_bPerm[14];
acadoWorkspace.rk_dim18_bPerm[16] += A[303]*acadoWorkspace.rk_dim18_bPerm[15];

acadoWorkspace.rk_dim18_bPerm[17] += A[306]*acadoWorkspace.rk_dim18_bPerm[0];
acadoWorkspace.rk_dim18_bPerm[17] += A[307]*acadoWorkspace.rk_dim18_bPerm[1];
acadoWorkspace.rk_dim18_bPerm[17] += A[308]*acadoWorkspace.rk_dim18_bPerm[2];
acadoWorkspace.rk_dim18_bPerm[17] += A[309]*acadoWorkspace.rk_dim18_bPerm[3];
acadoWorkspace.rk_dim18_bPerm[17] += A[310]*acadoWorkspace.rk_dim18_bPerm[4];
acadoWorkspace.rk_dim18_bPerm[17] += A[311]*acadoWorkspace.rk_dim18_bPerm[5];
acadoWorkspace.rk_dim18_bPerm[17] += A[312]*acadoWorkspace.rk_dim18_bPerm[6];
acadoWorkspace.rk_dim18_bPerm[17] += A[313]*acadoWorkspace.rk_dim18_bPerm[7];
acadoWorkspace.rk_dim18_bPerm[17] += A[314]*acadoWorkspace.rk_dim18_bPerm[8];
acadoWorkspace.rk_dim18_bPerm[17] += A[315]*acadoWorkspace.rk_dim18_bPerm[9];
acadoWorkspace.rk_dim18_bPerm[17] += A[316]*acadoWorkspace.rk_dim18_bPerm[10];
acadoWorkspace.rk_dim18_bPerm[17] += A[317]*acadoWorkspace.rk_dim18_bPerm[11];
acadoWorkspace.rk_dim18_bPerm[17] += A[318]*acadoWorkspace.rk_dim18_bPerm[12];
acadoWorkspace.rk_dim18_bPerm[17] += A[319]*acadoWorkspace.rk_dim18_bPerm[13];
acadoWorkspace.rk_dim18_bPerm[17] += A[320]*acadoWorkspace.rk_dim18_bPerm[14];
acadoWorkspace.rk_dim18_bPerm[17] += A[321]*acadoWorkspace.rk_dim18_bPerm[15];
acadoWorkspace.rk_dim18_bPerm[17] += A[322]*acadoWorkspace.rk_dim18_bPerm[16];


acado_solve_dim18_triangular( A, acadoWorkspace.rk_dim18_bPerm );
b[0] = acadoWorkspace.rk_dim18_bPerm[0];
b[1] = acadoWorkspace.rk_dim18_bPerm[1];
b[2] = acadoWorkspace.rk_dim18_bPerm[2];
b[3] = acadoWorkspace.rk_dim18_bPerm[3];
b[4] = acadoWorkspace.rk_dim18_bPerm[4];
b[5] = acadoWorkspace.rk_dim18_bPerm[5];
b[6] = acadoWorkspace.rk_dim18_bPerm[6];
b[7] = acadoWorkspace.rk_dim18_bPerm[7];
b[8] = acadoWorkspace.rk_dim18_bPerm[8];
b[9] = acadoWorkspace.rk_dim18_bPerm[9];
b[10] = acadoWorkspace.rk_dim18_bPerm[10];
b[11] = acadoWorkspace.rk_dim18_bPerm[11];
b[12] = acadoWorkspace.rk_dim18_bPerm[12];
b[13] = acadoWorkspace.rk_dim18_bPerm[13];
b[14] = acadoWorkspace.rk_dim18_bPerm[14];
b[15] = acadoWorkspace.rk_dim18_bPerm[15];
b[16] = acadoWorkspace.rk_dim18_bPerm[16];
b[17] = acadoWorkspace.rk_dim18_bPerm[17];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 2.5000000000000001e-02, 5.3867513459481292e-02, 
-3.8675134594812867e-03, 2.5000000000000001e-02 };


/* Fixed step size:0.1 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[9] = rk_eta[117];
acadoWorkspace.rk_xxx[10] = rk_eta[118];
acadoWorkspace.rk_xxx[11] = rk_eta[119];
acadoWorkspace.rk_xxx[12] = rk_eta[120];
acadoWorkspace.rk_xxx[13] = rk_eta[121];
acadoWorkspace.rk_xxx[14] = rk_eta[122];
acadoWorkspace.rk_xxx[15] = rk_eta[123];
acadoWorkspace.rk_xxx[16] = rk_eta[124];
acadoWorkspace.rk_xxx[17] = rk_eta[125];
acadoWorkspace.rk_xxx[18] = rk_eta[126];
acadoWorkspace.rk_xxx[19] = rk_eta[127];
acadoWorkspace.rk_xxx[20] = rk_eta[128];
acadoWorkspace.rk_xxx[21] = rk_eta[129];
acadoWorkspace.rk_xxx[22] = rk_eta[130];
acadoWorkspace.rk_xxx[23] = rk_eta[131];
acadoWorkspace.rk_xxx[24] = rk_eta[132];
acadoWorkspace.rk_xxx[25] = rk_eta[133];
acadoWorkspace.rk_xxx[26] = rk_eta[134];
acadoWorkspace.rk_xxx[27] = rk_eta[135];
acadoWorkspace.rk_xxx[28] = rk_eta[136];
acadoWorkspace.rk_xxx[29] = rk_eta[137];
acadoWorkspace.rk_xxx[30] = rk_eta[138];
acadoWorkspace.rk_xxx[31] = rk_eta[139];
acadoWorkspace.rk_xxx[32] = rk_eta[140];
acadoWorkspace.rk_xxx[33] = rk_eta[141];
acadoWorkspace.rk_xxx[34] = rk_eta[142];
acadoWorkspace.rk_xxx[35] = rk_eta[143];
acadoWorkspace.rk_xxx[36] = rk_eta[144];
acadoWorkspace.rk_xxx[37] = rk_eta[145];
acadoWorkspace.rk_xxx[38] = rk_eta[146];
acadoWorkspace.rk_xxx[39] = rk_eta[147];
acadoWorkspace.rk_xxx[40] = rk_eta[148];
acadoWorkspace.rk_xxx[41] = rk_eta[149];
acadoWorkspace.rk_xxx[42] = rk_eta[150];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 9; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
acadoWorkspace.rk_A[tmp_index1 * 18] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 7] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 18 + 9] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 10] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 11] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 12] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 13] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 14] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 15] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 16] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 17] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j + 9)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 9] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 9 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 9 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 9 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 9 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 9 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 9 + 6] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 9 + 7] = acadoWorkspace.rk_kkk[run1 + 14] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 9 + 8] = acadoWorkspace.rk_kkk[run1 + 16] - acadoWorkspace.rk_rhsTemp[8];
}
det = acado_solve_dim18_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 9];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 9 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 9 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 9 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 9 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 9 + 5];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 9 + 6];
acadoWorkspace.rk_kkk[j + 14] += acadoWorkspace.rk_b[j * 9 + 7];
acadoWorkspace.rk_kkk[j + 16] += acadoWorkspace.rk_b[j * 9 + 8];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 9; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 9] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 9 + 1] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 9 + 2] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 9 + 3] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 9 + 4] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 9 + 5] = acadoWorkspace.rk_kkk[run1 + 10] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 9 + 6] = acadoWorkspace.rk_kkk[run1 + 12] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 9 + 7] = acadoWorkspace.rk_kkk[run1 + 14] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 9 + 8] = acadoWorkspace.rk_kkk[run1 + 16] - acadoWorkspace.rk_rhsTemp[8];
}
acado_solve_dim18_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 9];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 9 + 1];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 9 + 2];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 9 + 3];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 9 + 4];
acadoWorkspace.rk_kkk[j + 10] += acadoWorkspace.rk_b[j * 9 + 5];
acadoWorkspace.rk_kkk[j + 12] += acadoWorkspace.rk_b[j * 9 + 6];
acadoWorkspace.rk_kkk[j + 14] += acadoWorkspace.rk_b[j * 9 + 7];
acadoWorkspace.rk_kkk[j + 16] += acadoWorkspace.rk_b[j * 9 + 8];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 9; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_kkk[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
acadoWorkspace.rk_A[tmp_index1 * 18] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 1] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 2] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 3] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 4] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 5] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 6] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 7] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 8] = + acado_Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 18 + 9] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 10] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 11] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 12] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 13] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 14] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 15] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 16] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 18 + 17] = + acado_Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 18) + (j + 9)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 9; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 9] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1)];
acadoWorkspace.rk_b[i * 9 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 12)];
acadoWorkspace.rk_b[i * 9 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 24)];
acadoWorkspace.rk_b[i * 9 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 36)];
acadoWorkspace.rk_b[i * 9 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 48)];
acadoWorkspace.rk_b[i * 9 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 60)];
acadoWorkspace.rk_b[i * 9 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 72)];
acadoWorkspace.rk_b[i * 9 + 7] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 84)];
acadoWorkspace.rk_b[i * 9 + 8] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 96)];
}
if( 0 == run1 ) {
det = acado_solve_dim18_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
}
 else {
acado_solve_dim18_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 9];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 9 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 9 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 9 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 9 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 9 + 5];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 9 + 6];
acadoWorkspace.rk_diffK[i + 14] = acadoWorkspace.rk_b[i * 9 + 7];
acadoWorkspace.rk_diffK[i + 16] = acadoWorkspace.rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 12) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 12) + (run1)] += + acadoWorkspace.rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index1 = (i * 9) + (j);
tmp_index2 = (run1) + (j * 12);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (tmp_index2 + 9)];
}
}
acado_solve_dim18_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim18_perm );
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 9];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 9 + 1];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 9 + 2];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 9 + 3];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 9 + 4];
acadoWorkspace.rk_diffK[i + 10] = acadoWorkspace.rk_b[i * 9 + 5];
acadoWorkspace.rk_diffK[i + 12] = acadoWorkspace.rk_b[i * 9 + 6];
acadoWorkspace.rk_diffK[i + 14] = acadoWorkspace.rk_b[i * 9 + 7];
acadoWorkspace.rk_diffK[i + 16] = acadoWorkspace.rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 12) + (run1 + 9)] = + acadoWorkspace.rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[2]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[4]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[5]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[6]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[7]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[8]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[9]*(real_t)5.0000000000000003e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[10]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[11]*(real_t)5.0000000000000003e-02;
rk_eta[6] += + acadoWorkspace.rk_kkk[12]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[13]*(real_t)5.0000000000000003e-02;
rk_eta[7] += + acadoWorkspace.rk_kkk[14]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[15]*(real_t)5.0000000000000003e-02;
rk_eta[8] += + acadoWorkspace.rk_kkk[16]*(real_t)5.0000000000000003e-02 + acadoWorkspace.rk_kkk[17]*(real_t)5.0000000000000003e-02;
for (i = 0; i < 9; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index2 = (j) + (i * 9);
rk_eta[tmp_index2 + 9] = acadoWorkspace.rk_diffsNew2[(i * 12) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 90] = acadoWorkspace.rk_diffsNew2[(i * 12) + (j + 9)];
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 9; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



