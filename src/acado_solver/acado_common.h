/*
 *    This file was auto-generated by ACADO Code Generation Tool.
 *    
 *    ACADO Code Generation tool is a sub-package of ACADO toolkit --
 *    A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *    
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *    
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *    
 */


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES 0
/** FORCES QP solver indicator.*/
#define ACADO_FORCES  1
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES 2
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC 3
/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES

#include "acado_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 40
/** Number of online data values. */
#define ACADO_NOD 31
/** Number of control variables. */
#define ACADO_NU 3
/** Number of differential variables. */
#define ACADO_NX 13
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 16
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 10
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 120
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 1
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 2
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 2


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Matrix of size: 41 x 13 (row major format)
 * 
 *  Matrix containing 41 differential variable vectors.
 */
real_t x[ 533 ];

/** Matrix of size: 40 x 3 (row major format)
 * 
 *  Matrix containing 40 control variable vectors.
 */
real_t u[ 120 ];

/** Matrix of size: 41 x 31 (row major format)
 * 
 *  Matrix containing 41 online data vectors.
 */
real_t od[ 1271 ];

/** Column vector of size: 640
 * 
 *  Matrix containing 40 reference/measurement vectors of size 16 for first 40 nodes.
 */
real_t y[ 640 ];

/** Column vector of size: 10
 * 
 *  Reference/measurement vector for the 41. node.
 */
real_t yN[ 10 ];

/** Matrix of size: 640 x 16 (row major format) */
real_t W[ 10240 ];

/** Matrix of size: 10 x 10 (row major format) */
real_t WN[ 100 ];

/** Column vector of size: 13
 * 
 *  Current state feedback vector.
 */
real_t x0[ 13 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim26_swap;

/** Column vector of size: 26 */
real_t rk_dim26_bPerm[ 26 ];

real_t rk_ttt;

/** Row vector of size: 47 */
real_t rk_xxx[ 47 ];

/** Matrix of size: 13 x 2 (row major format) */
real_t rk_kkk[ 26 ];

/** Matrix of size: 26 x 26 (row major format) */
real_t rk_A[ 676 ];

/** Column vector of size: 26 */
real_t rk_b[ 26 ];

/** Row vector of size: 26 */
int rk_dim26_perm[ 26 ];

/** Column vector of size: 13 */
real_t rk_rhsTemp[ 13 ];

/** Matrix of size: 2 x 208 (row major format) */
real_t rk_diffsTemp2[ 416 ];

/** Matrix of size: 13 x 2 (row major format) */
real_t rk_diffK[ 26 ];

/** Matrix of size: 13 x 16 (row major format) */
real_t rk_diffsNew2[ 208 ];

/** Row vector of size: 255 */
real_t state[ 255 ];

/** Column vector of size: 520 */
real_t d[ 520 ];

/** Column vector of size: 640 */
real_t Dy[ 640 ];

/** Column vector of size: 10 */
real_t DyN[ 10 ];

/** Matrix of size: 520 x 13 (row major format) */
real_t evGx[ 6760 ];

/** Matrix of size: 520 x 3 (row major format) */
real_t evGu[ 1560 ];

/** Row vector of size: 47 */
real_t objValueIn[ 47 ];

/** Row vector of size: 272 */
real_t objValueOut[ 272 ];

/** Matrix of size: 520 x 13 (row major format) */
real_t Q1[ 6760 ];

/** Matrix of size: 520 x 16 (row major format) */
real_t Q2[ 8320 ];

/** Matrix of size: 120 x 3 (row major format) */
real_t R1[ 360 ];

/** Matrix of size: 120 x 16 (row major format) */
real_t R2[ 1920 ];

/** Matrix of size: 520 x 3 (row major format) */
real_t S1[ 1560 ];

/** Matrix of size: 13 x 13 (row major format) */
real_t QN1[ 169 ];

/** Matrix of size: 13 x 10 (row major format) */
real_t QN2[ 130 ];

/** Column vector of size: 13 */
real_t Dx0[ 13 ];

/** Matrix of size: 13 x 13 (row major format) */
real_t T[ 169 ];

/** Matrix of size: 10660 x 3 (row major format) */
real_t E[ 31980 ];

/** Matrix of size: 10660 x 3 (row major format) */
real_t QE[ 31980 ];

/** Column vector of size: 520 */
real_t Qd[ 520 ];

/** Column vector of size: 533 */
real_t QDy[ 533 ];

/** Matrix of size: 120 x 13 (row major format) */
real_t H10[ 1560 ];

/** Matrix of size: 120 x 120 (row major format) */
real_t H[ 14400 ];

/** Column vector of size: 120 */
real_t g[ 120 ];

/** Column vector of size: 120 */
real_t lb[ 120 ];

/** Column vector of size: 120 */
real_t ub[ 120 ];

/** Column vector of size: 120 */
real_t x[ 120 ];

/** Column vector of size: 120 */
real_t y[ 120 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array of size 47 to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int integrate( real_t* const rk_eta, int resetIntegrator );

/** An external function for evaluation of symbolic expressions. */
void rhs(const real_t* in, real_t* out);

/** An external function for evaluation of symbolic expressions. */
void rhs_jac(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 41 with xEnd. 2. Initialize node 41 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t getObjective(  );

/** An external function for evaluation of symbolic expressions. */
void evaluateLSQ(const real_t* in, real_t* out);

/** An external function for evaluation of symbolic expressions. */
void evaluateLSQEndTerm(const real_t* in, real_t* out);


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
