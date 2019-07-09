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
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4
#define ACADO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
/** Definition of the floating point data type. */
typedef double real_t;


/*
 * Common definitions
 */
/** Number of control/estimation intervals. */
#define ACADO_N 1
/** Number of online data values. */
#define ACADO_NOD 0
/** Number of control variables. */
#define ACADO_NU 6
/** Number of output functions. */
#define ACADO_NUMOUT 0
/** Number of differential variables. */
#define ACADO_NX 13
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0


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

} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim39_swap;

/** Column vector of size: 39 */
real_t rk_dim39_bPerm[ 39 ];

real_t rk_ttt;

/** Row vector of size: 19 */
real_t rk_xxx[ 19 ];

/** Matrix of size: 13 x 3 (row major format) */
real_t rk_kkk[ 39 ];

/** Matrix of size: 39 x 39 (row major format) */
real_t rk_A[ 1521 ];

/** Column vector of size: 39 */
real_t rk_b[ 39 ];

/** Row vector of size: 39 */
int rk_dim39_perm[ 39 ];

/** Column vector of size: 13 */
real_t rk_rhsTemp[ 13 ];

/** Matrix of size: 3 x 247 (row major format) */
real_t rk_diffsTemp2[ 741 ];

/** Matrix of size: 13 x 3 (row major format) */
real_t rk_diffK[ 39 ];

/** Matrix of size: 13 x 19 (row major format) */
real_t rk_diffsPrev2[ 247 ];

/** Matrix of size: 13 x 19 (row major format) */
real_t rk_diffsNew2[ 247 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array of size 19 to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_rhs(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_diffs(const real_t* in, real_t* out);


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
