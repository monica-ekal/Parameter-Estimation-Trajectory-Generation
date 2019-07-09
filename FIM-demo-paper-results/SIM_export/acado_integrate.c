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


/*
input
{
	x
	u
	od
	seed:  only in case of adjoint sensitivity propagation
	grid1: only in case of extra output functions with an online grid
	grid2: ...
	grid3: ...
	grid4: ...
	reset: optional
		0: reset nothing and reuse factorization
		1: keep memory but recompute the factorization; default behaviour (!!)
		2: reset all memory + recompute the factorization
}

output: states
{
	value
	sensX: only in case of first order sensitivity propagation
	sensU: ...
	sens2: only in case of second order sensitivity propagation
}
output: out1, out2, out3, out4
{
	value
	sensX: only in case of first order sensitivity propagation
	sensU: ...
}
output: info
{
	errorCode
	clockTime
	debugMat: only in debugging mode
}
*/

/** MEX interface for the ACADO integrator
 *
 *  \author Rien Quirynen, rien.quirynen@esat.kuleuven.be
 *
 *  Credits: Milan Vukov
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "mex.h"
#include "acado_common.h"
#include "acado_auxiliary_sim_functions.h"


#define ONLINE_GRID 0
#define DEBUG_MODE 0
#define SENS_PROP 1
#define CALLS_TIMING 1
#define NUM_STAGES 3

/* GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM: */
/* --------------------------------------------------- */
   ACADOvariables acadoVariables;
   ACADOworkspace acadoWorkspace;
   

/** A bit more advanced printing function. */
void mexErrMsgTxtAdv(	char* string,
						...
						)
{
	static char buffer[ 128 ];
	
	va_list printArgs;
	va_start(printArgs, string);
	
	vsprintf(buffer, string, printArgs);
	va_end( printArgs );

	mexErrMsgTxt( buffer );
}

/** A simple helper function. */
void printMatrix(	const char* name,
					real_t* mat,
					unsigned nRows,
					unsigned nCols
					)
{
    unsigned r, c;
    mexPrintf("%s: \n", name);
    for (r = 0; r < nRows; ++r)
    {
        for(c = 0; c < nCols; ++c)
            mexPrintf("\t%f", mat[r * nCols + c]);
        mexPrintf("\n");
    }
}

/** A function for copying data from MATLAB to C array. */
int getArray(	const unsigned mandatory,
				const mxArray* source,
				const int index,
				const char* name,
				real_t* destination,
				const unsigned nRows,
				const unsigned nCols
				)
{
	mxArray* mxPtr = mxGetField(source, index, name);
	unsigned i, j;
	double* dPtr;
	
	if (mxPtr == NULL)
	{
		if ( !mandatory )
			return -1;
		else
			mexErrMsgTxtAdv("Field %s not found.", name);
	}

    if ( !mxIsDouble( mxPtr ) )
		mexErrMsgTxtAdv("Field %s must be an array of doubles.", name);

    if (mxGetM( mxPtr ) != nRows || mxGetN( mxPtr ) != nCols )
		mexErrMsgTxtAdv("Field %s must be of size: %d x %d.", name, nRows, nCols);

	dPtr = mxGetPr( mxPtr );
	
	if (destination == NULL)
		destination = (real_t*)mxCalloc(nRows * nCols, sizeof( real_t ));

	if (nRows == 1 && nCols == 1)
		*destination = *dPtr;
	else
		for (i = 0; i < nRows; ++i)
			for (j = 0; j < nCols; ++j)
				destination[i * nCols + j] = (real_t)dPtr[j * nRows + i];
			
	return 0;
}

void setArray( 	mxArray* destination,
				const int index,
				const char* name,
				real_t* source,
				const unsigned nRows,
				const unsigned nCols
				)
{
	mxArray* mxPtr = mxCreateDoubleMatrix(nRows, nCols, mxREAL);
	double* dPtr = mxGetPr( mxPtr );
	unsigned i, j;
	
	if (nRows == 1 && nCols == 1)
		*dPtr = *source;
	else
		for (i = 0; i < nRows; ++i)
			for(j = 0; j < nCols; ++j)
				dPtr[j * nRows + i] = (double)source[i * nCols + j];

	mxSetField(destination, index, name, mxPtr);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    int error, reset, c, i, j, k;
	double tmp[ 1 ];
	const mxArray* src = prhs[ 0 ];
    real_t *xnext, *sensX, *sensU, *sens2, *grid, *mat;
    real_t x[ACADO_NX+ACADO_NXA];
    #if ACADO_NU > 0
    real_t u[ACADO_NU];
    #endif
    #if ACADO_NOD > 0
    real_t od[ACADO_NOD];
    #endif
	#if SENS_PROP == 2 || SENS_PROP == 3 || SENS_PROP == 4
		real_t seed[ACADO_NX];
	#endif
    #if SENS_PROP == 1
		real_t next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU+ACADO_NOD];
    #elif SENS_PROP == 2
		real_t next[ACADO_NX+ACADO_NXA+ACADO_NX+ACADO_NU+ACADO_NU+ACADO_NOD];
    #elif SENS_PROP == 3
		real_t next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+ACADO_NX*(ACADO_NX+ACADO_NU)+ACADO_NU*ACADO_NU+ACADO_NU+ACADO_NOD];
    #elif SENS_PROP == 4
		int numX = ACADO_NX*(ACADO_NX+1)/2.0;
		int numU = ACADO_NU*(ACADO_NU+1)/2.0;
		real_t next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+numX+ACADO_NX*ACADO_NU+numU+ACADO_NU+ACADO_NOD];
    #else
		real_t next[ACADO_NX+ACADO_NXA+ACADO_NU+ACADO_NOD];
    #endif
    const char *fieldnames[4], *infonames[3];
    acado_timer tmr;
	real_t start, end, time;
	real_t *result_timing;
    
    #if ACADO_NUMOUT > 0
        real_t* out1;
        #if SENS_PROP == 1
			out1 = (real_t*) malloc(ACADO_NMEAS[0]*ACADO_NOUT[0]*(1+ACADO_NX+ACADO_NU)*sizeof(*out1));
        #else
			out1 = (real_t*) malloc(ACADO_NMEAS[0]*ACADO_NOUT[0]*sizeof(*out1));
        #endif
        
        #if ACADO_NUMOUT > 1
            real_t* out2;
            #if SENS_PROP == 1
				out2 = (real_t*) malloc(ACADO_NMEAS[1]*ACADO_NOUT[1]*(1+ACADO_NX+ACADO_NU)*sizeof(*out2));
			#else
				out2 = (real_t*) malloc(ACADO_NMEAS[1]*ACADO_NOUT[1]*sizeof(*out2));
			#endif
            
            #if ACADO_NUMOUT > 2
                real_t* out3;
                #if SENS_PROP == 1
					out3 = (real_t*) malloc(ACADO_NMEAS[2]*ACADO_NOUT[2]*(1+ACADO_NX+ACADO_NU)*sizeof(*out3));
				#else
					out3 = (real_t*) malloc(ACADO_NMEAS[2]*ACADO_NOUT[2]*sizeof(*out3));
				#endif
                
                #if ACADO_NUMOUT > 3
                    real_t* out4;
                    #if SENS_PROP == 1
						out4 = (real_t*) malloc(ACADO_NMEAS[3]*ACADO_NOUT[3]*(1+ACADO_NX+ACADO_NU)*sizeof(*out4));
					#else
						out4 = (real_t*) malloc(ACADO_NMEAS[3]*ACADO_NOUT[3]*sizeof(*out4));
					#endif
                #endif
            #endif
        #endif
    #endif        
    
    fieldnames[0] = "value";  
    fieldnames[1] = "sensX";  
    fieldnames[2] = "sensU"; 
    fieldnames[3] = "sens2"; 
                        
    infonames[0] = "errorCode";  
    infonames[1] = "clockTime";  
    infonames[2] = "debugMat"; 
    
	if (nrhs != 1) {
		mexErrMsgTxt("This function requires exactly one input: a structure with parameters.");
	}
	
	/* Copy MATLAB arrays to C arrays. */
	getArray(1, src, 0, "x", x, ACADO_NX+ACADO_NXA, 1);
	#if ACADO_NU > 0
		getArray(1, src, 0, "u", u, ACADO_NU, 1);
	#endif
	#if ACADO_NOD > 0
		getArray(1, src, 0, "od", od, ACADO_NOD, 1);
	#endif
	#if SENS_PROP == 2 || SENS_PROP == 3 || SENS_PROP == 4
		getArray(1, src, 0, "seed", seed, ACADO_NX, 1);
	#endif
		
    
    #if ONLINE_GRID
		#if ACADO_NUMOUT > 0
			getArray(1, src, 0, "grid1", acadoVariables.gridOutput0, 1, ACADO_NMEAS[0]);
        
		#if ACADO_NUMOUT > 1
			getArray(1, src, 0, "grid2", acadoVariables.gridOutput1, 1, ACADO_NMEAS[1]);
            
		#if ACADO_NUMOUT > 2
			getArray(1, src, 0, "grid3", acadoVariables.gridOutput2, 1, ACADO_NMEAS[2]);
                
		#if ACADO_NUMOUT > 3
			getArray(1, src, 0, "grid4", acadoVariables.gridOutput3, 1, ACADO_NMEAS[3]);
		#endif
		#endif
		#endif
		#endif  
    #endif     
    
	/* Get the reset flag. */
	if (getArray(0, src, 0, "reset", tmp, 1, 1) == 0)
		reset = (unsigned)tmp[ 0 ];
	else
		reset = 1;
    
    if (reset > 1) {
		memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    }
    
    #if SENS_PROP == 2 || SENS_PROP == 3 || SENS_PROP == 4
		for( i = 0; i < ACADO_NX; ++i ) {
			next[ACADO_NX+ACADO_NXA+i] = seed[i];
		}
	#endif
		
    #if ACADO_NU > 0
	for( i = 0; i < ACADO_NU; ++i ) {
		#if SENS_PROP == 1
			next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+i] = u[i];
		#elif SENS_PROP == 2
			next[ACADO_NX+ACADO_NXA+ACADO_NX+ACADO_NU+i] = u[i];
		#elif SENS_PROP == 3
			next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+ACADO_NX*(ACADO_NX+ACADO_NU)+ACADO_NU*ACADO_NU+i] = u[i];
		#elif SENS_PROP == 4
			next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+numX+ACADO_NX*ACADO_NU+numU+i] = u[i];
		#else
			next[ACADO_NX+ACADO_NXA+i] = u[i];
		#endif
    }
	#endif
    #if ACADO_NOD > 0
    for( i = 0; i < ACADO_NOD; ++i ) {
		#if SENS_PROP == 1
			next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NU+i] = od[i];
		#elif SENS_PROP == 2
			next[ACADO_NX+ACADO_NXA+ACADO_NX+ACADO_NU+ACADO_NU+i] = od[i];
		#elif SENS_PROP == 3
			next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+ACADO_NX*(ACADO_NX+ACADO_NU)+ACADO_NU*ACADO_NU+ACADO_NU+i] = od[i];
		#elif SENS_PROP == 4
			next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+numX+ACADO_NX*ACADO_NU+numU+ACADO_NU+i] = od[i];
		#else
			next[ACADO_NX+ACADO_NXA+ACADO_NU+i] = od[i];
		#endif
    }
    #endif
    
    acado_tic(&tmr);
    for( c = 0; c < CALLS_TIMING; ++c ) {
	    for( i = 0; i < ACADO_NX+ACADO_NXA; ++i ) {
	        next[i] = x[i];
	    }
	    
	    #if ACADO_NUMOUT > 3
	        error = acado_integrate( next, out1, out2, out3, out4, reset );
	    #elif ACADO_NUMOUT > 2
	        error = acado_integrate( next, out1, out2, out3, reset );
	    #elif ACADO_NUMOUT > 1
	        error = acado_integrate( next, out1, out2, reset );
	    #elif ACADO_NUMOUT > 0
	        error = acado_integrate( next, out1, reset );
	    #else 
			error = acado_integrate( next, reset );
	    #endif
	    reset = 0;
    }
    time = acado_toc(&tmr)/CALLS_TIMING;
    
    #if SENS_PROP == 1
		plhs[0] = mxCreateStructMatrix(1, 1, 3, fieldnames);
		mxSetField(plhs[0],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,1,mxREAL));
		mxSetField(plhs[0],0,fieldnames[1],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,ACADO_NX,mxREAL));
		mxSetField(plhs[0],0,fieldnames[2],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,ACADO_NU,mxREAL));
    #elif SENS_PROP == 2
		plhs[0] = mxCreateStructMatrix(1, 1, 3, fieldnames);
		mxSetField(plhs[0],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,1,mxREAL));
		mxSetField(plhs[0],0,fieldnames[1],mxCreateDoubleMatrix(1,ACADO_NX,mxREAL));
		mxSetField(plhs[0],0,fieldnames[2],mxCreateDoubleMatrix(1,ACADO_NU,mxREAL));
	#else
		plhs[0] = mxCreateStructMatrix(1, 1, 1, fieldnames);
		mxSetField(plhs[0],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,1,mxREAL));
	#endif
    #if SENS_PROP == 3 || SENS_PROP == 4
		plhs[0] = mxCreateStructMatrix(1, 1, 4, fieldnames);
		mxSetField(plhs[0],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,1,mxREAL));
		mxSetField(plhs[0],0,fieldnames[1],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,ACADO_NX,mxREAL));
		mxSetField(plhs[0],0,fieldnames[2],mxCreateDoubleMatrix(ACADO_NX+ACADO_NXA,ACADO_NU,mxREAL));
		mxSetField(plhs[0],0,fieldnames[3],mxCreateDoubleMatrix(ACADO_NX+ACADO_NU,ACADO_NX+ACADO_NU,mxREAL));
    #endif
    
    xnext = mxGetPr(mxGetField(plhs[0],0,fieldnames[0]));
    #if SENS_PROP
		sensX = mxGetPr(mxGetField(plhs[0],0,fieldnames[1]));
		sensU = mxGetPr(mxGetField(plhs[0],0,fieldnames[2]));
    #endif
    #if SENS_PROP == 3 || SENS_PROP == 4
		sens2 = mxGetPr(mxGetField(plhs[0],0,fieldnames[3]));
    #endif
    
    for( i = 0; i < ACADO_NX+ACADO_NXA; ++i ) {
        xnext[i] = next[i];
    }
    
    #if SENS_PROP == 1
	    /* NOTE: Matlab is column-major, while C is row-major order !! */
	    for( i = 0; i < ACADO_NX+ACADO_NXA; ++i ) {
	        for( j = 0; j < ACADO_NX; ++j ) {
	            sensX[j*(ACADO_NX+ACADO_NXA)+i] = next[ACADO_NX+ACADO_NXA+i*ACADO_NX+j];
	        }
	    }
	    for( i = 0; i < ACADO_NX+ACADO_NXA; ++i ) {
	        for( j = 0; j < ACADO_NU; ++j ) {
	            sensU[j*(ACADO_NX+ACADO_NXA)+i] = next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX)+i*ACADO_NU+j];
	        }
	    }
    #elif SENS_PROP == 2
	    for( j = 0; j < ACADO_NX; ++j ) {
	        sensX[j] = next[ACADO_NX+ACADO_NXA+j];
	    }
	    for( j = 0; j < ACADO_NU; ++j ) {
	        sensU[j] = next[ACADO_NX+ACADO_NXA+ACADO_NX+j];
	    }
    #elif SENS_PROP == 4 || SENS_PROP == 3
	    /* NOTE: Matlab is column-major, while C is row-major order !! */
	    for( i = 0; i < ACADO_NX+ACADO_NXA; ++i ) {
	        for( j = 0; j < ACADO_NX; ++j ) {
	            sensX[j*(ACADO_NX+ACADO_NXA)+i] = next[ACADO_NX+ACADO_NX+ACADO_NXA+i*ACADO_NX+j];
	        }
	    }
	    for( i = 0; i < ACADO_NX+ACADO_NXA; ++i ) {
	        for( j = 0; j < ACADO_NU; ++j ) {
	            sensU[j*(ACADO_NX+ACADO_NXA)+i] = next[ACADO_NX+(ACADO_NX+ACADO_NXA)*(1+ACADO_NX)+i*ACADO_NU+j];
	        }
	    }
    #endif
    #if SENS_PROP == 3
		/* NOTE: Matlab is column-major, while C is row-major order !! */
		/* NOTE: The order here is [Sxx, Sux, Suu] !! */
	    for( i = 0; i < ACADO_NX; ++i ) {
	        for( j = 0; j < ACADO_NX; ++j ) {
	            sens2[j*(ACADO_NX+ACADO_NU)+i] = next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+i*ACADO_NX+j];
	        }
	    }
	    for( i = 0; i < ACADO_NU; ++i ) {
	        for( j = 0; j < ACADO_NX; ++j ) {
	            sens2[j*(ACADO_NX+ACADO_NU)+ACADO_NX+i] = next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX*(1+ACADO_NX)+i*ACADO_NX+j];
	            sens2[(ACADO_NX+i)*(ACADO_NX+ACADO_NU)+j] = sens2[j*(ACADO_NX+ACADO_NU)+ACADO_NX+i];
	        }
	    }
	    for( i = 0; i < ACADO_NU; ++i ) {
	        for( j = 0; j < ACADO_NU; ++j ) {
	            sens2[(ACADO_NX+j)*(ACADO_NX+ACADO_NU)+ACADO_NX+i] = next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX*(1+ACADO_NX+ACADO_NU)+i*ACADO_NU+j];
	        }
	    }
	#elif SENS_PROP == 4
		/* NOTE: Matlab is column-major, while C is row-major order !! */
		/* NOTE: The order here is the lower triangular of the full Hessian H !! */
	    for( i = 0; i < ACADO_NX+ACADO_NU; ++i ) {
	        for( j = 0; j <= i; ++j ) {
	            sens2[j*(ACADO_NX+ACADO_NU)+i] = next[(ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+ACADO_NX+i*(i+1)/2+j];
	            sens2[i*(ACADO_NX+ACADO_NU)+j] = sens2[j*(ACADO_NX+ACADO_NU)+i];
	        }
	    }
    #endif
    
    if (nlhs > 1+ACADO_NUMOUT) {
    #if DEBUG_MODE > 0
		plhs[1+ACADO_NUMOUT] = mxCreateStructMatrix(1, 1, 3, infonames);
		mxSetField(plhs[1+ACADO_NUMOUT],0,infonames[0],mxCreateDoubleScalar(error));
	    mxSetField(plhs[1+ACADO_NUMOUT],0,infonames[1],mxCreateDoubleScalar(time));
	    mxSetField(plhs[1+ACADO_NUMOUT],0,infonames[2],mxCreateDoubleMatrix(NUM_STAGES*(ACADO_NX+ACADO_NXA),NUM_STAGES*(ACADO_NX+ACADO_NXA),mxREAL));
	    
	    mat = mxGetPr(mxGetField(plhs[1+ACADO_NUMOUT],0,infonames[2]));
		for( i = 0; i < NUM_STAGES*(ACADO_NX+ACADO_NXA); ++i ) {
            for( j = 0; j < NUM_STAGES*(ACADO_NX+ACADO_NXA); ++j ) {
                mat[j*NUM_STAGES*(ACADO_NX+ACADO_NXA)+i] = acadoVariables.debug_mat[i*NUM_STAGES*(ACADO_NX+ACADO_NXA)+j];
            }
        }
	#else	
		plhs[1+ACADO_NUMOUT] = mxCreateStructMatrix(1, 1, 2, infonames);
		mxSetField(plhs[1+ACADO_NUMOUT],0,infonames[0],mxCreateDoubleScalar(error));
	    mxSetField(plhs[1+ACADO_NUMOUT],0,infonames[1],mxCreateDoubleScalar(time));
    #endif
    }
    
    #if ACADO_NUMOUT > 0
    if (nlhs > 1) {
		#if SENS_PROP == 1
	        plhs[1] = mxCreateStructMatrix(1, 1, 3, fieldnames);
	        mxSetField(plhs[1],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[0],ACADO_NMEAS[0],mxREAL));
	        mxSetField(plhs[1],0,fieldnames[1],mxCreateDoubleMatrix(ACADO_NOUT[0]*ACADO_NMEAS[0],ACADO_NX,mxREAL));
	        mxSetField(plhs[1],0,fieldnames[2],mxCreateDoubleMatrix(ACADO_NOUT[0]*ACADO_NMEAS[0],ACADO_NU,mxREAL));
        #else
			plhs[1] = mxCreateStructMatrix(1, 1, 1, fieldnames);
	        mxSetField(plhs[1],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[0],ACADO_NMEAS[0],mxREAL));
        #endif
        
        xnext = mxGetPr(mxGetField(plhs[1],0,fieldnames[0]));
        #if SENS_PROP == 1
	        sensX = mxGetPr(mxGetField(plhs[1],0,fieldnames[1]));
	        sensU = mxGetPr(mxGetField(plhs[1],0,fieldnames[2]));
        #endif
        
        for( k = 0; k < ACADO_NMEAS[0]; ++k ) {
            for( i = 0; i < ACADO_NOUT[0]; ++i ) {
				#if SENS_PROP == 1
					xnext[k*ACADO_NOUT[0]+i] = out1[k*ACADO_NOUT[0]*(1+ACADO_NX+ACADO_NU)+i];
                #else
					xnext[k*ACADO_NOUT[0]+i] = out1[k*ACADO_NOUT[0]+i];
                #endif
            }
        }
        
        #if SENS_PROP == 1
	        /* NOTE: Matlab is column-major, while C is row-major order !! */
	        for( k = 0; k < ACADO_NMEAS[0]; ++k ) {
	            for( i = 0; i < ACADO_NOUT[0]; ++i ) {
	                for( j = 0; j < ACADO_NX; ++j ) {
	                    sensX[j*ACADO_NOUT[0]*ACADO_NMEAS[0]+k*ACADO_NOUT[0]+i] = out1[k*ACADO_NOUT[0]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[0]+i*ACADO_NX+j];
	                }
	            }
	        }
	        for( k = 0; k < ACADO_NMEAS[0]; ++k ) {
	            for( i = 0; i < ACADO_NOUT[0]; ++i ) {
	                for( j = 0; j < ACADO_NU; ++j ) {
	                    sensU[j*ACADO_NOUT[0]*ACADO_NMEAS[0]+k*ACADO_NOUT[0]+i] = out1[k*ACADO_NOUT[0]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[0]*(1+ACADO_NX)+i*ACADO_NU+j];
	                }
	            }
	        }
        #endif
        free(out1);
        #if ACADO_NUMOUT > 1
        if (nlhs > 2) {
			#if SENS_PROP == 1
	            plhs[2] = mxCreateStructMatrix(1, 1, 3, fieldnames);
	            mxSetField(plhs[2],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[1],ACADO_NMEAS[1],mxREAL));
	            mxSetField(plhs[2],0,fieldnames[1],mxCreateDoubleMatrix(ACADO_NOUT[1]*ACADO_NMEAS[1],ACADO_NX,mxREAL));
	            mxSetField(plhs[2],0,fieldnames[2],mxCreateDoubleMatrix(ACADO_NOUT[1]*ACADO_NMEAS[1],ACADO_NU,mxREAL));
            #else
				plhs[2] = mxCreateStructMatrix(1, 1, 1, fieldnames);
	            mxSetField(plhs[2],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[1],ACADO_NMEAS[1],mxREAL));
            #endif

            xnext = mxGetPr(mxGetField(plhs[2],0,fieldnames[0]));
            #if SENS_PROP == 1
	            sensX = mxGetPr(mxGetField(plhs[2],0,fieldnames[1]));
	            sensU = mxGetPr(mxGetField(plhs[2],0,fieldnames[2]));
            #endif

            for( k = 0; k < ACADO_NMEAS[1]; ++k ) {
                for( i = 0; i < ACADO_NOUT[1]; ++i ) {
					#if SENS_PROP == 1
						xnext[k*ACADO_NOUT[1]+i] = out2[k*ACADO_NOUT[1]*(1+ACADO_NX+ACADO_NU)+i];
                    #else
						xnext[k*ACADO_NOUT[1]+i] = out2[k*ACADO_NOUT[1]+i];
                    #endif
                }
            }

			#if SENS_PROP == 1
	            /* NOTE: Matlab is column-major, while C is row-major order !! */
	            for( k = 0; k < ACADO_NMEAS[1]; ++k ) {
	                for( i = 0; i < ACADO_NOUT[1]; ++i ) {
	                    for( j = 0; j < ACADO_NX; ++j ) {
	                        sensX[j*ACADO_NOUT[1]*ACADO_NMEAS[1]+k*ACADO_NOUT[1]+i] = out2[k*ACADO_NOUT[1]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[1]+i*ACADO_NX+j];
	                    }
	                }
	            }
	            for( k = 0; k < ACADO_NMEAS[1]; ++k ) {
	                for( i = 0; i < ACADO_NOUT[1]; ++i ) {
	                    for( j = 0; j < ACADO_NU; ++j ) {
	                        sensU[j*ACADO_NOUT[1]*ACADO_NMEAS[1]+k*ACADO_NOUT[1]+i] = out2[k*ACADO_NOUT[1]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[1]*(1+ACADO_NX)+i*ACADO_NU+j];
	                    }
	                }
	            }
            #endif
            free(out2);
            #if ACADO_NUMOUT > 2
            if (nlhs > 3) {
				#if SENS_PROP == 1
	                plhs[3] = mxCreateStructMatrix(1, 1, 3, fieldnames);
	                mxSetField(plhs[3],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[2],ACADO_NMEAS[2],mxREAL));
	                mxSetField(plhs[3],0,fieldnames[1],mxCreateDoubleMatrix(ACADO_NOUT[2]*ACADO_NMEAS[2],ACADO_NX,mxREAL));
	                mxSetField(plhs[3],0,fieldnames[2],mxCreateDoubleMatrix(ACADO_NOUT[2]*ACADO_NMEAS[2],ACADO_NU,mxREAL));
                #else
					plhs[3] = mxCreateStructMatrix(1, 1, 1, fieldnames);
	                mxSetField(plhs[3],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[2],ACADO_NMEAS[2],mxREAL));
                #endif

                xnext = mxGetPr(mxGetField(plhs[3],0,fieldnames[0]));
                #if SENS_PROP == 1
	                sensX = mxGetPr(mxGetField(plhs[3],0,fieldnames[1]));
	                sensU = mxGetPr(mxGetField(plhs[3],0,fieldnames[2]));
                #endif

                for( k = 0; k < ACADO_NMEAS[2]; ++k ) {
                    for( i = 0; i < ACADO_NOUT[2]; ++i ) {
						#if SENS_PROP == 1
							xnext[k*ACADO_NOUT[2]+i] = out3[k*ACADO_NOUT[2]*(1+ACADO_NX+ACADO_NU)+i];
                        #else
							xnext[k*ACADO_NOUT[2]+i] = out3[k*ACADO_NOUT[2]+i];
                        #endif
                    }
                }

				#if SENS_PROP == 1
	                /* NOTE: Matlab is column-major, while C is row-major order !! */
	                for( k = 0; k < ACADO_NMEAS[2]; ++k ) {
	                    for( i = 0; i < ACADO_NOUT[2]; ++i ) {
	                        for( j = 0; j < ACADO_NX; ++j ) {
	                            sensX[j*ACADO_NOUT[2]*ACADO_NMEAS[2]+k*ACADO_NOUT[2]+i] = out3[k*ACADO_NOUT[2]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[2]+i*ACADO_NX+j];
	                        }
	                    }
	                }
	                for( k = 0; k < ACADO_NMEAS[2]; ++k ) {
	                    for( i = 0; i < ACADO_NOUT[2]; ++i ) {
	                        for( j = 0; j < ACADO_NU; ++j ) {
	                            sensU[j*ACADO_NOUT[2]*ACADO_NMEAS[2]+k*ACADO_NOUT[2]+i] = out3[k*ACADO_NOUT[2]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[2]*(1+ACADO_NX)+i*ACADO_NU+j];
	                        }
	                    }
	                }
                #endif
                free(out3);
                #if ACADO_NUMOUT > 3
                if (nlhs > 4) {
					#if SENS_PROP == 1
	                    plhs[4] = mxCreateStructMatrix(1, 1, 3, fieldnames);
	                    mxSetField(plhs[4],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[3],ACADO_NMEAS[3],mxREAL));
	                    mxSetField(plhs[4],0,fieldnames[1],mxCreateDoubleMatrix(ACADO_NOUT[3]*ACADO_NMEAS[3],ACADO_NX,mxREAL));
	                    mxSetField(plhs[4],0,fieldnames[2],mxCreateDoubleMatrix(ACADO_NOUT[3]*ACADO_NMEAS[3],ACADO_NU,mxREAL));
                    #else
						plhs[4] = mxCreateStructMatrix(1, 1, 1, fieldnames);
	                    mxSetField(plhs[4],0,fieldnames[0],mxCreateDoubleMatrix(ACADO_NOUT[3],ACADO_NMEAS[3],mxREAL));
                    #endif

                    xnext = mxGetPr(mxGetField(plhs[4],0,fieldnames[0]));
                    #if SENS_PROP == 1
	                    sensX = mxGetPr(mxGetField(plhs[4],0,fieldnames[1]));
	                    sensU = mxGetPr(mxGetField(plhs[4],0,fieldnames[2]));
                    #endif

                    for( k = 0; k < ACADO_NMEAS[3]; ++k ) {
                        for( i = 0; i < ACADO_NOUT[3]; ++i ) {
							#if SENS_PROP == 1
								xnext[k*ACADO_NOUT[3]+i] = out4[k*ACADO_NOUT[3]*(1+ACADO_NX+ACADO_NU)+i];
							#else
								xnext[k*ACADO_NOUT[3]+i] = out4[k*ACADO_NOUT[3]+i];
							#endif
                        }
                    }

					#if SENS_PROP == 1
	                    /* NOTE: Matlab is column-major, while C is row-major order !! */
	                    for( k = 0; k < ACADO_NMEAS[3]; ++k ) {
	                        for( i = 0; i < ACADO_NOUT[3]; ++i ) {
	                            for( j = 0; j < ACADO_NX; ++j ) {
	                                sensX[j*ACADO_NOUT[3]*ACADO_NMEAS[3]+k*ACADO_NOUT[3]+i] = out4[k*ACADO_NOUT[3]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[3]+i*ACADO_NX+j];
	                            }
	                        }
	                    }
	                    for( k = 0; k < ACADO_NMEAS[3]; ++k ) {
	                        for( i = 0; i < ACADO_NOUT[3]; ++i ) {
	                            for( j = 0; j < ACADO_NU; ++j ) {
	                                sensU[j*ACADO_NOUT[3]*ACADO_NMEAS[3]+k*ACADO_NOUT[3]+i] = out4[k*ACADO_NOUT[3]*(1+ACADO_NX+ACADO_NU)+ACADO_NOUT[3]*(1+ACADO_NX)+i*ACADO_NU+j];
	                            }
	                        }
	                    }
                    #endif
                    free(out4);
                }
                #endif
            }
            #endif
        }
        #endif
    }
    #endif
}
