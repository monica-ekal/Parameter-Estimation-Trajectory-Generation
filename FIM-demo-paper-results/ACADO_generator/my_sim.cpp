/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
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


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState r1;
    DifferentialState r2;
    DifferentialState r3;
    DifferentialState v1;
    DifferentialState v2;
    DifferentialState v3;
    DifferentialState q1;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState q4;
    DifferentialState w1;
    DifferentialState w2;
    DifferentialState w3;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    Control u5;
    Control u6;
    SIMexport ExportModule1( 1, 1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_RIIA5 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(r1) == v1;
    acadodata_f1 << dot(r2) == v2;
    acadodata_f1 << dot(r3) == v3;
    acadodata_f1 << dot(v1) == 1/9.69999999999999928946e+00*u1;
    acadodata_f1 << dot(v2) == 1/9.69999999999999928946e+00*u2;
    acadodata_f1 << dot(v3) == 1/9.69999999999999928946e+00*u3;
    acadodata_f1 << dot(q1) == ((-q3)*5.00000000000000000000e-01*w2+5.00000000000000000000e-01*q2*w3+5.00000000000000000000e-01*q4*w1);
    acadodata_f1 << dot(q2) == ((-q1)*5.00000000000000000000e-01*w3+5.00000000000000000000e-01*q3*w1+5.00000000000000000000e-01*q4*w2);
    acadodata_f1 << dot(q3) == ((-q2)*5.00000000000000000000e-01*w1+5.00000000000000000000e-01*q1*w2+5.00000000000000000000e-01*q4*w3);
    acadodata_f1 << dot(q4) == ((-q1)*5.00000000000000000000e-01*w1+(-q2)*5.00000000000000000000e-01*w2+(-q3)*5.00000000000000000000e-01*w3);
    acadodata_f1 << dot(w1) == ((-1.42857142857142849213e-01)*(1.00000000000000000000e+01*w3*w2-7.00000000000000000000e+00*w2*w3)+1.42857142857142849213e-01*u4);
    acadodata_f1 << dot(w2) == ((-1.00000000000000000000e+01*w3*w1+7.00000000000000000000e+00*w1*w3)*(-1.42857142857142849213e-01)+1.42857142857142849213e-01*u5);
    acadodata_f1 << dot(w3) == ((-1.00000000000000005551e-01)*(-7.00000000000000000000e+00*w1*w2+7.00000000000000000000e+00*w2*w1)+1.00000000000000005551e-01*u6);

    ExportModule1.setModel( acadodata_f1 );

    uint export_flag = 0;
    ExportModule1.setTimingSteps( 0 );
    export_flag = ExportModule1.exportCode( "../SIM_export" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

