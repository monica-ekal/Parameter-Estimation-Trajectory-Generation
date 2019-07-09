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
    DifferentialState psi1;
    DifferentialState psi2;
    DifferentialState psi3;
    DifferentialState psi4;
    DifferentialState psi5;
    DifferentialState psi6;
    DifferentialState psi7;
    DifferentialState psi8;
    DifferentialState psi9;
    DifferentialState psi10;
    DifferentialState psi11;
    DifferentialState psi12;
    DifferentialState psi13;
    DifferentialState psi14;
    DifferentialState psi15;
    DifferentialState psi16;
    DifferentialState psi17;
    DifferentialState psi18;
    DifferentialState psi19;
    DifferentialState psi20;
    DifferentialState psi21;
    DifferentialState psi22;
    DifferentialState psi23;
    DifferentialState psi24;
    DifferentialState psi25;
    DifferentialState psi26;
    DifferentialState psi27;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    Control u5;
    Control u6;
    OnlineData Mass; 
    OnlineData I_xx; 
    OnlineData I_yy; 
    OnlineData I_zz; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "test_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "test_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << 1/(1.00000000000000000000e+00+1.00000000000000000000e+02*psi11*psi11+1.00000000000000000000e+02*psi12*psi12+1.00000000000000000000e+02*psi13*psi13+1.00000000000000000000e+02*psi18*psi18+1.00000000000000000000e+02*psi19*psi19+1.00000000000000000000e+02*psi20*psi20+1.00000000000000000000e+02*psi25*psi25+1.00000000000000000000e+02*psi26*psi26+1.00000000000000000000e+02*psi27*psi27+1.00000000000000000000e+02*psi4*psi4+1.00000000000000000000e+02*psi5*psi5+1.00000000000000000000e+02*psi6*psi6);
    acadodata_f1 << r1;
    acadodata_f1 << r2;
    acadodata_f1 << r3;
    acadodata_f1 << v1;
    acadodata_f1 << v2;
    acadodata_f1 << v3;
    acadodata_f1 << (-(-q1*q4+q2*q3)*2.00000000000000000000e+00+(q1*q4+q2*q3)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+1.00000000000000000000e+00));
    acadodata_f1 << ((q1*q3+q2*q4)*2.00000000000000000000e+00-(q1*q3-q2*q4)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+1.00000000000000000000e+00));
    acadodata_f1 << ((q1*q2+q3*q4)*2.00000000000000000000e+00-(q1*q2-q3*q4)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+1.00000000000000000000e+00));
    acadodata_f1 << w1;
    acadodata_f1 << w2;
    acadodata_f1 << w3;
    Function acadodata_f2;
    acadodata_f2 << r1;
    acadodata_f2 << r2;
    acadodata_f2 << r3;
    acadodata_f2 << (-(-q1*q4+q2*q3)*2.00000000000000000000e+00+(q1*q4+q2*q3)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+1.00000000000000000000e+00));
    acadodata_f2 << ((q1*q3+q2*q4)*2.00000000000000000000e+00-(q1*q3-q2*q4)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+1.00000000000000000000e+00));
    acadodata_f2 << ((q1*q2+q3*q4)*2.00000000000000000000e+00-(q1*q2-q3*q4)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))+1.00000000000000000000e+00));
    acadodata_f2 << v1;
    acadodata_f2 << v2;
    acadodata_f2 << v3;
    acadodata_f2 << w1;
    acadodata_f2 << w2;
    acadodata_f2 << w3;
    OCP ocp1(0, 40, 40);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo((-5.00000000000000027756e-02) <= u1 <= 5.00000000000000027756e-02);
    ocp1.subjectTo((-5.00000000000000027756e-02) <= u2 <= 5.00000000000000027756e-02);
    ocp1.subjectTo((-5.00000000000000027756e-02) <= u3 <= 5.00000000000000027756e-02);
    ocp1.subjectTo((-5.00000000000000027756e-02) <= u4 <= 5.00000000000000027756e-02);
    ocp1.subjectTo((-5.00000000000000027756e-02) <= u5 <= 5.00000000000000027756e-02);
    ocp1.subjectTo((-5.00000000000000027756e-02) <= u6 <= 5.00000000000000027756e-02);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(r1) == v1;
    acadodata_f3 << dot(r2) == v2;
    acadodata_f3 << dot(r3) == v3;
    acadodata_f3 << dot(v1) == 1/Mass*u1;
    acadodata_f3 << dot(v2) == 1/Mass*u2;
    acadodata_f3 << dot(v3) == 1/Mass*u3;
    acadodata_f3 << dot(q1) == ((-q3)*5.00000000000000000000e-01*w2+5.00000000000000000000e-01*q2*w3+5.00000000000000000000e-01*q4*w1);
    acadodata_f3 << dot(q2) == ((-q1)*5.00000000000000000000e-01*w3+5.00000000000000000000e-01*q3*w1+5.00000000000000000000e-01*q4*w2);
    acadodata_f3 << dot(q3) == ((-q2)*5.00000000000000000000e-01*w1+5.00000000000000000000e-01*q1*w2+5.00000000000000000000e-01*q4*w3);
    acadodata_f3 << dot(q4) == ((-q1)*5.00000000000000000000e-01*w1+(-q2)*5.00000000000000000000e-01*w2+(-q3)*5.00000000000000000000e-01*w3);
    acadodata_f3 << dot(w1) == ((-1/I_xx)*(-I_yy*w2*w3+I_zz*w3*w2)+1/I_xx*u4);
    acadodata_f3 << dot(w2) == ((-1/I_yy)*(I_xx*w1*w3-I_zz*w3*w1)+1/I_yy*u5);
    acadodata_f3 << dot(w3) == ((-1/I_zz)*(-I_xx*w1*w2+I_yy*w2*w1)+1/I_zz*u6);
    acadodata_f3 << dot(psi1) == psi4;
    acadodata_f3 << dot(psi2) == psi5;
    acadodata_f3 << dot(psi3) == psi6;
    acadodata_f3 << dot(psi4) == (-1.00000000000000000000e+00)*pow(Mass,(-2.00000000000000000000e+00))*u1;
    acadodata_f3 << dot(psi5) == (-1.00000000000000000000e+00)*pow(Mass,(-2.00000000000000000000e+00))*u2;
    acadodata_f3 << dot(psi6) == (-1.00000000000000000000e+00)*pow(Mass,(-2.00000000000000000000e+00))*u3;
    acadodata_f3 << dot(psi7) == ((-q3)*5.00000000000000000000e-01*psi12+(-w2)*5.00000000000000000000e-01*psi9+5.00000000000000000000e-01*psi10*w1+5.00000000000000000000e-01*psi11*q4+5.00000000000000000000e-01*psi13*q2+5.00000000000000000000e-01*psi8*w3);
    acadodata_f3 << dot(psi8) == ((-q1)*5.00000000000000000000e-01*psi13+(-w3)*5.00000000000000000000e-01*psi7+5.00000000000000000000e-01*psi10*w2+5.00000000000000000000e-01*psi11*q3+5.00000000000000000000e-01*psi12*q4+5.00000000000000000000e-01*psi9*w1);
    acadodata_f3 << dot(psi9) == ((-q2)*5.00000000000000000000e-01*psi11+(-w1)*5.00000000000000000000e-01*psi8+5.00000000000000000000e-01*psi10*w3+5.00000000000000000000e-01*psi12*q1+5.00000000000000000000e-01*psi13*q4+5.00000000000000000000e-01*psi7*w2);
    acadodata_f3 << dot(psi10) == ((-q1)*5.00000000000000000000e-01*psi11+(-q2)*5.00000000000000000000e-01*psi12+(-q3)*5.00000000000000000000e-01*psi13+(-w1)*5.00000000000000000000e-01*psi7+(-w2)*5.00000000000000000000e-01*psi8+(-w3)*5.00000000000000000000e-01*psi9);
    acadodata_f3 << dot(psi11) == ((1/I_xx*I_yy*w2-1/I_xx*I_zz*w2)*psi13+(1/I_xx*I_yy*w3-1/I_xx*I_zz*w3)*psi12-pow(I_xx,(-2.00000000000000000000e+00))*I_yy*w2*w3+pow(I_xx,(-2.00000000000000000000e+00))*I_zz*w2*w3-pow(I_xx,(-2.00000000000000000000e+00))*u4);
    acadodata_f3 << dot(psi12) == ((-1.00000000000000000000e+00)/I_yy*w1*w3+(1/I_yy*I_zz*w1-I_xx/I_yy*w1)*psi13+(1/I_yy*I_zz*w3-I_xx/I_yy*w3)*psi11);
    acadodata_f3 << dot(psi13) == ((I_xx/I_zz*w1-I_yy/I_zz*w1)*psi12+(I_xx/I_zz*w2-I_yy/I_zz*w2)*psi11+1/I_zz*w1*w2);
    acadodata_f3 << dot(psi14) == ((-q3)*5.00000000000000000000e-01*psi19+(-w2)*5.00000000000000000000e-01*psi16+5.00000000000000000000e-01*psi15*w3+5.00000000000000000000e-01*psi17*w1+5.00000000000000000000e-01*psi18*q4+5.00000000000000000000e-01*psi20*q2);
    acadodata_f3 << dot(psi15) == ((-q1)*5.00000000000000000000e-01*psi20+(-w3)*5.00000000000000000000e-01*psi14+5.00000000000000000000e-01*psi16*w1+5.00000000000000000000e-01*psi17*w2+5.00000000000000000000e-01*psi18*q3+5.00000000000000000000e-01*psi19*q4);
    acadodata_f3 << dot(psi16) == ((-q2)*5.00000000000000000000e-01*psi18+(-w1)*5.00000000000000000000e-01*psi15+5.00000000000000000000e-01*psi14*w2+5.00000000000000000000e-01*psi17*w3+5.00000000000000000000e-01*psi19*q1+5.00000000000000000000e-01*psi20*q4);
    acadodata_f3 << dot(psi17) == ((-q1)*5.00000000000000000000e-01*psi18+(-q2)*5.00000000000000000000e-01*psi19+(-q3)*5.00000000000000000000e-01*psi20+(-w1)*5.00000000000000000000e-01*psi14+(-w2)*5.00000000000000000000e-01*psi15+(-w3)*5.00000000000000000000e-01*psi16);
    acadodata_f3 << dot(psi18) == ((1/I_xx*I_yy*w2-1/I_xx*I_zz*w2)*psi20+(1/I_xx*I_yy*w3-1/I_xx*I_zz*w3)*psi19+1/I_xx*w2*w3);
    acadodata_f3 << dot(psi19) == ((1/I_yy*I_zz*w1-I_xx/I_yy*w1)*psi20+(1/I_yy*I_zz*w3-I_xx/I_yy*w3)*psi18+I_xx*pow(I_yy,(-2.00000000000000000000e+00))*w1*w3-pow(I_yy,(-2.00000000000000000000e+00))*I_zz*w1*w3-pow(I_yy,(-2.00000000000000000000e+00))*u5);
    acadodata_f3 << dot(psi20) == ((-1.00000000000000000000e+00)/I_zz*w1*w2+(I_xx/I_zz*w1-I_yy/I_zz*w1)*psi19+(I_xx/I_zz*w2-I_yy/I_zz*w2)*psi18);
    acadodata_f3 << dot(psi21) == ((-q3)*5.00000000000000000000e-01*psi26+(-w2)*5.00000000000000000000e-01*psi23+5.00000000000000000000e-01*psi22*w3+5.00000000000000000000e-01*psi24*w1+5.00000000000000000000e-01*psi25*q4+5.00000000000000000000e-01*psi27*q2);
    acadodata_f3 << dot(psi22) == ((-q1)*5.00000000000000000000e-01*psi27+(-w3)*5.00000000000000000000e-01*psi21+5.00000000000000000000e-01*psi23*w1+5.00000000000000000000e-01*psi24*w2+5.00000000000000000000e-01*psi25*q3+5.00000000000000000000e-01*psi26*q4);
    acadodata_f3 << dot(psi23) == ((-q2)*5.00000000000000000000e-01*psi25+(-w1)*5.00000000000000000000e-01*psi22+5.00000000000000000000e-01*psi21*w2+5.00000000000000000000e-01*psi24*w3+5.00000000000000000000e-01*psi26*q1+5.00000000000000000000e-01*psi27*q4);
    acadodata_f3 << dot(psi24) == ((-q1)*5.00000000000000000000e-01*psi25+(-q2)*5.00000000000000000000e-01*psi26+(-q3)*5.00000000000000000000e-01*psi27+(-w1)*5.00000000000000000000e-01*psi21+(-w2)*5.00000000000000000000e-01*psi22+(-w3)*5.00000000000000000000e-01*psi23);
    acadodata_f3 << dot(psi25) == ((-1.00000000000000000000e+00)/I_xx*w2*w3+(1/I_xx*I_yy*w2-1/I_xx*I_zz*w2)*psi27+(1/I_xx*I_yy*w3-1/I_xx*I_zz*w3)*psi26);
    acadodata_f3 << dot(psi26) == ((1/I_yy*I_zz*w1-I_xx/I_yy*w1)*psi27+(1/I_yy*I_zz*w3-I_xx/I_yy*w3)*psi25+1/I_yy*w1*w3);
    acadodata_f3 << dot(psi27) == ((I_xx/I_zz*w1-I_yy/I_zz*w1)*psi26+(I_xx/I_zz*w2-I_yy/I_zz*w2)*psi25-I_xx*pow(I_zz,(-2.00000000000000000000e+00))*w1*w2+I_yy*pow(I_zz,(-2.00000000000000000000e+00))*w1*w2-pow(I_zz,(-2.00000000000000000000e+00))*u6);

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 6 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 4 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 80 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-05 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "../NMPC_export" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

