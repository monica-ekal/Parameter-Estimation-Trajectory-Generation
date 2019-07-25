%{ 
This script calls ACADO to generate MEX files of C++ NMPC code.
Edit and run this with correct paths to the ACADO folder 
ONLY if the parameters have been changed from the first initialization.

Inputs:
Correct file path to ACADO Toolkit, see below.

Outputs:
NMPC_export folder, containg the MEX version of this code.
%}

%*** Specify file paths here
addpath(genpath('../../../ACADOtoolkit'));  % Set this to your ACADO Toolkit folder!
%*** Specify file paths here

clear all
acadoSet('results_to_file', false); 

%% ACADO declarations and setup
% Declare all differential states, controls, parameters
DifferentialState r(3)  v(3) q(4) w(3) psi(27)  % psi is used for FIM calc

Control u(6)  % [F_x F_y F_z T_x T_y T_z], force is in inertial frame; torque is in body frame

OnlineData Mass I_xx I_yy I_zz  % excluding: products of inertia and CoM position

dt = 1;  % length of step
N = 40;  % length of horizon
    
%% Dynamic equations.

%TODO: these need to be estimated from the KF
% I_xx = 10.0;
% I_yy = 10.0;
% I_zz = 10.0;

I = [I_xx 0    0;
     0    I_yy 0;
     0    0    I_zz];
 
% dynamic equations for 6DoF RBD about CoM
f = [ dot(r); dot(v); dot(q); dot(w); dot(psi)] == ...
    [ v;               ...
      u(1:3)/Mass;     ...   % forces
      0.5*H_bar_T(q)*w; ...
      dot_w(I,w,u(4:6)); ... % torques
      Calc_psidot(r,v,q,w,Mass,I_xx,I_yy,I_zz,u,psi)];   
   
%% Set up NMPC (ocp) solver
% W_mat = eye(11);  % state error, 13 DoF plus 1 for information term. q_err is compressed down to 1 metric, otherwise dim = 13
W_mat = eye(13);  % state error, for q
WN_mat = eye(12);  % terminal cost, 13 DoF
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp = acado.OCP(0.0, N*dt, N);  % start time 0, end time, number of intervals: OCP(tStart, tEnd, N)

% objective
% TO DO: add fuel use
x_des = [0 0 0 0 0 0 1 0 0 0 0 0 0]';  % desired final state, doesn't include FIM terms
r_des = x_des(1:3);
q_des = x_des(4:7);
v_des = x_des(8:10);
w_des = x_des(11:13);

ocp.minimizeLSQ( W,  [1/(trace(CalcFIM(psi))+1);r-r_des;  v-v_des;err_q(q,q_des); w-w_des]);% r-r_des; err_q(q,q_des); v-v_des; w-w_des]);  % Minimize inverse FIM + error between states, 1 to avoid singularity
x_g =  [0;0;0; 0;0;0;1; 0;0;0; 0;0;0];% desired final state
ocp.minimizeLSQEndTerm( WN, [r; err_q(q,q_des); v; w]);

% constraints
ocp.subjectTo( -0.05 <= u(1:3) <= 0.05);  % control constraints

ocp.subjectTo( -0.05 <= u(4:6) <= 0.05);  % control constraints
ocp.setModel(f);  % constraint from dynamic model

%% Export NMPC code
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-5				);  % steps over horizon

mpc.exportCode('../NMPC_export');  % export code and compile
copyfile('../../../../ACADOtoolkit/external_packages/qpoases', '../NMPC_export/qpoases', 'f'); % supply path to ACADO toolkit
cd '../NMPC_export'
make_acado_solver('acado_NMPC_6DoF')  % place in subdirectory
cd '../ACADO_generator';

%% Support functions

% quaternion metric for cost function
function [error_quat] = err_q2(q,q_des)
%     error_quat = sqrt((q'*q_des)^2);
    error_quat = sqrt(2*(1 - sqrt((q'*q_des)^2)));
end

% quaternion error angle calculation for cost function
function [error_quat] = err_q(q,q_des)
    error_quat = invskew(Rotmat(q_des)'*Rotmat(q) - Rotmat(q)'*Rotmat(q_des))/(2*sqrt(1+tr(Rotmat(q_des)'*Rotmat(q))));
end
function [skew_v] = skew(vec)
    skew_v = [0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];  % Cross product matrix
end

function [vec] = invskew(A)
    vec = [A(3,2); A(1,3); A(2,1)];
end
function [Trace] = tr(A)
    Trace =  A(1,1) + A(2,2) + A(3,3);  % Trace
end

% rotation matrix for R_I_B: quaternion uses scalar last convention
function [R] = Rotmat(q)
    R = [q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)),          2*(q(1)*q(3)-q(2)*q(4));...
             2*(q(1)*q(2)-q(3)*q(4)),        -q(1)^2+q(2)^2-q(3)^2 + q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));...
             2*(q(1)*q(3)+q(2)*q(4)),        2*(q(2)*q(3)-q(1)*q(4)),          -q(1)^2-q(2)^2+q(3)^2+q(4)^2]';
end

% quaternion conversion matrix: quaternion uses scalar last convention,
% w_IB expressed in body frame
function out = H_bar_T(q)
    out = [ q(4) q(3) -q(2) -q(1);
              -q(3) q(4) q(1) -q(2);
              q(2) -q(1) q(4) -q(3)]';
end

% dot_w calculation from xyz torque
function out = dot_w(I, w, tau)
% unfortunately, inverse does not work for ACADO online data.
% We could 1.pass the inverse of I, element by element,  as online data each time,
% 2. As this is a simple diagonal matrix, directly use 1/diagonals.
    inv_I = [1/I(1,1)   0         0    ;
                0      1/I(2,2)   0    ;
                0       0        1/I(3,3)];
    out = (-inv_I * cross(w, I*w) + inv_I*tau);
end

% calculates df/dx for the quaternion portion, q, of the dynamics
function df_dx = quat_df_dx(q, w)
    df_dx = [ 0     w(3) -w(2) w(1)  q(4) -q(3)  q(2);
             -w(3)   0    w(1) w(2)  q(3)  q(4) -q(1);
              w(2) -w(1)  0    w(3) -q(2)  q(1)  q(4);
             -w(1) -w(2) -w(3)  0   -q(1) -q(2) -q(3)];
end

function [FIM] = CalcFIM(psi)

% measurements = v_x, v_y,v_z
    dh_x1 = [ 0 0 0 1 0 0;
              0 0 0 0 1 0;
              0 0 0 0 0 1];  % dh/dx == dy/dx
    
% measurements w_x, w_y, w_z
    dh_x2 = [0 0 0 0 1 0 0;
             0 0 0 0 0 1 0;
             0 0 0 0 0 0 1];  % dh/dx == dy/dx
         
    sigma  = 0.01*ones(1,1);  
    inv_R = inv(diag([sigma;sigma;sigma]));
    
    psi_matrix1 = [psi(1:6)];
    
    psi_matrix2 = [ psi(7:13)  psi(14:20)  psi(21:27)];
    
    FIM1 = ((dh_x1*psi_matrix1)'*inv_R*(dh_x1*psi_matrix1));    % H' * inv_R * H = [1 x 1]
    FIM2 = ((dh_x2*psi_matrix2)'*inv_R*(dh_x2*psi_matrix2));    % H' * inv_R * H = [1 x 1]
    
    FIM =[ FIM1   zeros(1,3);
           0      FIM2     ]
    
%     % reassemble the psi_matrix by using the separated psi states. The
%     % psi_matrix will be 7x3.
%     FIM = [FIM1_1          0;
%              0            FIM2_2];  % [j x j]
%          
    % convert to a usable scalar

end

function[psi_dot] = Calc_psidot(x,v,q,w,Mass,I_xx,I_yy,I_zz,u,psi)
     % x_dot = f(x,theta,T);
%     f = [ w_z; u(3)/I_zz ];
%     df_x = simplify(jacobian(f, [theta_z; w_z]));  % Jacobian (df/dx)
%     df_theta = simplify(jacobian(f, [I_zz])); % Jacobian (df/dtheta)
%     psi_dot = df_x*psi + df_theta;  % d/dt[ dx/dtheta ]
    
    I = [I_xx 0    0;
         0    I_yy 0;
         0    0    I_zz];


    %x_dot = f(x,theta,T);
    f1 = [v;u(1:3)/Mass];                       % position
    f2 = [0.5*H_bar_T(q)*w; dot_w(I,w,u(4:6))]; % attitude


    df_x1 = simplify(jacobian(f1,[x;v]));  % Jacobian (df/dx)
    df_x2 = [0.5*quat_df_dx(q, w); simplify(jacobian(f2(5:end,:),[q;w]))];           % attitude


    df_theta1 =  simplify(jacobian(f1,Mass)); % Jacobian (df/dtheta)
    df_theta2 =  simplify(jacobian(f2,[I_xx;I_yy;I_zz]));


%     psi_dot(1:6,1) = df_x1*psi(1:6)+df_theta1;
    psi_dot(1:6,1) = df_x1*psi(1:6)+df_theta1; %df/dMass  % d/dt [ dx/dtheta ]
    psi_dot(7:13,1) = df_x2*psi(7:13)+df_theta2(:,1); %df/dI_xx  % d/dt [ dx/dtheta ]
    psi_dot(14:20,1) = df_x2*psi(14:20)+df_theta2(:,2); %df/dI_xx  % d/dt [ dx/dtheta ]
    psi_dot(21:27,1) = df_x2*psi(21:27)+df_theta2(:,3); %df/dI_xx  % d/dt [ dx/dtheta ]

    psi_dot
end