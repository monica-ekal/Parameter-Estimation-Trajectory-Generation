%{ 
This script calls ACADO to generate MEX files of C++ system
simulator code. Edit and run this with correct paths to the ACADO folder 
ONLY if the dynamics need to be modified.

Inputs:
Correct file path to ACADO Toolkit, see below.

Outputs:
SIM_export folder, containg the MEX version of this code.
%}

%*** Specify file paths here
addpath(genpath('../../../ACADOtoolkit'));  % Set this to your ACADO Toolkit folder!
%*** Specify file paths here
clear all
acadoSet('results_to_file', false); 

%% ACADO declarations and setup
% Declare all differential states, controls, parameters
DifferentialState r(3) v(3) q(4) w(3)

Control u(6)  % [F_x F_y F_z T_x T_y T_z], force is in inertial frame; torque is in body frame

Ts = 1;

%% Set up simulator
Mass = 9.7;
I_xx = 7;
I_yy = 7;
I_zz = 10.0;
I = [I_xx 0    0;
     0    I_yy 0;
     0    0    I_zz];
 
% dynamic equations for 6DoF RBD about CoM
f = [ dot(r);  dot(v);dot(q); dot(w)] == ...
    [ v;               ...
      u(1:3)/Mass;     ...  % forces
      0.5*H_bar_T(q)*w; ...
      dot_w(I,w,u(4:6))];   % torques
   
%% Export simulation code
acadoSet('problemname', 'my_sim');
sim = acado.SIMexport(Ts); % sampling time
sim.setModel(f);
% sim.linkMatlabODE('ode_3DoF');

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        10      );

sim.exportCode( '../SIM_export' );
cd '../SIM_export'
make_acado_integrator('sim_6DoF')  % place in subdirectory
cd '../ACADO_generator'

%% Support functions
% quaternion conversion matrix: quaternion uses scalar last convention,
% w_IB expressed in body frame
function out = H_bar_T(q)
    out = [ q(4) q(3) -q(2) -q(1);
              -q(3) q(4) q(1) -q(2);
              q(2) -q(1) q(4) -q(3)]';
end

% dot_w calculation from xyz torque
function out = dot_w(I, w, tau)
    out = (-inv(I) * cross(w, I*w) + inv(I)*tau);
end