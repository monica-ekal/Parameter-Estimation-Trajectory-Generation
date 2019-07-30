%{
NMPC_6DoF_main.m

Main code for running information weighted NMPC using ACADO and a UKF.
The dynamics used are those of a 6 DoF rigid body with uncertain mass and
moments of inertia.

The state vector is X = [r(3)  v(3) q(4) w(3) psi(27)], see generate_NMPC_solver_6DoF.m.
Note that the augmented state psi has 6 + 7*3, or 27 elements,
discounting all the zeros that we dont want to waste time computing.

The variable gamma is found in NMPC.W(1,1).

Inputs:
Correct file paths, see below.
Initialization parameters, see below.

Outputs:
MATLAB workspace variables for plotting, see inputs to plot_results_general_no_estimates.m.

Keenan Albee, Monica Ekal, 2019.
%}

%*** Specify file paths for ACADO exports here. ../ is default.
 clear all
 addpath('../');
 addpath('NMPC_export');
 addpath('SIM_export');
%*** Specify file paths for ACADO exports here. ../ is default.
 
%% Initialization


Objective_value = [];
tf = 100;  % final time, s
Ts = 1;  % estimator time step, s
N = 40;  % horizon length
time = 1;  % start time, s

store_results = [];  % for logging UKF data
sim_results = [];  % for logging sim data
control_ip = [];  % for logging control data
sigma  = 0.005;  
r = [1e-2;1e-2; 1e-2; 1e-2; 1e-2;1e-2];  % SD of measurement and process noise
q = [1e-5; 1e-5; 1e-5; 1e-5];
        
m_est = 2; I_xx_est = 3.5; I_yy_est = 3.5; I_zz_est = 3.5;
inv_R = inv(diag([sigma*ones(6,1)]));
P1 = diag([20;20;20;20]);
x_hat = [m_est;I_xx_est;I_yy_est;I_zz_est];

estimator = Run_UKF_(x_hat, P1, Ts);
estimator.set_noise_properties(q, r);
estimator.set_y_tilde_prev([0; 0; 0; 0; 0; 0]);
%% Acado specific initializations

%% System simulator variables
state_sim = [2;2;2; 0;0;0; 0.2227177;0.0445435;0.4454354;0.8660254; 0;0;0.0]';  % the simulator (system) will have 13 states. 
% state_sim = [-1.5;-1.5;-2; 0.6030691; 0.3481821; 0.5509785; 0.4598907; 0;0;0; 0;0;0.0]';
% state_sim = [0;0;0; 0;0;0;1; 0;0;0; 0.0;0.0;0.0]';

%% NMPC variables
input_NMPC.x = repmat([state_sim'; zeros(27,1)]', N+1, 1);  % initial values of states, including the four added ones for FIM calculations
input_NMPC.u = repmat([ 0.1 ; 0.1; 0.1; 0.1;0.1;0.1]',N,1); % matrix of horizon number of control inputs
input_NMPC.W = eye(13);                     % weight matrix; the first element is for the FIM., the rest are for states
% input_NMPC.W(1,1) = 0;                    % set to 0 to see effects without FIM weighting
input_NMPC.WN = eye(12)*1e2;                % Weight for the terminal cost
input_NMPC.y = repmat(zeros(1,13), N, 1);
input_NMPC.yN = zeros(1,12);                % reference for the minimizing function

%% FIM variables
dh_x = [zeros(3) eye(3) zeros(3,7) ;
        zeros(3,10)         eye(3)   ];
F_local = zeros(N,1);
F_horizon = zeros(tf,1);
F_aggregate = 0;          % FIM over all horizons, corresponding to the applied (first) input of each MPC horizon
    
%% Main loop   

while time <= tf
    %% ACADO NMPC planning
    % use states from simulation of the previous step, and the last estimated mass
    input_NMPC.od = repmat([m_est, I_xx_est, I_yy_est, I_zz_est], N+1, 1);  % latest mass estimate for NMPC online data
    input_NMPC.x0 = [state_sim(end,:) zeros(1,27)];  % latest ground truth value, add zeros because of the extra states

    output = acado_NMPC_6DoF(input_NMPC);  % this step runs MPC

    output.info  % gives some info about the value of the objective after each MPC horizon
    Objective_value  = [Objective_value; output.info.objValue];
 
    % FIM calc over hoirzon using output.x
    psi = output.x(:,14:end);  % Grab psi propagation. Actually dx/d_theta is a jacobian, with columns: [dx/dmass dx/dIzz]. It has been vectorized.
    for k = 1:N+1
        psi_matrix1 = [psi(1:6)'] ;    
        psi_matrix2 = [ psi(7:13)'  psi(14:20)'  psi(21:27)']; 
        
        dx_theta = [psi_matrix1        zeros(6,3)    ;
                    zeros(7,1)         psi_matrix2  ];   % reassemble as a Jacobian, [n x j]
                
        tr_F_local(k)  = trace((dh_x*dx_theta)'*inv_R*(dh_x*dx_theta));
    end
    F_horizon(time) = sum(tr_F_local);
    F_aggregate = [F_aggregate; F_aggregate(end) + tr_F_local(2)];  % only add on the information actually gained


    % forward shifting: use NMPC's predicted next state results 
    input_NMPC.x = [output.x(2:end,:); output.x(end,:)]; 
    input_NMPC.u = [output.u(2:end,:); output.u(end,:)];

    %% ACADO Dynamics Simulation
    % prepare inputs and state for forward dynamics sim
    input_sim.x = state_sim(end,:).';       % latest state of simulation
    input_sim.u = output.u(1,:).';          % first input from NMPC optimization
    
    % Apply NMPC input and evaluated forward dynamics over timestep dt
    output_sim = sim_6DoF(input_sim);

    %% Store Data
    control_ip = [control_ip; output.u(1,:)];  % record applied u
    state_sim = [state_sim;
                 output_sim.value'];  % record ground truth data
    Norm(time) = norm(output.x(1,7:10));
    Avg_norm_over_Horizon(time) = sum(sum(output.x(:,7:10).^2,2))/N;
    
    % run parameter UKF
    z = [output_sim.value(4:6); output_sim.value(11:13)];
    z = z + sigma*randn(numel(z),1);

    vels_tilde_prev = [input_sim.x(4:6); input_sim.x(11:13)];
    vels_tilde_next = z;
    
    % --- NOTE: comment for comparison without updates regarding recent parameter estimates ---
    estimator.UKF_Loop_6DoF(input_sim.u,vels_tilde_next); 
    
    m_est = estimator.x_hat(1);
    I_xx_est = estimator.x_hat(2);
    I_yy_est = estimator.x_hat(3);
    I_zz_est = estimator.x_hat(4);
    
    info{1} = output_sim.value;
    info{2} = time;
    W = gamma_shifter(input_NMPC.W, info);
    input_NMPC.W = W;  % update gamma
    % --------

    time = time + Ts;
    P = diag(estimator.P1);  % for plotting diagonals of the covariance  
    
    store_results = [store_results;time, [m_est,I_xx_est,I_yy_est,I_zz_est], P', vels_tilde_next', input_NMPC.W(1,1)];
end

%% Misc plotting

plot_results_general_no_estimates(store_results,size(P),size(z,1),string({'Mass','Ixx','Iyy','Izz'}), 'velocities',F_horizon,F_aggregate,control_ip,state_sim,[9.7 7 7 10]) 
%norm(estimator.x_hat - [9.7;7;7;10],'fro') % frobenius norm of errors.


%% Utility functions

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

% Rotation matrix for R_I_B: quaternion uses scalar last convention
function [R] = Rotmat(q)
    R = [q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)),          2*(q(1)*q(3)-q(2)*q(4));...
             2*(q(1)*q(2)-q(3)*q(4)),        -q(1)^2+q(2)^2-q(3)^2 + q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));...
             2*(q(1)*q(3)+q(2)*q(4)),        2*(q(2)*q(3)-q(1)*q(4)),          -q(1)^2-q(2)^2+q(3)^2+q(4)^2]';
end

% Provides the error quaternion.
function [error_quat] = err_q2(q,q_des)
    error_quat = sqrt(2*(1 - abs(dot(q, q_des))));
end

% Updates gamma according to a specified shifting pattern.
function W_out = gamma_shifter(W, info)
    % gamma is the first term
    gamma = W(1, 1);
    x = info{1};
    t = info{2};
    tau = 20;  % time constant for decay by a factor of e

    % adjust gamma for decent tracking
    % options: distance to goal; quality of tracking; change in parameter
    % estimates; hard cutoff after enough time
    
    % for now, it just ramps down as you get closer to x_g
    gamma = 10.0*exp(1)^(-1/tau*t);
    gamma = gamma + norm(x)^2 -1; % quaternion scalar element does not let this go to zero
    if gamma > 20
        gamma = 20;
    end  % limit max
    
    W(1, 1) = gamma;  % set gamma
    W_out = W;
end