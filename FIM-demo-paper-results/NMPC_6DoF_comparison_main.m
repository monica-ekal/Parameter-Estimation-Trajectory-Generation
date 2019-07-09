% Main code for running information weighted MPC from Acado +UKF 
 clear all
 addpath(genpath('../../ukf-estimator/UKF 6DoF mass and Ixx Iyy Izz'));
 addpath(genpath('../../plotting'));
 addpath('../');
 addpath('NMPC_export');
 addpath('SIM_export');
%  close all

tf = 100;

%%%--- extra variables needed for plotting in the paper. comment this section otherwise
num_runs = 10;
x_hat_all = zeros(tf,6,num_runs);
P_all = zeros(tf,4,num_runs);  % set up 3D matrices for housing data from 100 runs.
State_sim_all = zeros(tf+1,13,num_runs); % store the trajectory followed by the robot
Control_ip_all = zeros(tf,6,num_runs);

%% With update, no weighting
for run_number = 1:num_runs
%%%---

%% Initialization
% estimator = RunUKF([2;0;2;0]);      % Assumed initial states % Initialize with first measurements
Objective_value = [];
Ts = 1;
N = 40;  % horizon length
time = 1;  % start time

store_results = [];                 % for logging UKF data
sim_results = [];                   % for logging sim data
control_ip = [];                    % for logging control data
sigma  = 0.005;  

m_est = 2; I_xx_est = 3.5; I_yy_est = 3.5; I_zz_est = 3.5;
inv_R = inv(diag([sigma*ones(6,1)]));% P0 = (0.5(xu-xl))'*(0.5(xu-xl)),xu = 12, xl = 5
P1 = diag([20;12.25;12.25;12.25]);
x_hat = [m_est;I_xx_est;I_yy_est;I_zz_est]; %[0.5]; % x0_hat = 0.5*(xu + xl); %
%(based on Schneider and Georgakis, 2013. How to not make the extended Kalman filter fail.)
estimator = Run_UKF(x_hat, P1);

%% Acado specific initializations

%% System simulator variables
state_sim = [2;2;2; 0;0;0; 0.2227177;0.0445435;0.4454354;0.8660254; 0;0;0.0]';  % the simulator (system) will have 13 states. 
% state_sim = [-1.5;-1.5;-2; 0.6030691; 0.3481821; 0.5509785; 0.4598907; 0;0;0; 0;0;0.0]';
% state_sim = [0;0;0; 0;0;0;1; 0;0;0; 0.0;0.0;0.0]';
%% NMPC variables
input_NMPC.x = repmat([state_sim'; zeros(27,1)]', N+1, 1); % initial values of states, including the four added ones for FIM calculations
input_NMPC.u = repmat([ 0.1 ; 0.1; 0.1; 0.1;0.1;0.1]',N,1);               % matrix of horizon number of control inputs
input_NMPC.W = eye(13);                   % weight matrix; the first element is for the FIM., the rest are for states
input_NMPC.W(1,1) = 0;             % to see effects with and without FIM
% input_NMPC.W(5,5) = 1000;  % quaternion error metric
input_NMPC.WN = eye(12)*1e2;              % Weigh for the terminal cost
input_NMPC.y = repmat(zeros(1,13), N, 1);
input_NMPC.yN = zeros(1,12);              % reference for the minimizing function, what does it do?

%% FIM variables
dh_x = [zeros(3) eye(3) zeros(3,7) ;
        zeros(3,10)         eye(3)   ]
    
F_local = zeros(N,1);
F_horizon = zeros(tf,1);
F_aggregate = 0;          % FIM over all horizons, corresponding to the applied (first) input of each MPC horizon
    
%% Main loop   
while time <= tf
    %% ACADO NMPC planning
    % use states from simulation of the previous step, and the last estimated mass
    % TODO: don't use set mass
%     m_est = 9.7;
    input_NMPC.od = repmat([m_est, I_xx_est, I_yy_est, I_zz_est], N+1, 1);  % latest mass estimate for NMPC online data
    input_NMPC.x0 = [state_sim(end,:) zeros(1,27)];  % latest ground truth value, add zeros because of the extra states
    tic
    output = acado_NMPC_6DoF(input_NMPC);  % this step runs MPC
    toc
    output.info  % gives some info about the value of the objective after each MPC horizon
    Objective_value  = [  Objective_value; output.info.objValue];
 
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
    
    
    disp(time);
%     sum(err_q(state_sim(end,4:7), [0,0,0,1]))
%     err_q2(state_sim(end,4:7), [0,0,0,1])

    % run UKF
    z = [output_sim.value(4:6); output_sim.value(11:13)];
    z = z + sigma*randn(numel(z),1);

    vels_tilde_prev = [input_sim.x(4:6); input_sim.x(11:13)];
    vels_tilde_next = z;
    [estimator] = estimator.UKF_Loop_6DoF(Ts,input_sim.u,vels_tilde_next',vels_tilde_prev'); 
    
    %% comment for comparison without updates + without weighting
    m_est = estimator.x_hat(1);
    I_xx_est = estimator.x_hat(2); I_yy_est = estimator.x_hat(3); I_zz_est = estimator.x_hat(4);   % comment this for comparison without updating

    
    info{1} = output_sim.value;  % comment this for comparison without weighting
    info{2} = time;
    W = gamma_shifter(input_NMPC.W, info);
    input_NMPC.W = W;
    %%%%%%%%%%%%
    input_NMPC.W(1,1)


    time = time + Ts;
    P = diag(estimator.P1);  % for plotting diagonals of the covariance  

    store_results = [store_results;time, [m_est,I_xx_est,I_yy_est,I_zz_est], P', vels_tilde_next', estimator.y_hat', input_NMPC.W(1,1)];
end

%%%--- an add on plotting things for the paper. comment otherwise.
x_hat_all(:,:,run_number) = store_results(:,2:7);
P_all(:,:,run_number) = store_results(:,4:7);
State_sim_all(:,:,run_number) = state_sim;
Control_ip_all(:,:,run_number) = control_ip;
% anim_FK3(state_sim(:,1:3), state_sim(:,7:10));

end

P_avg = sum(P_all,3)./num_runs;
x_hat_avg  = sum(x_hat_all,3)./num_runs;
State_sim_avg = sum(State_sim_all,3)./num_runs;
Control_ip_avg = sum(Control_ip_all,3)./num_runs;
store_results(:,2:11) = [x_hat_avg P_avg];
plot_results_paper_avg(store_results,size(P_avg,2),size(z,1),'n/a', 'velocities', F_horizon, F_aggregate, Control_ip_avg, State_sim_avg) 










%%%---


% anim_FK3(state_sim(:,1:3), state_sim(:,7:10));
% plot_results_general(store_results,size(P),size(z,1),string({'Mass','Ixx','Iyy','Izz'}), 'velocities',F_horizon,F_aggregate,control_ip,state_sim,[9.7 7 7 10]) 


% figure()
% subplot(2,1,1)
% plot(1:tf,Norm)
% title(' Quat Norm, NMPC first propagated state')
% subplot(2,1,2)
% plot(1:tf,Avg_norm_over_Horizon)
% title('Quat Avg norm over each horizon')





%% Assitant functions
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

function [error_quat] = err_q2(q,q_des)
    error_quat = sqrt(2*(1 - abs(dot(q, q_des))));
end

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
    
    gamma
    W(1, 1) = gamma;  % set gamma
    W_out = W;
end