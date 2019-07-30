%{
Run_UKF_

This Unscented Kalman Filter implementation estimates mass and three inertial
parameters (principal moments of inertia). Most robot localization algorithms fuse
inertial and absolute measurements to generate state estimates. We assume that pose data
is available after such a multi-sensor fusion.

Initialized using Run_UKF_()

Inputs:
y_tilde - angular velocities
FandT - applied torques

Outputs: 
Estimated mass, Ixx, Iyy and Izz and their covariances

%}
classdef Run_UKF_ < handle
  
    
     properties
       P1
       x_hat
     end   
%     
     properties(GetAccess = 'private', SetAccess = 'private')
        L
        x_hat_
        y_hat
        sizeX_sigmapts
        sizeY_sigmapts
        dt
        w_c
        w_m
        R
        Q
        eta
        y_tilde_prev
        X_sigmapts
        Y_sigmapts
    end
    

        
        
    methods
    


         function obj = Run_UKF_(I_hat_init,P,dt)
            if(nargin > 0)
                obj.L = numel(I_hat_init);
                obj.dt = dt;                                       % time step
                alpha = 1e-4;                                      % determines spread of sigma points around x_hat. Usually, 1e-4<alpha<1
                beta = 2;                                          % incorporates prior knowledge about the distribution of x. 
                lambda = obj.L*(alpha^2 - 1);                          % value of 2 is optimal for Gaussian dist
                obj.eta = sqrt(obj.L + lambda);                        % both lambda and eta are scaling parameters
                c = (obj.L+lambda);
                obj.w_m = [lambda/c 0.5/c*ones(1,2*obj.L)] ;               % weights for mean
                obj.w_c = [lambda/c + (1-alpha^2 + beta) obj.w_m(2:end)];  % weights for covariance
                obj.P1 = P;                                                % covariance of estimates
                obj.x_hat = I_hat_init;                                    % intial guess
            end
         end
                 
         function set_noise_properties(obj,q,r) 
            obj.sizeX_sigmapts = 2*numel(q) + 1;
            obj.sizeY_sigmapts = 2*numel(q) + 1;
            obj.R = diag(r.^2);                                    % Measurement covariance
            obj.Q = diag(q.^2);                                    % Process noise covariance
         end
             
         
        function UKF_Loop_6DoF(obj,FandT,y_tilde)                                   
            
            % Find sigma points,X_sigmapts
            obj.generate_SigmaPoints();                      
            % no propagation, inertial parameters are assumed to be constant and have zero process noise
    
            %measurement prediction, Y_sigmapts
            obj.measurement_prediction(FandT);  

            %calculation of mean and covariances
            [obj.x_hat_,P1_]= obj.unscented_transform(obj.X_sigmapts,obj.sizeX_sigmapts,obj.Q);              
            [obj.y_hat,P2_ ] = obj.unscented_transform(obj.Y_sigmapts,obj.sizeY_sigmapts,obj.R);

            %computing cross covariance and Kalman gain
            P12 = obj.Cross_covariance();
            K = P12/P2_;
            
            % update
            obj.x_hat = obj.x_hat_ + K*(y_tilde - obj.y_hat);
            obj.P1 = P1_ -  K*P2_*K'; 
            
            % prepare for next step
            obj.set_y_tilde_prev(y_tilde);
        end
               
        function generate_SigmaPoints(obj)
            l = chol(obj.P1, 'lower');
            x_rep = repmat(obj.x_hat,1,obj.L);
            obj.X_sigmapts = [obj.x_hat  x_rep+obj.eta*l x_rep-obj.eta*l];
        end
        
        function measurement_prediction(obj,FandT)
            for k = 1:obj.sizeX_sigmapts
                Mass = obj.X_sigmapts(1,k);
                Inertia = diag(obj.X_sigmapts(2:4,k));
                dot_v = @(Mass,Forces) (inv(Mass))*(Forces);
                dot_w = @(I,w,Moments) (inv(I))*(Moments - cross(w,I*w));
                v_k = obj.y_tilde_prev(1:3);
                w_k = obj.y_tilde_prev(4:6);
                v_kplus1 = v_k + obj.dt*dot_v(Mass,FandT(1:3));
                w_kplus1 = w_k + obj.dt*dot_w(Inertia,w_k,FandT(4:6));
                Y_states_hat(:,k)  = [v_kplus1;w_kplus1]   ;               
            end
            
            obj.Y_sigmapts = Y_states_hat;
        end
        
        
        function [mean, cov_mat] = unscented_transform(obj,sigmaPt_vector,size_SigmaptsVec,noise_cov)
            for j = 1:size_SigmaptsVec
                mean_(:,j) = obj.w_m(j)*sigmaPt_vector(:,j);
            end

            mean = sum(mean_,2);
            cov_mat = (sigmaPt_vector - repmat(mean,1,size_SigmaptsVec))*diag(obj.w_c)*(sigmaPt_vector - repmat(mean,1,size_SigmaptsVec))' + noise_cov;
        end
        

         
        function [P12] =  Cross_covariance(obj)
            P12 = [obj.X_sigmapts - repmat(obj.x_hat_,1,obj.sizeX_sigmapts)]*diag(obj.w_c)*[obj.Y_sigmapts - repmat(obj.y_hat,1,obj.sizeX_sigmapts)]'; % cross-covariance
        end
        
       function [obj] = set_y_tilde_prev(obj,y_tilde)
            obj.y_tilde_prev = y_tilde;         % prepare for next step
        end
    end
end
