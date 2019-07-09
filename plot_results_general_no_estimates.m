function [ ] = plot_results_general_no_estimates(store_results, L, M, param, meas, F_horizon, ...
                                    F_total,control_ip,states,real_vals)
    % Plotting of UKF results
    t = store_results(:,1);
    x_hat = store_results(:,2:L+1);
    diag_cov = store_results(:,L+2:2*L+1);
    measured = store_results(:,2*L+2:2*L+1+M);
    gamma = store_results(:,end);
        
    figure(1)
    hold on
    for i = 1:size(x_hat,2)
    subplot(2,2,i)
    plot(t,x_hat(:,i))
    hold on
    plot(t,ones(size(t,1))*real_vals(i))
    xlabel('time')
    ylabel(param(:,i))
    legend(['Estimated ', param(:,i)]), grid
    title([param(:,i) ,' Estimation'])
    end

    figure()
    plot(t,diag_cov),xlabel('time (s)'),title('Diagonal of covariance matrix of the estimates, P'),grid
    legend(param)

    figure()
    subplot(2,1,1)
    hold on
    plot(t,F_horizon,'LineWidth',2)
    xlabel('time t')
    title('Fisher information over each MPC horizon')
    subplot(2,1,2)
    hold on
    plot(t, F_total(2:end),'LineWidth',2)
    xlabel('time t')
    ylabel('Cumulative sum of FIM')
    title('Total Fisher information over time, using MPC first input')
    
    figure()
    hold on
    plot(t,control_ip')
    title('Control iputs')
    legend ('u1','u2','u3','u4','u5','u6')
    
    figure(6)
    hold on
    plot( sqrt(sum(states.^2,2)))
    xlabel('Time')
    title('L2 norm of state')
    
    figure()
    plot( gamma)
    xlabel('Time')
    title('gamma')
end
