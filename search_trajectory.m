% search_trajectory: binary search used to find inputs that give a periodic
% trajectory
% James Zhu
% Carnegie Mellon University Robomechanics Lab
function search_trajectory

close all;

y_init = 2.5;
   
%% SLIP
mode = 3;
params = set_params();

jt = 0;
eps = 1e-6;
max_it = 100;
init_state = [0; 2; y_init; 0; 0; 0];

a = 0.23;
b = 0.28;

target_length = 0.5125; 

d = target_length + 0.02;
e = target_length - 0.02;
f = (d+e)/2;

[~,~,~,final_state,c] = bin_search...
    (a,b,init_state,[0;d],mode,params);
err_d = final_state(2) - init_state(2);

[~,~,~,final_state,c] = bin_search...
    (a,b,init_state,[0;e],mode,params);
err_e = final_state(2) - init_state(2);

[~,~,~,final_state,c] = bin_search...
    (a,b,init_state,[0;f],mode,params);
err_f = final_state(2) - init_state(2);

while (abs(err_f) > eps) && (jt < max_it)
        if sign(err_f) == sign(err_d)
            d = f;
        else
            e = f;
        end
        
        f = (d+e)/2;
        jt = jt + 1;
        
        target_inputs = [0;f];

        [transition_times,transition_states,...
            final_time,final_state,c] = bin_search...
                (a,b,init_state,target_inputs,mode,params);
        target_inputs(1) = c;
        
        err_f = final_state(2) - init_state(2);
end

% save initial conditions.
% important to save final time because controller will be clock-based
% periodic with period of final_time.

save('nom_conditions','init_state','transition_times', 'transition_states','final_time','final_state','target_inputs');

end
%%
function [transition_times,transition_states,...
    final_time,final_state,c] = bin_search...
    (a,b,init_state,target_inputs,mode,params)

eps = 1e-6;
max_it = 100;

it = 0;

[~,~,~,a_state] = simulate_return(init_state, [a;target_inputs(2:end)], 0, params);
[~,~,~,b_state] = simulate_return(init_state, [b;target_inputs(2:end)], 0, params);
c = (a+b)/2;
[~,~,~,c_state] = simulate_return(init_state, [c;target_inputs(2:end)], 0, params);

err_a = a_state(3) - init_state(3);
err_b = b_state(3) - init_state(3);
err_c = c_state(3) - init_state(3);

while (abs(err_c) > eps) && (it < max_it)
        if sign(err_c) == sign(err_a)
            a = c;
            err_a = err_c;
        else
            b = c;
        end
        
        c = (a+b)/2;
        it = it + 1;
        
        [transition_times,transition_states,...
    final_time,final_state] = simulate_return(init_state, [c;target_inputs(2:end)],0, params);
        
        if mode < 3
            err_c = final_state(1) - init_state(1);
        elseif floor(mode) == 3
            err_c = final_state(3) - init_state(3);
        elseif floor(mode) == 4
            err_c = final_state(2) - init_state(2);
        elseif floor(mode) == 5
            err_c = final_state(2) - init_state(2);
        end
end
end

%%
function [transition_times,transition_states,...
    final_time,final_state] = simulate_return(new_state, target_inputs, u, params)

vals = set_values();
num_domains = vals.num_domains;
prev_time = 0;
sim_params = set_sim_params();
time_span = sim_params.time_span;

state_vec = [];
time_vec = [];

for domain = 1:num_domains
    options = odeset('Events', @(t,x)guards(t,x,u,params,target_inputs,domain,[]), 'RelTol',1e-8);
    [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),prev_time+time_span,new_state,options);

    hold on
    plot(time,states(:,3));
    prev_time = time(end);
    transition_times{domain} = prev_time;
    end_state = states(end,:)';
    transition_states{domain} = end_state;
    
    time_vec = [time_vec; time];
    state_vec = [state_vec; states];

    new_state = resets(prev_time, end_state, domain, params, target_inputs);
end   
final_time = prev_time;
final_state = new_state;

end