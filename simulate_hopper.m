% simulate_system: using ode45, simulate orbits of hopper system
% James Zhu
% Carnegie Mellon University Robomechanics Lab
function [final_time, final_state, time_vec, state_vec, input_vec,domain_vec,...
    transition_times,transition_states]...
    = simulate_hopper(dx, u, options)

params = set_params();
sim_params = set_sim_params();

parameters = params.parameters;
time_span = sim_params.time_span;

cycles = options.cycles;
show_figure = options.show_figure;

vals = set_values();
y_coord = vals.y_coord;
num_domains = vals.num_domains;
state_1 = vals.state_1;
state_2 = vals.state_2;

nom_conditions = load('nom_conditions');
new_state = nom_conditions.init_state;
new_state = new_state + dx(1:length(new_state));
transition_states = nom_conditions.transition_states;
target_inputs = nom_conditions.target_inputs;
end_time = nom_conditions.final_time;

prev_time = dx(length(new_state)+1);
state_vec = [];
time_vec = [];
domain_vec = [];
input_vec = [];

for kk = 1:cycles
    for domain = 1:num_domains
        end_phase = end_time * (kk-1);
        options = odeset('Events', @(t,x)guards(t-end_phase,x,u,params,[],domain,end_time), 'RelTol',1e-8);
        [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),prev_time+time_span,new_state,options);

        if domain ~= 2
            for i = 1:length(time)
                inputs = get_inputs(time(i),states(i,1:4),u,params);
                theta = inputs(1); ell = inputs(2); 
                states(i,5:6) = [states(i,1) + ell.*sin(theta), states(i,3) - ell.*cos(theta)];
            end
        end
        
        prev_time = time(end);
        transition_times{domain} = prev_time;
        end_state = states(end,:)';
        transition_states{domain} = end_state';
        
        if show_figure
            if domain == 1
                c = 'g';
            elseif domain == 2
                c = 'r';
            elseif domain == 3
                c = 'c';
            end
            scatter(end_state(state_1), end_state(state_2),'filled',c);
        end
        
        time_vec = [time_vec; time];
        domain_vec = [domain_vec; domain*ones(size(time))];
        state_vec = [state_vec; states];
        
        if domain == 1
            torques = [0;0;0];
        elseif domain == 2
            torques = [0;0.75;0.2];
        elseif domain == 3
            torques = [0;0;0];
        elseif domain == 4
            torques = [0;0;0];
        elseif domain == 5
            torques = [-1;-1;0];
        elseif domain == 6
            torques = [0;0;0];
        end
        
        input_vec = [input_vec; repmat(torques',length(time),1)];
        
        
        new_state = resets(prev_time, end_state, u, domain, params)';
    end   
    
    if y_coord > 0
    constraint = find(state_vec(:,y_coord)<0);
        if ~isempty(constraint)
            state_vec = state_vec(1:constraint-1,:);
            time_vec = time_vec(1:constraint-1,:);
            break
        end
    end
end
    
final_time = prev_time;
final_state = new_state;
% input_vec = zeros(length(time_vec),length(target_inputs));
% for i = 1:length(input_vec)
%     input_vec(i,:) = get_inputs(time_vec(i), state_vec(i,end), u, params);
% end

if show_figure
    b = plot(state_vec(:,state_1),state_vec(:,state_2),'b','LineStyle','--');
    %i = plot(state_vec(:,1),input_vec(:,2).*cos(input_vec(:,1)),'k');
    %legend([b,i],'Body Trajectory','Input Trajectory','Location','southeast')
end