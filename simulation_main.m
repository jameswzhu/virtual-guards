% simulate_system: using ode45, simulate orbits of hopper system
% James Zhu
% Carnegie Mellon University Robomechanics Lab
function simulation_main()

close all; hold on;

f = figure();
f.Position = [340,246,1554,750];

params = set_params();
sim_params = set_sim_params();

parameters = params.parameters;
time_span = sim_params.time_span;

trials = 20;
cycles = 100;
show_figure = 1;

vals = set_values();
y_coord = vals.y_coord;
num_domains = vals.num_domains;
state_1 = vals.state_1;
state_2 = vals.state_2;

nom_conditions = load('nom_conditions');
transition_states = nom_conditions.transition_states;
target_inputs = nom_conditions.target_inputs;
end_time = nom_conditions.final_time;

u = [-0.4,0.129];
show_figure = 0;

div = cell(trials-1,cycles);

for ii = 1:trials
    
    dx = zeros(6,1);
    
    if ii  > 1
        dx(2:4) = 0.1 - 0.2* rand(3,1);
    end
    
    new_state = nom_conditions.init_state + dx;
    
    if show_figure
        s = scatter(new_state(state_1), new_state(state_2),60,'c','filled','MarkerEdgeColor','k');
        uistack(s,'top');
    end
    
    prev_time = 0;
    state_vec = [];
    time_vec = [];
    domain_vec = [];
    
    for kk = 1:cycles
        for domain = 1:num_domains
            end_phase = end_time * (kk-1);
            options = odeset('Events', @(t,x)guards(t-end_phase,x,u,params,[],domain,end_time), 'RelTol',1e-8);
            [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),prev_time+time_span,new_state,options);

            prev_time = time(end);
            end_state = states(end,:)';

            time_vec = [time_vec; time];
            domain_vec = [domain_vec; domain*ones(size(time))];
            state_vec = [state_vec; states];        
            
            if show_figure
                plot(states(:,state_1),states(:,state_2),'Color',[0,0,0,0.04]);
            end

            new_state = resets(prev_time, end_state, u, domain, params)';
        end   
        
        final_time = prev_time;
        final_state = new_state;
        
        if ii > 1
            div{ii-1,kk} = norm(final_state(2:4)' - nom_conditions.init_state(2:4));
        end

    end
    
    if show_figure
        e = scatter(new_state(state_1), new_state(state_2),130,'r','p','filled','MarkerEdgeColor','k');
        uistack(e,'top');
    end
end

f = figure();
f.Position = [340,246,1554,750];

hold on;

for ii = 1:trials-1
    plot(1:kk,cell2mat(div(ii,:)),'Color',[0,0,0, 0.15]);
end

plot(1:kk,mean(cell2mat(div)),'Color',[0,0,0, 1],'LineWidth',4);

export_fig('C:/Users/James/Documents/CMU/Hybrid Event Shaping/Paper Figures/hopper_convergence','-opengl', '-eps');

xlabel('Step \#');
ylabel('Error ($||x-\bar{x}||$)');