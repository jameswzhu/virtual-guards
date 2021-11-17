function inputs = get_inputs(t,x,u,params)

vals =  set_values;
num_inputs = vals.num_inputs;

nom_conditions = load('nom_conditions');
init_state = nom_conditions.init_state;
target_inputs = nom_conditions.target_inputs;
transition_times = nom_conditions.transition_times;
final_time = nom_conditions.final_time;
t_phase = mod(t,final_time);


theta = target_inputs(1);
omega = u(1);
K = u(2);

d = 0.4;

x_dot_des = init_state(2);
x_dot = x(2);

a_1 = omega;
b_1 = theta - transition_times{1}.*a_1 + K .* (x_dot - x_dot_des);

x_1 = [a_1, b_1];

if abs(t_phase - transition_times{1}) < d
    x_p = polyval(x_1,t_phase);
else
    x_p = (polyval(x_1,transition_times{1}-d) - polyval(x_1,transition_times{1}+d)) / (final_time-2*d) ...
    .* mod(t_phase - transition_times{1} - d,final_time) + polyval(x_1,transition_times{1}+d);
end

ell = target_inputs(2);

a_2 = 0;
b_2 = ell;

x_2 = [a_2, b_2];

inputs = [x_p,b_2];