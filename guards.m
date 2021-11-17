function [value,isterminal,direction] = guards(t,x,u,params,inputs,domain,end_time)
if isempty(inputs)
    inputs = get_inputs(t,x,u,params);
    %g_end = end_time - t;
    g_end = x(4);
else
    g_end = x(4);
end

y = x(3);
theta = inputs(1); ell = inputs(2);

y_toe = y  - ell*cos(theta);
g_1 = y_toe;

l_0 = params.l_0; k_l = params.k_l; b = params.b;

x_B = x(1,1); x_dot = x(2,1);
y_B = x(3,1); y_dot = x(4,1); 
x_toe = x(5,1); y_toe = x(6,1);

x_c = x_B - x_toe; y_c = y_B - y_toe;
ell = sqrt(x_c^2 + y_c^2);

F_s = k_l * (l_0-ell);
F_b = -b * (x_c*x_dot + y_c*y_dot) / ell;

g_2 = F_s + F_b;

g = [g_1;g_2;g_end];

% Check all contact conditions with this value
value = g(domain); % The value that we want to be zero
isterminal = ones(size(g(domain)));  % Halt integration and change contact modes
direction = -ones(size(g(domain)));   % The zero can only be approached from this direction
end