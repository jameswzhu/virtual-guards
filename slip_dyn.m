clear; close all;
syms x y x_dot y_dot x_stance y_stance ell l_dot theta omega real
syms m k_l k_a b g l_0 real
%%
states_1 = [x_dot;y;y_dot];
states_2 = [x_stance;x_dot;y_stance;y_dot];
inputs = [theta; ell];
u = [omega; l_dot];
parameters = [m,k_l,k_a,b,g,l_0];

% mode 1 is descent. mode 2 is ascent

R12 = [-ell*sin(theta);x_dot;ell*cos(theta);y_dot];
DR12 = jacobian(R12,states_1);
R21 = [x_dot;y_stance;y_dot];
DR21 = jacobian(R21,states_2);
DtR12 = jacobian(R12,inputs) * u;
DtR21 = jacobian(R21,inputs) * u;

% stance dynamics in first state variables
theta_stance1 = theta;
ell_stance1 = ell;

F_s1 = k_l * (l_0-ell_stance1);
F_b1 = -b * (R12(1)*x_dot + R12(3)*y_dot) / ell_stance1;
%T_s1 = k_a * theta_stance1 / ell_stance1;

f11 = [0;y_dot;-g];
f21 = [R12(2);(-(F_s1+F_b1)*sin(theta_stance1))/m;...
    R12(4);-g+((F_s1+F_b1)*cos(theta_stance1))/m];
    
% stance dynamics in second state variables
theta_stance2 = atan2(-x_stance,y_stance);
ell_stance2 = sqrt(x_stance^2+y_stance^2);

F_s2 = k_l * (l_0-ell_stance2);
F_b2 = -b * (x_stance*x_dot + y_stance*y_dot) / ell_stance2;
%T_s2 = k_a * theta_stance2 / ell_stance2;

f12 = [0;R21(3);-g];
f22 = [x_dot;(-(F_s2+F_b2)*sin(theta_stance2) )/m;...
    y_dot;-g + ((F_s2+F_b2)*cos(theta_stance2) )/m];

g1 = y - ell*cos(theta);
Dg1 = jacobian(g1,states_1);
g2 = F_s2 + F_b2;
Dg2 = jacobian(g2,states_2);
Dtg1 = jacobian(g1,inputs) * u;
Dtg2 = jacobian(g2,inputs) * u;

Xi_1 = simplify(DR12+(f21 - DR12*f11 - DtR12)*Dg1/(Dtg1 + Dg1*f11));
Xi_2 = simplify(DR21+(f12 - DR21*f22 - DtR21)*Dg2/(Dtg2 + Dg2*f22));
dXi_omega = diff(Xi_1,u(1)); dXi_ell = diff(Xi_1,u(2));

matlabFunction(Xi_1,'File','calc_Xi_1','Vars',[{states_1},{inputs},{u},{parameters}])
matlabFunction(Xi_2,'File','calc_Xi_2','Vars',[{states_2},{parameters}])
matlabFunction(dXi_omega,'File','calc_dXi_omega','Vars',[{states_1},{inputs},{u},{parameters}])
matlabFunction(dXi_ell,'File','calc_dXi_ell','Vars',[{states_1},{inputs},{u},{parameters}])