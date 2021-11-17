function dxVec = flows(t, x, u, domain, params)

g = params.g; m = params.m; l_0 = params.l_0;
k_l = params.k_l; k_a = params.k_a; b = params.b;
    
if domain == 1
    dxVec = zeros(6,1);
    
    x_dot = x(2);
    y_dot = x(4);
    
    dxVec(1,1) = x_dot;
    dxVec(2,1) = 0;
    dxVec(3,1) = y_dot;
    dxVec(4,1) = -g;
elseif domain == 2
    g = params.g; m = params.m; l_0 = params.l_0;
    k_l = params.k_l; k_a = params.k_a; b = params.b;

    x_B = x(1,1); x_dot = x(2,1);
    y_B = x(3,1); y_dot = x(4,1); 
    x_toe = x(5,1); y_toe = x(6,1);
    ell = sqrt((x_B - x_toe)^2 + (y_B - y_toe)^2);
    theta = atan2(x_toe - x_B,y_B - y_toe);
    
    x_c = x_B - x_toe; y_c = y_B - y_toe;

    F_s = k_l * (l_0-ell);
    F_b = -b * (x_c*x_dot + y_c*y_dot) / ell;

    dxVec = zeros(6,1);

    dxVec(1,1) = x_dot;
    dxVec(2,1) = (-(F_s+F_b)*sin(theta)) / m;
    dxVec(3,1) = y_dot;
    dxVec(4,1) = -g + ((F_s+F_b)*cos(theta)) / m;
elseif domain == 3
    dxVec = zeros(6,1);
    
    x_dot = x(2);
    y_dot = x(4);
    
    dxVec(1,1) = x_dot;
    dxVec(2,1) = 0;
    dxVec(3,1) = y_dot;
    dxVec(4,1) = -g;
else
    error('Invalid Domain');
end