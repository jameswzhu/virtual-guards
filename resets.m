function x_plus = resets(t, x, u, domain, params, target_inputs, varargin)

if nargin < 6
    target_inputs = get_inputs(t,x,u,params);
end

if domain == 1
    theta = target_inputs(1);
    ell = target_inputs(2);
        
    x_plus = x;

    x_plus(5) = x_plus(1) + ell.*sin(theta);
    x_plus(6) = x_plus(3) - ell.*cos(theta);
elseif domain == 2
    x_plus = x;
elseif domain == 3
    theta = target_inputs(1);
    ell = target_inputs(2);
        
    x_plus = x;

    x_plus(5) = x_plus(1) + ell.*sin(theta);
    x_plus(6) = x_plus(3) - ell.*cos(theta);
else
    error('Invalid Domain');
end