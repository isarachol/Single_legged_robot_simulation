function [bolddotx] = solve_6dof(x, damp)
   % Solve ODE for  6 dof versions of the single legged robot
   % Input: x = [q, dq] (12 variables)
   %        damp = damping constants (+) for each joint (defualt zero)

    arguments
        x
        damp {mustBeNumeric} = 0
    end

    n = size(x,1)/2;

    % check for errors
    assert(n==6, "The robot must have 6 DOF");

    tol = 10^(6);
    bolddotx = zeros(size(x));

    q = x(1:n);
    dq = x(n+1:end);
    bolddotx(1:n) = dq;

    M = M_6dof(q);
    C = C_6dof(q, dq)*dq;

    Q = zeros(n,1); % no external forces/toirques

    bolddotx(n+1:end) = M\(Q-C);

    bolddotx = numerical_check(bolddotx, tol);
end