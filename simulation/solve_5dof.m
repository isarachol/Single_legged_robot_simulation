function [bolddotx] = solve_5dof(x, damp)
   % Solve ODE for 2, 3, 4, and 6 dof versions of the single legged robot
   % Input: x = [q, dq] (4, 6, 8, or 12 variables)
   %        damp = damping constants (+) for each joint (defualt zero)

    arguments
        x
        damp {mustBeNumeric} = 0
    end

    n = size(x,1)/2;

    % check for errors
    assert(n==5, "The robot must have 5 DOF");

    tol = 10^(6);
    bolddotx = zeros(size(x));

    q = x(1:n);
    dq = x(n+1:end);
    bolddotx(1:n) = dq;

    % Kinetic energy
    M = M_5dof(q);
    C = C_5dof(q, dq)*dq;

    % Potential energy
    g = dUgdq_5dof(q);

    % Loss
    Fd = damp.*dq;

    % General force
    Q = zeros(n,1); % no external forces/torques

    bolddotx(n+1:end) = M \ (Q-C-Fd-g);

    bolddotx = numerical_check(bolddotx, tol);
end