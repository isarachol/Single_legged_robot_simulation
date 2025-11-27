function [bolddotx] = solve_4dof(x, ddx_prev, damp)
   % Solve ODE for 2, 3, 4, and 6 dof versions of the single legged robot
   % Input: x = [q, dq] (4, 6, 8, or 12 variables)
   %        damp = damping constants (+) for each joint (defualt zero)

    arguments
        x
        ddx_prev
        damp {mustBeNumeric} = 0
    end

    n = size(x,1)/2;

    % check for errors
    assert(n==4, "The robot must have 4 DOF");

    tol = 10^(6);
    bolddotx = zeros(size(x));

    q = x(1:n);
    dq = x(n+1:end);
    bolddotx(1:n) = dq;

    % Kinetic energy
    M = M_4dof(q);
    C = C_4dof(q, dq)*dq;

    % Potential energy
    g = dUgdq_4dof(q);

    % Loss
    Fd = damp.*dq;

    % General force
    Q = zeros(n,1); % no external forces/torques

    if det(M) < 10^(-13) % possible singularity
        bolddotx(n+1:end) = ddx_prev; % use previous one to move out of singularity
        disp("skip singularity");
    else
        bolddotx(n+1:end) = M\(Q-C);
    end

    bolddotx = numerical_check(bolddotx, tol);
end