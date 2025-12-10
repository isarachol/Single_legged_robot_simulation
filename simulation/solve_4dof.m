function [bolddotx] = solve_4dof(x, damp, grav)
   % Solve ODE for 4 dof versions of the single legged robot
   % Input: x = [q, dq] (8 variables)
   %        damp = damping constants (+) for each joint (defualt zero)

    arguments
        x
        damp {mustBeNumeric} = 0
        grav {mustBeNumeric} = 1
    end
    n = size(x,1)/2;
    assert(n==4, "The robot must have 5 DOF"); % check for errors
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

    if grav == 0
        bolddotx(n+1:end) = M \ (Q-C-Fd);
    else
        bolddotx(n+1:end) = M \ (Q-C-Fd-g);
    end

    bolddotx = numerical_check(bolddotx, tol);
end