function [bolddotx] = solve_3dof(x, damp)
   % Solve ODE for 3 dof version of the single legged robot
   % Input: x = [q, dq] (6 variables)
   %        damp = damping constants (+) for each joint (defualt zero)

    arguments
        x
        damp {mustBeNumeric} = 0
    end

    n = size(x,1)/2;

    % check for errors
    assert(n==3, "The robot 3 DOF");

    tol = 10^(6);
    bolddotx = zeros(size(x));

    q = x(1:n);
    dq = x(n+1:end);
    bolddotx(1:n) = dq;

    % Kinetic energy
    M_inv = Minv_3dof(q);
    C = C_3dof(q, dq)*dq;

    % Potential energy
    g = dUgdq_3dof(q);

    % Loss
    Fd = damp.*dq;

    % General force
    Q = zeros(n,1); % no external forces/torques
    
    bolddotx(n+1:end) = M_inv*(Q-C-Fd-g);

    bolddotx = numerical_check(bolddotx, tol);
end