function [bolddotx] = solve_ndof(x, damp, method)
   % Solve ODE for 2, 3, 4, and 6 dof versions of the single legged robot
   % Input: x = [q, dq] (4, 6, 8, or 12 variables)
   %        damp = damping constants (+) for each joint (defualt zero)

    arguments
        x
        damp {mustBeNumeric} = 0
        method {mustBeNumeric} = 0
    end

    n = size(x,1)/2;

    % check for errors
    assert(n==2 || n==3 || n==4 || n==6, "The robot must have 2, 3, 4, or 6 DOF");

    tol = 10^(6);
    bolddotx = zeros(size(x));

    q = x(1:n);
    dq = x(n+1:end);
    bolddotx(1:n) = dq;

    if n==2
        M_inv = Minv_2dof(q);
        C = C_2dof(q, dq)*dq;
    elseif n==3
        M_inv = Minv_3dof(q);
        C = C_3dof(q, dq)*dq;
    elseif n==4
        M_inv = Minv_4dof(q);
        C = C_4dof(q, dq);
    elseif n==6
        M = M_6dof(q);
        M_inv = inv(M); % recalculate every time
        C = C_6dof(q, dq)*dq;
    end

    Q = zeros(n,1); % no external forces/toirques

    bolddotx(n+1:end) = M_inv*(Q-C);

    bolddotx = numerical_check(bolddotx, tol);
end