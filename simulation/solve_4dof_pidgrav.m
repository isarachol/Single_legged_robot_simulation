function res = solve_4dof_pidgrav(x, xd, sigma, dt, damp, grav)
   % Solve ODE for 4 dof manipulator with desired states xd
   % Assume xd = R^4x1

    arguments
        x
        xd
        sigma
        dt
        damp {mustBeNumeric} = 0
        grav {mustBeNumeric} = 1 % not used
    end
    n = size(x,1)/2;
    assert(n==4, "The robot must have 5 DOF"); % check for errors
    tol = 10^(6);

    % States
    bolddotx = zeros(size(x));
    q = x(1:n);
    dq = x(n+1:end);
    bolddotx(1:n) = dq;

    % Controller gains
    Kp = [10;10;10;10];
    Kd = [0.2;0.2;0.2;0.2];
    Ki = [0.000000000001;0.000000000001;0.000000000001;0.000000000001];

    % I controller param
    e = q - xd;
    edt = e*dt;
    sigma = sigma + edt;

    % Internal dynamics (LHS)
    M = M_4dof(q);
    C = C_4dof(q, dq)*dq;
    g = dUgdq_4dof(q); % Potential energy
    Fd = damp.*dq; % Viscous friction

    % External forces/torques (RHS)
    Q = zeros(n,1); % no external forces/torques
    u = -Kp.*e - Kd.*dq - g - Ki.*sigma;% PD+grav Control

    bolddotx(n+1:end) = M \ (Q-C-Fd-g + u);
    bolddotx = numerical_check(bolddotx, tol);

    res.bolddotx = bolddotx;
    res.u = u;
    res.sigma = sigma;
end