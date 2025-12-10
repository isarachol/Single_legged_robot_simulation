function res = solve_4dof_pdgrav(x, xd, damp, grav)
   % Solve ODE for 4 dof manipulator with desired states xd
   % Assume xd = R^4x1

    arguments
        x
        xd
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

    % Internal dynamics (LHS)
    M = M_4dof(q);
    C = C_4dof(q, dq)*dq;
    g = dUgdq_4dof(q); % Potential energy
    Fd = damp.*dq; % Viscous friction

    % External forces/torques (RHS)
    Q = zeros(n,1); % no external forces/torques
    u = -Kp.*(q-xd) - Kd.*dq - g;% PD+grav Control

    bolddotx(n+1:end) = M \ (Q-C-Fd-g + u);
    bolddotx = numerical_check(bolddotx, tol);

    res.bolddotx = bolddotx;
    res.u = u;
end