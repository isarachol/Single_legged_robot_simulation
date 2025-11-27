function [bolddotx] = solve_2dof(x)
    tol = 10^(6);
    bolddotx = zeros(size(x));
    bolddotx(1:2) = x(3:4);
    % bolddotx(1) = x(2);

    M_inv = Minv_2dof(x(1:2));
    C = C_2dof(x(1:2), x(3:4))*x(3:4);
    % C = C_2dof(x(1), x(2))*x(2);

    Q = [0;0]; % no external forces/toirques
    bolddotx(3:4) = M_inv*(Q-C);
    % bolddotx(2) = M_inv*Q;

    bolddotx = numerical_check(bolddotx, tol);
end