function [bolddotx] = solve_2dof(x)
    tol = 10^(6);
    bolddotx = zeros(size(x));
    bolddotx(1:2) = x(3:4);

    M_inv = Minv_2dof(x);
    C = C_2dof(x(1:2), x(3:4))*x(3:4);

    Q = -C;
    bolddotx(3:4) = M_inv*Q;

    bolddotx = numerical_check(bolddotx, tol);
end