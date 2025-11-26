function [bolddotx] = solve_ndof(x)
    tol = 10^(6);
    n = size(x,1)/2;
    bolddotx = zeros(size(x));
    bolddotx(1:n) = x(n+1:end);

    if n==2
        M_inv = Minv_2dof(x(1:2));
        C = C_2dof(x(1:2), x(3:4))*x(3:4);
    elseif n==3
        M_inv = Minv_3dof(x(1:n));
        C = C_3dof(x(1:n), x(n+1:end))*x(n+1:end);
    else
        error("Error, have not done %.2f DOF yet!", n);
    end

    Q = zeros(n,1); % no external forces/toirques
    bolddotx(n+1:end) = M_inv*(Q-C);

    bolddotx = numerical_check(bolddotx, tol);
end