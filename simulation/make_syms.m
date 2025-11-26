function results = make_syms(n)
    if n==2
        results.Minv_sym = calc_M_inv_sym_2dof();
        results.C_sym = calc_coriolis_sym_2dof();
    elseif n==3
        results.Minv_sym = calc_M_inv_sym_3dof();
        results.C_sym = calc_coriolis_sym_3dof();
    elseif n==6
        results.Minv_sym = calc_M_inv_sym_6dof();
        results.C_sym = calc_coriolis_sym_6dof();
    else
        error("Error, have not done %.2f DOF yet!", n);
    end
end