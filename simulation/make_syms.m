function make_syms(n)
    if n==2
        calc_M_inv_sym_2dof();
        calc_coriolis_sym_2dof();
    elseif n==3
        calc_M_inv_sym_3dof();
        calc_coriolis_sym_3dof();
    elseif n==5
        calc_M_inv_sym_5dof();
        calc_coriolis_sym_5dof();
    else
        error("Error, have not done %.2f DOF yet!", n);
    end
end