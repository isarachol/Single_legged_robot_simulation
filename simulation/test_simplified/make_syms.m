function results = make_syms()
    results.Minv_sym = calc_M_inv_sym_2dof();
    results.C_sym = calc_coriolis_sym_2dof();
end