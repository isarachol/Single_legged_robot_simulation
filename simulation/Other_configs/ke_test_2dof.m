function E = ke_test_2dof(x)

assert(size(x,1) == 4, "Wrong input size (4x1)");

E = 1/2*x(3:4)'*M_2dof(x(1:2))*x(3:4);
% E = 1/2*x(2)'*M_2dof(x(1))*x(2);

end