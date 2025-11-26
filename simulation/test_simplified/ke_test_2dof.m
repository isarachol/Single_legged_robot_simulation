function E = ke_test_2dof(x)

E = 1/2*x(3:4)'*M_2dof(x(1:2))*x(3:4);
% E = 1/2*x(2)'*M_2dof(x(1))*x(2);

end