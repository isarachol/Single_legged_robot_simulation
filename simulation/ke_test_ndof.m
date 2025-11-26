function E = ke_test_ndof(x)

n = size(x, 1)/2;
E = 1/2*x(n+1:end)'*M_3dof(x(1:n))*x(n+1:end);

end