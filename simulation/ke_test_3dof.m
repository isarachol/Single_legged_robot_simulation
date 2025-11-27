function E = ke_test_3dof(x)

assert(size(x,1) == 6, "Wrong input size (6x1)");

n = size(x, 1)/2;
E = 1/2*x(n+1:end)'*M_3dof(x(1:n))*x(n+1:end);

end