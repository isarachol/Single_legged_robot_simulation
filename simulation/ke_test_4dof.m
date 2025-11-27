function E = ke_test_4dof(x)

assert(size(x,1) == 8, "Wrong input size (8x1)");

n = size(x, 1)/2;
E = 1/2*x(n+1:end)'*M_4dof(x(1:n))*x(n+1:end);

end