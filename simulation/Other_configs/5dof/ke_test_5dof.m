function E = ke_test_5dof(x)

assert(size(x,1) == 10, "Wrong input size (10x1)");

n = size(x, 1)/2;
E = 1/2*x(n+1:end)'*M_5dof(x(1:n))*x(n+1:end);

end