function E = ke_test_6dof(x)

assert(size(x,1) == 12, "Wrong input size (12x1)");

n = size(x, 1)/2;
E = 1/2*x(n+1:end)'*M_6dof(x(1:n))*x(n+1:end);

end