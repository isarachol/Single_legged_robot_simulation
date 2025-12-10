function results = ke_pe_test_4dof(x, grav)

assert(size(x,1) == 8, "Wrong input size (10x1)");
n = size(x, 1)/2;

KE = 1/2*x(n+1:end)'*M_4dof(x(1:n))*x(n+1:end);
if grav == 0
    PE = zeros(size(KE));
else
    PE = Ug_4dof(x(1:n));
end

results = [KE; PE];

end