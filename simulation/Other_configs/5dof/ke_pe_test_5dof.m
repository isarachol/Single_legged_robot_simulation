function results = ke_pe_test_5dof(x, grav)

arguments
    x
    grav {mustBeNumeric} = 1 % not used
end

assert(size(x,1) == 10, "Wrong input size (10x1)");
n = size(x, 1)/2;

KE = 1/2*x(n+1:end)'*M_5dof(x(1:n))*x(n+1:end);
if grav == 0
    PE = zeros(size(KE));
else
    PE = Ug_5dof(x(1:n));
end

results = [KE; PE];

end