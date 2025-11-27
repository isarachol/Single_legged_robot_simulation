function E = ke_pe_test_5dof(x)

assert(size(x,1) == 10, "Wrong input size (10x1)");

n = size(x, 1)/2;
KE = 1/2*x(n+1:end)'*M_5dof(x(1:n))*x(n+1:end);

desc_filename = 'robot_desc_v2.mat';
load(desc_filename);


PE = m(3) * 9.81 * 

end