function C_sym = calc_coriolis_sym_5dof()

clc
clear

C_filename = 'C_5dof';

syms q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 'real'
q = [q1; q2; q3; q4; q5];
dq = [dq1; dq2; dq3; dq4; dq5];
M = M_5dof(q);
C_sym = sym('c', size(M,1)); % to preallocate space
C_c = sym('c', size(M)); % to subtract out later

disp("Partial differentiating the mass matrix");
dM_dq1 = diff(M, q1);
dM_dq2 = diff(M, q2);
dM_dq3 = diff(M, q3);
dM_dq4 = diff(M, q4);
dM_dq5 = diff(M, q5);
dM_dqij = {dM_dq1, dM_dq2, dM_dq3, dM_dq4, dM_dq5};
N = size(M,1);

disp("Calculating the coriolis matrix");
for i=1:N
    for j=1:N
        for k=1:N
            C_sym(i,j) = C_sym(i,j) + 1/2*(dM_dqij{k}(i,j) + dM_dqij{j}(i,k) - dM_dqij{i}(j,k)) * dq(k);
        end
    end
end
C_sym = C_sym - C_c;

% turn q's into vector
q = sym("q", [size(C_sym,1) 1]);
dq = sym("dq", [size(C_sym,1) 1]);

C_sym = subs(C_sym, q1, q(1));
C_sym = subs(C_sym, q2, q(2));
C_sym = subs(C_sym, q3, q(3));
C_sym = subs(C_sym, q4, q(4));
C_sym = subs(C_sym, q5, q(5));
C_sym = subs(C_sym, dq1, dq(1));
C_sym = subs(C_sym, dq2, dq(2));
C_sym = subs(C_sym, dq3, dq(3));
C_sym = subs(C_sym, dq4, dq(4));
C_sym = subs(C_sym, dq5, dq(5));

C_sym

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(C_sym, "File", C_filename, 'vars', {q, dq});
disp(strcat("Saved file to ", C_filename));

end