function [Minv_sym] = calc_M_inv_sym()
%calc_M_inv_sym Use MATLAB's symbolic solver to calculate the inverse of a
%matrix, presumably the mass matrix
%   input = None, To Do allow user to pass in a M_sym
%   output = inverse of the matrix hard-coded into this function
%   state change = save the calculation to a function .m file for easier
%   use later

% The m-file name to save our M_inv function once calculated
Minv_sym_filename = "Minv_twolinkrobot";

% The variables in the mass matrix need to matlab "sym"s
syms a1 a2 ell1 ell2 q1 q2 m1 m2 Iz1 Iz2 'real'

% Here:
%   a1, a2 are lengths of bars
%   ell1, ell2 are the distance along bar to the center of mass
%   Iz1, Iz2 are the scalar mass moments of inertia around the
%       z-axis, since planar arm is only moving in 2D.

% The mass matrix in symbolic form: 2x2.

b11 = Iz1 + m1*ell1^2 + m2*(a1^2 + ell2^2 + 2*a1*ell2*cos(q2)); % Isara
b12 = Iz2 + m2*(ell2^2 + a1*ell2*cos(q2)); % Isara
b22 = Iz2 + m2*ell2^2; % Isara
M = [b11,   b12;
     b12,   b22];
    
disp('Calculating the inverse of this matrix:')
M

Minv_sym = inv(M)

% One way to force a certain number of arguments is to use a symbolic
% vector and then MATLAB will index into it.
% Let's use our full set of the non-constrained q^i's. We know there are
% the same number of these as our M matrix, so:
q = sym("q", [size(M,1) 1]);

% this is how you swap variable names in a matlab symbolic expression
Minv_sym = subs(Minv_sym, q1, q(1));
Minv_sym = subs(Minv_sym, q2, q(2));

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(Minv_sym, "File", Minv_sym_filename, 'vars', {q, m1, m2, Iz1, Iz2, a1, ell1, ell2});
disp(strcat("Saved file to ", Minv_sym_filename));

end