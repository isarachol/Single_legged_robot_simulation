function [joint_pos] = calc_kinematics_3body()
% calc_kinematics Use MATLAB's symbolic solver to calculate the homogeneous
% transformation matrices
% assume: 
%   1) joint has no mass 
%   2) joint takes up no space

% The m-file name to save our M_inv function once calculated
filename = "kinematics3";

% The variables in the mass matrix need to matlab "sym"s
syms a1 [3 1] 'real' % distance from j_i to j_i+1 (j6 = fixed joint for head)
syms a2 [3 1] 'real'
syms a3 [3 1] 'real'
% syms ell [3 1] 'real' % distance from j_i to COM of ell_i
syms q [5 1] 'real'
% syms m [3 1] 'real'
% syms I [3 3] 'real' % Ix, Iy, Iz
% ex. l0y = l01y = l1_world_y - l0_world_y

% Here:
%   a1, a2 are lengths of bars
%   ell1, ell2 are the distance along bar to the center of mass
%   Iz1, Iz2 are the scalar mass moments of inertia around the
%       z-axis, since planar arm is only moving in 2D.

A_j1_j2 = [Ry(q1), [0;0;0]];
A_j1_j2 = [A_j1_j2; 0 0 0 1];

A_j2_j3 = [Rx(q2), Rx(q2) * [a11;a21;a31]];
A_j2_j3 = [A_j2_j3; 0 0 0 1];

A_j3_j4 = [Rx(q3), Rx(q3) * [a12;a22;a32]];
A_j3_j4 = [A_j3_j4; 0 0 0 1];

A_j4_j5 = [Rx(q4), [0;0;0]];
A_j4_j5 = [A_j4_j5; 0 0 0 1];

A_j5_j6 = [Rz(q5), Rz(q5) * [a13;a23;a33]];
A_j5_j6 = [A_j5_j6; 0 0 0 1];

vec_4to3 = [1 0 0 0; 0 1 0 0; 0 0 1 0];
pnaught = [0; 0; 0; 1];

disp('Calculating the robot kinematics (joint positions):')
x1 = [0;0;0];
x2 = vec_4to3 * A_j1_j2 * pnaught;
x3 = vec_4to3 * A_j1_j2 * A_j2_j3 * pnaught;
x4 = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * pnaught;
x5 = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * A_j4_j5 * pnaught;
x6 = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * A_j4_j5 * A_j5_j6 * pnaught;

x = [x1, x2, x3, x4, x5, x6];
% x = zeros(3,6);
% x(:,2) = vec_4to3 * A_j1_j2 * pnaught;
% x(:,3) = vec_4to3 * A_j1_j2 * A_j2_j3 * pnaught;
% x(:,4) = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * pnaught;
% x(:,5) = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * A_j4_j5 * pnaught;
% x(:,6) = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * A_j4_j5 * A_j5_j6 * pnaught;

joint_pos = x

% One way to force a certain number of arguments is to use a symbolic
% vector and then MATLAB will index into it.
% Let's use our full set of the non-constrained q^i's. We know there are
% the same number of these as our M matrix, so:
q = sym("q", [5 1]);
% a = sym("q", [5 1]);
% a = sym("ell", [3 3]);

% this is how you swap variable names in a matlab symbolic expression
joint_pos = subs(joint_pos, q1, q(1));
joint_pos = subs(joint_pos, q2, q(2));
joint_pos = subs(joint_pos, q3, q(3));
joint_pos = subs(joint_pos, q4, q(4));
joint_pos = subs(joint_pos, q5, q(5));
% 
% joint_pos = subs(joint_pos, a1, a(1));
% joint_pos = subs(joint_pos, a2, a(2));
% joint_pos = subs(joint_pos, a3, a(3));
% 
% joint_pos = subs(joint_pos, a1_1, a(1,1));
% joint_pos = subs(joint_pos, a2_1, a(2,1));
% joint_pos = subs(joint_pos, a3_1, a(3,1));
% joint_pos = subs(joint_pos, a1_2, a(1,2));
% joint_pos = subs(joint_pos, a2_2, a(2,2));
% joint_pos = subs(joint_pos, a3_2, a(3,2));
% joint_pos = subs(joint_pos, a1_3, a(1,3));
% joint_pos = subs(joint_pos, a2_3, a(2,3));
% joint_pos = subs(joint_pos, a3_3, a(3,3));

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(joint_pos, "File", filename, 'vars', {q, a1, a2, a3});
disp(strcat("Saved file to ", filename));

end