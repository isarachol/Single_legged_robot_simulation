function [Minv_sym] = calc_M_inv_sym_2dof()
% Generate a function to calculate M inverse quickly

clc
clear

% File names
Minv_sym_filename = "Minv_2dof";
M_sym_filename = "M_2dof";

% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

% define variables
syms q1 q2 'real'
% q1 = 0;
q = [q1; 0; q2];
T = solve_kinematics(q, joint_to_com, rot);

% Currently, T{2} represents to global origin for the robot (no jumping)
%% q1
% Jp^1_1
z_0 = [1;0;0]; % abs rotation axis (relative is Ex)
p_0 = zero_vec;
r_1 = T{1}(1:3, 4);
r_1_0 = r_1 - p_0;
Jp1_1 = skw(z_0) * r_1_0;

%% q2
% Jp^2_1
r_2 = T{3}(1:3, 4);
r_2_0 = r_2 - p_0;
Jp2_1 = skw(z_0) * r_2_0;

% Jp^2_2
z_1 = T{1}(1:3, 1); % relative is Ex
p_1 = T{2}(1:3, 4); % edit --> position from joint, not from COM of link 1
r_2_1 = r_2 - p_1;
Jp2_2 = skw(z_1) * r_2_1;

%% combine Jp, Jo
% Jp3 = Jp3_3;
% Jo3 = z_2;
Jp1 = [Jp1_1, zero_vec];
Jo1 = [z_0, zero_vec];
Jp2 = [Jp2_1, Jp2_2];
Jo2 = [z_0, z_1];

I_1 = diag(I(:,1));
I_2 = diag(I(:,3));
R_1 = T{1}(1:3, 1:3);
R_2 = T{3}(1:3, 1:3);

disp('Calculating the inverse of this matrix:')
% i = 1:1
M_sym = m(1) * (Jp1' * Jp1) + Jo1'*(R_1*I_1*R_1')*Jo1; % should be rock solid
M_sym = M_sym + m(3) * (Jp2' * Jp2) + Jo2'*(R_2*I_2*R_2')*Jo2
Minv_sym = inv(M_sym);

% turn q's into vector
q = sym("q", [size(M_sym,1) 1]);

% this is how you swap variable names in a matlab symbolic expression
Minv_sym = subs(Minv_sym, q1, q(1));
Minv_sym = subs(Minv_sym, q2, q(2));

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(M_sym, "File", M_sym_filename, 'vars', {q});
disp(strcat("Saved file to ", M_sym_filename));
matlabFunction(Minv_sym, "File", Minv_sym_filename, 'vars', {q});
disp(strcat("Saved file to ", Minv_sym_filename));
end