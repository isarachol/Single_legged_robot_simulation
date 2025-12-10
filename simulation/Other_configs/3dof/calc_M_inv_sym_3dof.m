function [Minv_sym] = calc_M_inv_sym_3dof()
% Generate a function to calculate M inverse quickly

clc
clear

% File names
Minv_sym_filename = "Minv_3dof";
M_sym_filename = "M_3dof";
dUgdq_filename = "dUgdq_3dof";

% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

% define variables
syms q1 q2 q3 'real'
q = [0; 0; q1; q2; 0; q3; 0; 0; 0; 0];
T = solve_kinematics(q, joint_to_com, rot);

% Currently, T{2} represents to global origin for the robot (no jumping)
%% q1
% Jp^3_3
z_2 = T{2}(1:3, 1); % abs rotation axis (relative is Ex)
p_2 = T{2}(1:3, 4);
r_3 = T{3}(1:3, 4);
r_3_2 = r_3 - p_2;
Jp3_3 = skw(z_2) * r_3_2; % checked

%% q2
% Jp^4_3
r_4 = T{4}(1:3, 4);
r_4_2 = r_4 - p_2;
Jp4_3 = skw(z_2) * r_4_2;

% Jp^4_4
z_3 = T{3}(1:3, 1); % relative is Ex
p_3 = T{3}(1:3, 4);
r_4_3 = r_4 - p_3; % norm correct
Jp4_4 = skw(z_3) * r_4_3;

%% q3
% Jp^6_3
r_6 = T{6}(1:3, 4);
r_6_2 = r_6 - p_2;
Jp6_3 = skw(z_2) * r_6_2;

% Jp^6_4
r_6_3 = r_6 - p_3;
Jp6_4 = skw(z_3) * r_6_3;

% Jp^6_6 --> 5 is a joint, assume no mass for now
z_5 = T{5}(1:3,1); % local Ex
p_5 = T{5}(1:3, 4); % position of com of joint 5 (p_4 is com of leg1 which is fixed)
r_6_6 = r_6 - p_5;
Jp6_6 = skw(z_5) * r_6_6;

%% combine Jp, Jo
% Jp3 = Jp3_3; --> tested 1 dof
% Jo3 = z_2;
Jp3 = [Jp3_3, zero_vec, zero_vec];
Jp4 = [Jp4_3, Jp4_4, zero_vec];
Jp6 = [Jp6_3, Jp6_4, Jp6_6];

Jo3 = [z_2, zero_vec, zero_vec];
Jo4 = [z_2, z_3, zero_vec];
Jo6 = [z_2, z_3, z_5];

%% Calc Ug
g = [0; 0; 9.81];

% rearrange for dUgdq
Jp_m3 = [Jp3_3, Jp4_3, Jp6_3];
Jp_m4 = [zero_vec, Jp4_4, Jp6_4];
Jp_m6 = [zero_vec, zero_vec, Jp6_6];

dUgdq1 = m(3)*sum(g' * Jp_m3)
dUgdq2 = m(4)*sum(g' * Jp_m4)
dUgdq3 = m(6)*sum(g' * Jp_m6)

%% Calc Mass Matrix
I_3 = diag(I(:,3));
I_4 = diag(I(:,4));
I_6 = diag(I(:,6));

R_3 = T{3}(1:3, 1:3);
R_4 = T{4}(1:3, 1:3);
R_6 = T{6}(1:3, 1:3);

disp('Calculating the inverse of this matrix:')
% i = 1:1
M_sym = m(3) * (Jp3' * Jp3) + Jo3'*(R_3*I_3*R_3')*Jo3; % should be rock solid
M_sym = M_sym + m(4) * (Jp4' * Jp4) + Jo4'*(R_4*I_4*R_4')*Jo4;
M_sym = M_sym + m(6) * (Jp6' * Jp6) + Jo6'*(R_6*I_6*R_6')*Jo6
Minv_sym = inv(M_sym);

% turn q's into vector
q = sym("q", [size(M_sym,1) 1]);

% save dUgdq function
matlabFunction([dUgdq1; dUgdq2; dUgdq3], "File", dUgdq_filename, 'vars', {q});

% this is how you swap variable names in a matlab symbolic expression
Minv_sym = subs(Minv_sym, q1, q(1));
Minv_sym = subs(Minv_sym, q2, q(2));
Minv_sym = subs(Minv_sym, q3, q(3));

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(M_sym, "File", M_sym_filename, 'vars', {q});
disp(strcat("Saved file to ", M_sym_filename));
matlabFunction(Minv_sym, "File", Minv_sym_filename, 'vars', {q});
disp(strcat("Saved file to ", Minv_sym_filename));
end