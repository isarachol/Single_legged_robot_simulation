function [Minv_sym] = calc_M_inv_sym_6dof()
% Generate a function to calculate M inverse quickly
% Assume 1 body per DOF

clc
clear

% File names
Minv_sym_filename = "Minv_6dof";
M_sym_filename = "M_6dof";

% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

% define variables
syms q1 q2 q3 q4 q5 q6 'real'
q = [0; 0; q1; q2; 0; q3; 0; q4; q5; q6];
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

%% q4
% Jp^8_3
r_8 = T{8}(1:3, 4);
r_8_2 = r_8 - p_2;
Jp8_3 = skw(z_2) * r_8_2;

% Jp^8_4
r_8_3 = r_8 - p_3;
Jp8_4 = skw(z_3) * r_8_3;

% Jp^8_6 --> 5 is a joint, assume no mass for now
r_8_6 = r_8 - p_5;
Jp8_6 = skw(z_5) * r_8_6;

% Jp^8_8
z_7 = T{7}(1:3,1); % local Ex
p_7 = T{7}(1:3, 4); % position of com of joint 5 (p_4 is com of leg1 which is fixed)
r_8_8 = r_8 - p_7;
Jp8_8 = skw(z_7) * r_8_8;

%% q5
% Jp^9_3
r_9 = T{9}(1:3, 4);
r_9_2 = r_9 - p_2;
Jp9_3 = skw(z_2) * r_9_2;

% Jp^8_4
r_9_3 = r_9 - p_3;
Jp9_4 = skw(z_3) * r_9_3;

% Jp^8_6 --> 5 is a joint, assume no mass for now
r_9_6 = r_9 - p_5;
Jp9_6 = skw(z_5) * r_9_6;

% Jp^9_8
r_9_8 = r_9 - p_7;
Jp9_8 = skw(z_7) * r_9_8;

% Jp^9_9
z_8 = T{8}(1:3,1); % local Ex
p_8 = T{8}(1:3, 4); % position of com of joint 5 (p_4 is com of leg1 which is fixed)
r_9_9 = r_9 - p_8;
Jp9_9 = skw(z_8) * r_9_9;

%% q6
% Jp^10_3
r_10 = T{10}(1:3, 4);
r_10_2 = r_10 - p_2;
Jp10_3 = skw(z_2) * r_10_2;

% Jp^10_4
r_10_3 = r_10 - p_3;
Jp10_4 = skw(z_3) * r_10_3;

% Jp^10_6 --> 5 is a joint, assume no mass for now
r_10_6 = r_10 - p_5;
Jp10_6 = skw(z_5) * r_10_6;

% Jp^10_8
r_10_8 = r_10 - p_7;
Jp10_8 = skw(z_7) * r_10_8;

% Jp^10_9
r_10_9 = r_10 - p_8;
Jp10_9 = skw(z_8) * r_10_9;

% Jp^10_10
z_9 = T{9}(1:3,1); % local Ex
p_9 = T{9}(1:3, 4); % position of com of joint 5 (p_4 is com of leg1 which is fixed)
r_10_10 = r_10 - p_9;
Jp10_10 = skw(z_9) * r_10_10;

%% combine Jp, Jo
% Jp3 = Jp3_3; --> tested 1 dof
% Jo3 = z_2;
Jp3 = [Jp3_3, zero_vec, zero_vec, zero_vec, zero_vec, zero_vec];
Jp4 = [Jp4_3, Jp4_4, zero_vec, zero_vec, zero_vec, zero_vec];
Jp6 = [Jp6_3, Jp6_4, Jp6_6, zero_vec, zero_vec, zero_vec];
Jp8 = [Jp8_3, Jp8_4, Jp8_6, Jp8_8, zero_vec, zero_vec];
Jp9 = [Jp9_3, Jp9_4, Jp9_6, Jp9_8, Jp9_9, zero_vec];
Jp10 = [Jp10_3, Jp10_4, Jp10_6, Jp10_8, Jp10_9, Jp10_10];

Jo3 = [z_2, zero_vec, zero_vec, zero_vec, zero_vec, zero_vec];
Jo4 = [z_2, z_3, zero_vec, zero_vec, zero_vec, zero_vec];
Jo6 = [z_2, z_3, z_5, zero_vec, zero_vec, zero_vec];
Jo8 = [z_2, z_3, z_5, z_7, zero_vec, zero_vec];
Jo9 = [z_2, z_3, z_5, z_7, z_8, zero_vec];
Jo10 = [z_2, z_3, z_5, z_7, z_8, z_9];

I_3 = diag(I(:,3));
I_4 = diag(I(:,4));
I_6 = diag(I(:,6));
I_8 = diag(I(:,8));
I_9 = diag(I(:,9));
I_10 = diag(I(:,10));

R_3 = T{3}(1:3, 1:3);
R_4 = T{4}(1:3, 1:3);
R_6 = T{6}(1:3, 1:3);
R_8 = T{8}(1:3, 1:3);
R_9 = T{9}(1:3, 1:3);
R_10 = T{10}(1:3, 1:3);

disp('Calculating the inverse of this matrix:')
M_sym = m(3) * (Jp3' * Jp3) + Jo3'*(R_3*I_3*R_3')*Jo3; % should be rock solid
M_sym = M_sym + m(4) * (Jp4' * Jp4) + Jo4'*(R_4*I_4*R_4')*Jo4;
M_sym = M_sym + m(6) * (Jp6' * Jp6) + Jo6'*(R_6*I_6*R_6')*Jo6;
M_sym = M_sym + m(8) * (Jp8' * Jp8) + Jo8'*(R_8*I_8*R_8')*Jo8;
M_sym = M_sym + m(9) * (Jp9' * Jp9) + Jo9'*(R_9*I_9*R_9')*Jo9;
M_sym = M_sym + m(10) * (Jp10' * Jp10) + Jo10'*(R_10*I_10*R_10')*Jo10
Minv_sym = inv(M_sym);

% turn q's into vector
q = sym("q", [size(M_sym,1) 1]);

% this is how you swap variable names in a matlab symbolic expression
Minv_sym = subs(Minv_sym, q1, q(1));
Minv_sym = subs(Minv_sym, q2, q(2));
Minv_sym = subs(Minv_sym, q3, q(3));
Minv_sym = subs(Minv_sym, q4, q(4));
Minv_sym = subs(Minv_sym, q5, q(5));
Minv_sym = subs(Minv_sym, q6, q(6));

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(M_sym, "File", M_sym_filename, 'vars', {q});
disp(strcat("Saved file to ", M_sym_filename));
matlabFunction(Minv_sym, "File", Minv_sym_filename, 'vars', {q});
disp(strcat("Saved file to ", Minv_sym_filename));
end