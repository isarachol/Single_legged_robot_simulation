function [Jp_1_sym] = calc_Jp_1()

syms q1 'real'

% File names
Jp1_sym_filename = "Jp1_2dof";

% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

% T_foot = Tx(0, foot_w_to_com); % fixed joint w to foot
% T_invl1 = T_foot * Tx(0, invl1_prev_to_com); % fixed joint foot to j1
% T_invl2 = T_invl1 * Tx(q1, invl2_prev_to_com); % revolute j1 to j2

% define variables
syms q1 q2 q3 q4 q5 q6 'real'
% q1 = 0;
q = [0; 0; q1; q2; 0; q3; 0; q4; q5; q6];
% q = [0; 0; 0; q2; 0; q3; 0; q4; q5; q6];
T = solve_kinematics(q, joint_to_com, rot);

% Jp^1_1
r_ax_0 = T{3}(1:3, 1); % abs rotation axis (relative is Ex), also Jo^1_1 = r_ax_0 (z_{i-1} in handout)
r_1 = T{3}(1:3, 4);
p_0 = T{2}(1:3, 4);
r_1_1 = r_1 - p_0;
Jp1_1 = skw(r_ax_0) * r_1_1; % z0 x (r1 - p0)

Jp_1_sym = [Jp1_1, zeros_vec, zero_vec, zero_vec, zero_vec, zero_vec];

end