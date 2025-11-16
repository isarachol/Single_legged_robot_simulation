function [Jp_1_sym] = calc_Jp_1()

syms q1 'real'

% load robot description (geometry)
desc_filename = 'robot_desc.mat';
if (isfile(desc_filename))
    load(desc_filename);
else
    make_robot_description(desc_filename);
    load(desc_filename);
end

T_foot = Tx(0, foot_w_to_com); % fixed joint w to foot
T_invl1 = T_foot * Tx(0, invl1_prev_to_com); % fixed joint foot to j1
T_invl2 = T_invl1 * Tx(q1, invl2_prev_to_com); % revolute j1 to j2

% Jp^1_1
r_ax_0 = T_invl2(1:3, 1); % abs rotation axis (relative is Ex), also Jo^1_1 = r_ax_0 (z_{i-1} in handout)
r_1 = T_invl2(1:3, 4);
p_0 = T_invl1(1:3, 4);
r_1_1 = r_1 - p_0;
Jp1_1 = skw(r_ax_0) * r_1_1; % z0 x (r1 - p0)

Jp_1_sym = [Jp1_1, zeros_vec, zero_vec, zero_vec, zero_vec, zero_vec];

end