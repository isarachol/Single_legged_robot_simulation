function [results] = sim_static()

close all;
clc;

disp('Kinematics test')

% MACROS
zero_vec = [0;0;0];

% LINKS DEFINITION (1 joint per)
% 1) link 0 (worl to foot)
foot_l = 0.10;
foot_w = 0.06;
foot_h = 0.02;
foot_dim = [foot_l; foot_w; foot_h];
foot_w_to_com = [0; 0; foot_h/2]; % move up so the foot is on the ground

% 2) invisiblelink(invl) 1 (foot to j1)
invl_l = 0.02;
invl_w = 0.01;
invl_h = 0.01;
invl1_dim = [invl_l; invl_w; invl_h];
invl1_prev_to_com = [-foot_l/3; 0; foot_h];

% 3) invisiblelink(invl) 2 (j1 to j2)
invl2_dim = [invl_w; invl_l; invl_h];
invl2_prev_to_com = [-invl_w; 0; invl_w];

% 4) leg 1 (j2 to leg1)
leg1_l = 0.15;
leg1_w = 0.01;
leg1_h = 0.01;
leg1_dim = [leg1_l; leg1_w; leg1_h];
leg1_prev_to_com = [-leg1_l/2; 0; 0]; % p_ (position vector pointing from j2 to COM)

% 5) invisiblelink(invl) 3 (leg1 to j3)
invl3_dim = [invl_w; invl_l; invl_h];
invl3_prev_to_com = [-leg1_l/2; 0; 0];

% 6) leg 2 (j3 to leg2)
leg2_l = 0.15;
leg2_w = 0.01;
leg2_h = 0.01;
leg2_dim = [leg2_l; leg2_w; leg2_h];
leg2_prev_to_com = [-leg2_l/2; 0; 0];

% 7) invisiblelink(invl) 4 (leg2 to j4)
invl4_dim = [invl_w; invl_l; invl_h];
invl4_prev_to_com = [-leg2_l/2; 0; 0];

% 8) invisiblelink(invl) 5 (j4 to j5)
invl5_dim = [invl_l; invl_w; invl_h];
invl5_prev_to_com = [-invl_w; 0; 0];

% 9) invisiblelink(invl) 6 (j5 to j6)
invl6_dim = [invl_w; invl_h; invl_l];
invl6_prev_to_com = [0; 0; invl_h/2+invl_l/2];

% 10) head (j6 to head)
head_l = 0.10;
head_w = 0.05;
head_h = 0.05;
head_dim = [head_l; head_w; head_h];
head_prev_to_com = [head_l/3; 0; head_h/2+invl_h/2];

% Define neutral
q1_ = 0;
q2_ = 0.1;
q3_ = pi-2*q2_;
q4_ = q2_-pi/2;
q5_ = 0;
q6_ = 0;
q_ = [q1_;q2_;q3_;q4_;q5_; q6_];

% Define configuration
q1 = 0;
q2 = 0.2;
q3 = pi-2*q2;
q4 = q2-pi/2;
q5 = 0;
q6 = 0;
q = [q1; q2; q3; q4; q5; q6];

% Plot 3D
show_plot = 1;
if(show_plot)
    figure()
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    title("Pixaar simulation: static");
    axis equal;
    view (135,45);
    % cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    % world frame
    quiver3(0,0,0,1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);
    
    vecsize_fix_joint = 0.01;
    % plot foot
    T_foot = Tx(0, foot_w_to_com); % fixed joint w to foot
    handle_foot = plot_box_3d(T_foot, foot_dim);

    % invl1
    T_invl1 = T_foot * Tx(0, invl1_prev_to_com); % fixed joint foot to j1
    handle_invl1 = plot_box_3d(T_invl1, invl1_dim);

    % invl2
    T_invl2 = T_invl1 * Tx(q(1), invl2_prev_to_com); % revolute j1 to j2
    handle_invl2 = plot_box_3d(T_invl2, invl2_dim);

    % leg1
    T_leg1 = T_invl2 * Ty(q(2), leg1_prev_to_com); % revolute j2 to leg1
    handle_leg1 = plot_box_3d(T_leg1, leg1_dim);

    % invl3
    T_invl3 = T_leg1 * Ty(0, invl3_prev_to_com); % fixed leg1 to j3
    handle_invl3 = plot_box_3d(T_invl3, invl3_dim);

    % leg2
    T_leg2 = T_invl3 * Ty(q(3), leg2_prev_to_com); % revolute j3 to leg2
    handle_leg2 = plot_box_3d(T_leg2, leg2_dim);

    % invl4
    T_invl4 = T_leg2 * Tx(0, invl4_prev_to_com); % fixed leg2 to j4
    handle_invl4 = plot_box_3d(T_invl4, invl4_dim);

    % invl5
    T_invl5 = T_invl4 * Ty(q(4), invl5_prev_to_com) * Ty(-pi/2, zero_vec); % revolute j4 to j5 flip frame to match foot
    handle_invl5 = plot_box_3d(T_invl5, invl5_dim);
    
    % invl6
    T_invl6 = T_invl5 * Tx(q(5), invl6_prev_to_com); % fixed j5 to j6
    handle_invl6 = plot_box_3d(T_invl6, invl6_dim);

    % head
    T_head = T_invl6 * Tz(q(6), head_prev_to_com); % revolute j6 to head
    handle_head = plot_box_3d(T_head, head_dim);
end

end