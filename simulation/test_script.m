function [results] = test_script()

close all;
clc;

disp('Kinematics test')

% Macros
zero_vec = [0;0;0];

% Constants
% link 0 (worl to foot)
foot_l = 0.10;
foot_w = 0.06;
foot_h = 0.02;
foot_dim = [foot_w; foot_l; foot_h];
foot_w_to_com = [0; 0; foot_h/2]; % move up so the foot is on the ground

% invisiblelink(invl) 1 (foot to j1)
invl_l = 0.02;
invl_w = 0.01;
invl_h = 0.01;
invl1_dim = [invl_w; invl_l; invl_h];
invl1_j_to_com = [0; foot_l/3; foot_h];

% invisiblelink(invl) 2 (j1 to j2)
invl2_dim = [invl_l; invl_w; invl_h];
invl2_j_to_com = [0; invl_w; invl_w];

% leg 1 (j2 to leg1)
leg1_l = 0.15;
leg1_w = 0.01;
leg1_h = 0.01;
leg1_dim = [leg1_w; leg1_l; leg1_h];
leg1_prev_to_com = [0; leg1_l/2; 0]; % p_ (position vector pointing from j2 to COM)
leg1_prev_to_next = [0; leg1_l; 0];

% invisiblelink(invl) 3 (leg1 to j3)
invl3_dim = [invl_l; invl_w; invl_h];
invl3_j_to_com = [0; leg1_l/2; 0];

% link 3 (j3 to j4)
leg2_l = 0.15;
leg2_w = 0.01;
leg2_h = 0.01;
leg2_dim = [leg2_w; leg2_l; leg2_h];
leg2_j_to_com = [0; leg2_l/2; 0];

% invisiblelink(invl) 4 (j4 to j5)

% link 5 (j5 to j6)
head_l = 0.10;
head_w = 0.05;
head_h = 0.05;
head_dim = [head_w; head_h; head_l];
head_j_to_com = [0; head_h/2; head_l/3];

% Define configuration
q1 = 0.1;
q2 = pi/2;
q3 = pi-2*q2;
q4 = 0;
q5 = 0;
q = [q1;q2;q3;q4;q5];

% Define kinematics
% a_0 = [0;0;0];
% a0 = a_0;
% a1 = a_0;
% a2 = [0; leg1_l; 0];
% a3 = [0; leg2_l; 0];
% a4 = a_0;
% a5 = [0; head_l*2/3; 0];
% a = [a0, a1, a2, a3, a4, a5];

% x = kinematics(q, a);
% x = kinematics3(q, a1, a2, a3);


% Plot 3D
show_plot = 1;
if(show_plot)
    figure()
    hold on
    xlabel("E1");
    ylabel("E2");
    zlabel("E3");
    axis equal;
    view (45,45);

    % world frame
    quiver3(0,0,0,1,0,0, 0.02, 'r', "MaxHeadSize", 100);
    quiver3(0,0,0,0,1,0, 0.02, 'g', "MaxHeadSize", 100);
    quiver3(0,0,0,0,0,1, 0.02, 'b', "MaxHeadSize", 100);

    % plot foot
    T_foot = Tx(0, foot_w_to_com);
    foot_R_shift = zero_vec;
    handle_foot = plot_rectbar_3d_robotjoints(T_foot, foot_dim); %, foot_R_shift);

    % invl1
    T_invl1 = T_foot * Ty(q(1), invl1_j_to_com);
    invl1_R_shift = zero_vec;
    handle_invl1 = plot_rectbar_3d_robotjoints(T_invl1, invl1_dim); %, invl1_R_shift);

    % invl2
    T_invl2 = T_invl1 * Tx(q(2), invl2_j_to_com);
    invl2_R_shift = zero_vec;
    handle_invl2 = plot_rectbar_3d_robotjoints(T_invl2, invl2_dim); %, invl2_R_shift);

    % leg1
    T_leg1 = T_invl2 * Tx(0, leg1_prev_to_com);
    leg1_R_shift = zero_vec;
    handle_leg1 = plot_rectbar_3d_robotjoints(T_leg1, leg1_dim); %, leg1_R_shift);

    % invl3
    T_invl3 = T_leg1 * Tx(0, invl3_j_to_com);
    handle_invl3 = plot_rectbar_3d_robotjoints(T_invl3, invl3_dim); %, invl2_R_shift);
end

end