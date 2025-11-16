function make_robot_description(desc_filename)

arguments
    desc_filename string = 'robot_desc.mat'
end

disp("Defining Robot");
% MACROS
zero_vec = [0;0;0];

% LINKS DEFINITION (1 joint per)
% 1) foot (world to foot) --> essentially connects relative to global pos
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

save(desc_filename);

end