function make_robot_description_v2(desc_filename)
% Assumes every joint is rotated about x axis (Rx)

arguments
    desc_filename string = 'robot_desc_v2.mat'
end

disp("Defining Robot");
disp_description();

% MACROS
zero_vec = [0;0;0];
N = 10;

%% initializes matrices
m = zeros(1,N);
I = zeros(3,N);
dim = zeros(3,N);
joint_to_com = zeros(3,N);
rot = zeros(3,N);

%% Geomatry
% LINKS DEFINITION (1 joint per)
% 1) foot (world to foot) --> essentially connects relative to global pos
foot_l = 0.10;
foot_w = 0.06;
foot_h = 0.02;
dim(:,1) = [foot_l; foot_w; foot_h];
joint_to_com(:,1) = [0; 0; foot_h/2]; % move up so the foot is on the ground

% 2) invisiblelink(invl) 1 (foot to j1)
invl_l = 0.02;
invl_w = 0.01;
invl_h = 0.01;
dim(:,2) = [invl_l; invl_w; invl_h];
joint_to_com(:,2) = [-foot_l/3; 0; foot_h];

% 3) invisiblelink(invl) 2 (j1 to j2)
dim(:,3) = dim(:,2);%[invl_w; invl_l; invl_h];
joint_to_com(:,3) = [-invl_w; 0; invl_w];
rot(3,3) = pi/2;

% 4) leg 1 (j2 to leg1)
leg1_l = 0.15;
leg1_w = 0.01;
leg1_h = 0.01;
dim(:,4) = [leg1_l; leg1_w; leg1_h];
joint_to_com(:,4) = [0; leg1_l/2; 0]; % p_ (position vector pointing from j2 to COM)
rot(3,4) = -pi/2;

% 5) invisiblelink(invl) 3 (leg1 to j3)
dim(:,5) = dim(:,2);%[invl_w; invl_l; invl_h];
joint_to_com(:,5) = [-leg1_l/2; 0; 0];
rot(3,5) = pi/2;

% 6) leg 2 (j3 to leg2)
leg2_l = 0.15;
leg2_w = 0.01;
leg2_h = 0.01;
dim(:,6) = [leg2_l; leg2_w; leg2_h];
joint_to_com(:,6) = [0; leg2_l/2; 0];
rot(3,6) = -pi/2;

% 7) invisiblelink(invl) 4 (leg2 to j4)
dim(:,7) = dim(:,2);%[invl_w; invl_l; invl_h];
joint_to_com(:,7) = [-leg2_l/2; 0; 0];
rot(3,7) = pi/2;

% 8) invisiblelink(invl) 5 (j4 to j5)
dim(:,8) = [invl_l; invl_w; invl_h];
joint_to_com(:,8) = [0; invl_w; 0];
rot(:,8) = [-pi/2;0;-pi/2];

% 9) invisiblelink(invl) 6 (j5 to j6)
dim(:,9) = dim(:,2);%[invl_w; invl_h; invl_l];
joint_to_com(:,9) = [0; 0; invl_h/2+invl_l/2];
rot(:,9) = [0;-pi/2;0];

% 10) head (j6 to head)
head_l = 0.10;
head_w = 0.05;
head_h = 0.05;
dim(:,10) = [head_l; head_w; head_h];
joint_to_com(:,10) = [head_h/2+invl_h/2; 0; -head_l/3];
rot(:,10) = [0;pi/2;0];

%% Weight
for i=1:10
    if i==1 || i==4 || i==6 || i==10
        m(i) = 0.1; % body part weight
    else
        m(i) = 0.05; % joint weight
    end
    I(:,i) = m(i)/12*[dim(2,i)^2 + dim(3,i)^2; dim(1,i)^2 + dim(3,i)^2; dim(1,i)^2 + dim(2,i)^2];
end

save(desc_filename);

end
