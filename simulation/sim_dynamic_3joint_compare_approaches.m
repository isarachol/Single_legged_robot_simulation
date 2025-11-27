% function [results] = sim_dynamic_3joint()

close all;
clc;

dof = 3;
disp("Dynamic test (" + dof + " joints)")

%% Robot Descriptions
% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

%% Setup
% time
tmax = 20;
dt = 0.0005;
t_span = 0:dt:tmax-dt;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% initial conditions
q0 = [0; 0; pi*0.8]; %; 2; 0.1; 0; 0];
dq0 = [0; -1; 2]; %; 0.5; 0.1; 0; 0];
% q0 = 0;
% dq0 = 3;
x0 = [q0;dq0];

%% Solve for trajectory
x_traj1 = zeros(size(x0,1),n);
KE1 = zeros(n, 1);
x_traj1(:,1) = x0;
KE1(1) = ke_test_3dof(x_traj1(:,1));

x_traj2 = zeros(size(x0,1),n);
KE2 = zeros(n, 1);
x_traj2(:,1) = x0;
KE2(1) = ke_test_3dof(x_traj2(:,1));

% Using Minv
for i=2:n
    bolddotx_t = solve_3dof(x_traj1(:,i-1));
    x_traj1(:,i) = x_traj1(:,i-1) + bolddotx_t * dt; % forward euler
    KE1(i) = ke_test_3dof(x_traj1(:,i));
end

% Using M
for i=2:n
    bolddotx_t = solve_3dof_noMinv(x_traj2(:,i-1));
    x_traj2(:,i) = x_traj2(:,i-1) + bolddotx_t * dt; % forward euler
    KE2(i) = ke_test_3dof(x_traj2(:,i));
end

%% Conservation of energy test (KE only)
figure()
plot(t_span, KE1);
hold on;
plot(t_span, KE2);
xlabel("Time (s)");
ylabel("KE (J)");
title("KE vs time");
bound = 0.2;
ylim([max(KE1)*(1-bound), max(KE1)*(1+bound)]);

%% Simulation ====================== Rock solid =============================

% Define configuration
q1 = 0;
q2 = 0.2;
q3 = pi-2*q2;
q4 = q2-pi/2;
q5 = 0;
q6 = 0;
q = [q1; q2];
q_aug = [0; 0; q1; q2; 0; q3; 0; q4; q5; q6];

% Plot 3D
show_plot = 1;
N=10;
if(show_plot)
    speedup = 100;
    n_speedup = n/speedup;

    f = figure();
    grid on;
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    title("Pixaar simulation: dynamic (kinetic energy only)");
    axis equal;
    xlim([-0.4, 0.4]);
    ylim([-0.4, 0.4]);
    zlim([-0.4, 0.4]);
    f.Position = [250,70,1000,800];
    view(100,15); %(135,15);
    vecsize = 0.02;
    cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    % world frame
    quiver3(0,0,0,1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);

    q_aug = [0; 0; x_traj1(1,1); x_traj1(2,1); 0; x_traj1(3,1); 0; q4; q5; q6];
    T = solve_kinematics(q_aug, joint_to_com, rot);
    handle = {};
    handle = plot_single_legged_robot(T, dim, 0);
    drawnow;
    
    vecsize_fix_joint = 0.01;
    for ts=1:n_speedup
        t = ts * speedup;
        % delete previous plot
        for i=1:numel(handle)
            for k=1:numel(handle{i})
                delete(handle{i}{k});
            end
        end
        % display now config
        q_aug = [0; 0; x_traj1(1,t); x_traj1(2,t); 0; x_traj1(3,t); 0; q4; q5; q6];
        T = solve_kinematics(q_aug, joint_to_com, rot);
        handle = plot_single_legged_robot(T, dim, 0);
        drawnow limitrate;
    end
end

if(show_plot)
    speedup = 100;
    n_speedup = n/speedup;

    f = figure();
    grid on;
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    title("Pixaar simulation: dynamic no Minv (kinetic energy only)");
    axis equal;
    xlim([-0.4, 0.4]);
    ylim([-0.4, 0.4]);
    zlim([-0.4, 0.4]);
    f.Position = [250,70,1000,800];
    view(100,15); %(135,15);
    vecsize = 0.02;
    cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    % world frame
    quiver3(0,0,0,1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);

    q_aug = [0; 0; x_traj1(1,1); x_traj1(2,1); 0; x_traj1(3,1); 0; q4; q5; q6];
    T = solve_kinematics(q_aug, joint_to_com, rot);
    handle = {};
    handle = plot_single_legged_robot(T, dim, 0);
    drawnow;
    
    vecsize_fix_joint = 0.01;
    for ts=1:n_speedup
        t = ts * speedup;
        % delete previous plot
        for i=1:numel(handle)
            for k=1:numel(handle{i})
                delete(handle{i}{k});
            end
        end
        % display now config
        q_aug = [0; 0; x_traj1(1,t); x_traj1(2,t); 0; x_traj1(3,t); 0; q4; q5; q6];
        T = solve_kinematics(q_aug, joint_to_com, rot);
        handle = plot_single_legged_robot(T, dim, 0);
        drawnow limitrate;
    end
end

% end