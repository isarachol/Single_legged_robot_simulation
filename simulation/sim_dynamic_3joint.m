% function [results] = sim_dynamic_3joint()

close all;
clc;

disp('Dynamic test (3 joints)')

%% Constants
% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

%% Make M, M_inv, C functions
make_syms();

%% Setup
% time
tmax = 5;
dt = 0.0001;
t_span = 0:dt:tmax-dt;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% initial conditions
q0 = [0; 0]; %; 2; 0.1; 0; 0];
dq0 = [0.5; 0]; %; 0.5; 0.1; 0; 0];
% q0 = 0;
% dq0 = 3;
x0 = [q0;dq0];

%% Solve for trajectory
x_traj = zeros(size(x0,1),n);
KE = zeros(n, 1);
x_traj(:,1) = x0;
KE(1) = ke_test_2dof(x_traj(:,1));

for i=2:n
    bolddotx_t = solve_2dof(x_traj(:,i-1));
    x_traj(:,i) = x_traj(:,i-1) + bolddotx_t * dt; % forward euler
    KE(i) = ke_test_2dof(x_traj(:,i));
end

%% Conservation of energy test (KE only)
figure()
plot(t_span, KE);
xlabel("Time (s)");
ylabel("KE (J)");
title("KE vs time");

figure()
plot(t_span, x_traj(4,:));
xlabel("Time (s)");
ylabel("dq2 (rad/s)");
title("dq2 vs time");

figure()
plot(x_traj(2,:), x_traj(4,:));
xlabel("q2 (rad)");
ylabel("dq2 (rad/s)");
title("dq2 vs q2");

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
N=4;
if(show_plot)
    speedup = 100;
    n_speedup = n/speedup;

    figure()
    grid on;
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    title("Pixaar simulation: dynamic (kinetic energy only)");
    axis equal;
    xlim([-0.3, 0.1]);
    ylim([-0.1, 0.1]);
    zlim([-0.1, 0.3]);
    view (135,15);
    vecsize = 0.02;
    cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    % world frame
    quiver3(0,0,0,1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);

    q_aug = [0; 0; x_traj(1,1); x_traj(2,1); 0; q3; 0; q4; q5; q6];
    T = solve_kinematics(q_aug, joint_to_com, rot);
    handle = {};
    for i=1:N %numel(T)
        handle{i} = plot_box_3d(T{i}, dim(:,i), vecsize, 0);
    end
    drawnow;
    
    vecsize_fix_joint = 0.01;
    for ts=1:n_speedup
        t = ts * speedup;
        % delete previous plot
        for i=1:N
            for k=1:size(handle{i}, 1)
                delete(handle{i}{k});
            end
        end
        % display now config
        q_aug = [0; 0; x_traj(1,t); x_traj(2,t); 0; q3; 0; q4; q5; q6];
        T = solve_kinematics(q_aug, joint_to_com, rot);
        for i=1:N % takes 90% of execution time
            handle{i} = plot_box_3d(T{i}, dim(:,i), vecsize, 0);
        end
        drawnow limitrate;
    end
end

% end