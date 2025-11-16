function [results] = sim_dynamic_3joint()

close all;
clc;

disp('Dynamic test (3 joints)')

%% Constants
% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

%% Setup
% time
tmax = 5;
dt = 0.0001;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% initial conditions
q0 = [0; 0.5]; %; 2; 0.1; 0; 0];
dq0 = [0; -0.3]; %; 0.5; 0.1; 0; 0];
x0 = [q0;dq0];

%% Solve for trajectory
x_traj = zeros(size(x0,1),n);
x_traj(:,1) = x0;

for i=2:n
    bolddotx_t = solve_2dof(x_traj(:,i-1));
    x_traj(:,i) = x_traj(:,i-1) + bolddotx_t * dt; % forward euler
end

%% Simulation

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
if(show_plot)
    speedup = 1000;
    n_speedup = n/speedup;

    figure()
    grid on;
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    title("Pixaar simulation: static");
    axis equal;
    view (135,45);
    vecsize = 0.02;
    % cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    % world frame
    quiver3(0,0,0,1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(0,0,0,0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);

    q_aug = [0; 0; x_traj(1,1); x_traj(2,1); 0; q3; 0; q4; q5; q6];
    T = solve_kinematics(q_aug, joint_to_com, rot);
    handle = {};
    for i=1:numel(T)
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
        drawnow;
    end
end

end