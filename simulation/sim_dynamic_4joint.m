% function [results] = sim_dynamic_3joint()

close all;
clc;

dof = 4;
disp("Dynamic test (" + dof + " joints)")

%% Robot Descriptions
% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

%% Setup
% time
tmax = 10;
dt = 0.0001;
t_span = 0:dt:tmax-dt;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% initial conditions
q0 = [pi; 0; 0; 0]; % rad
dq0 = [0; 0; 0; 0]; % rad/s
x0 = [q0;dq0];

% physical constants
damp = 0.1; % N/(rad/s)
damp_mat = damp * ones([dof,1]);

%% Solve for trajectory
filename = "xtraj_E_4dof_grav.mat"; % "xtraj_KE_5dof.mat" "xtraj_KE_5dof_damp.mat" "xtraj_KE_5dof_dampgrav.mat"
if ~isfile(filename)
    if contains(filename, "grav")
        grav = 1;
    else
        grav = 0;
    end

    x_traj = zeros(size(x0,1),n);
    E = zeros(2, n);
    x_traj(:,1) = x0;
    E(:,1) = ke_pe_test_4dof(x_traj(:,1), grav);
    
    for i=2:n
        bolddotx_t = solve_4dof(x_traj(:,i-1), damp_mat, grav);
        x_traj(:,i) = x_traj(:,i-1) + bolddotx_t * dt; % forward euler
        E(:,i) =  ke_pe_test_4dof(x_traj(:,i), grav); % ke_test_5dof(x_traj(:,i));
    end

    % Shift PE so that it's referenced to 0
    PE_ref = min(E(2,:));
    E(2,:) = E(2,:) - PE_ref;
    
    save(filename, "x_traj", "E");
else
    load(filename);
end

%% Conservation of energy test (KE only)
figure()
plot(t_span, E(1,:), 'LineWidth', 2); % KE
hold on
plot(t_span, E(2,:), 'LineWidth', 2); % PE
plot(t_span, E(1,:) + E(2,:), 'LineWidth', 2); % E total
xlabel("Time (s)");
ylabel("E (J)");
% title("E vs time");
legend(["KE", "PE", "E_{tot}"]);
% bound = 0.2; % 0 - 0.5 or 0.6 ish if want buffer
% ylim([max(E(1,:))*(1-bound), max(E(1,:))*(1+bound)]);
set(gcf, 'Position', [100 100 800 200]);
fontname("Times New Roman");
fontsize(16, "points");

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
    view(135,15); %(135,15);
    vecsize = 0.02;
    cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    q_aug = [0; 0; x_traj(1,1); x_traj(2,1); 0; x_traj(3,1); 0; x_traj(4,1); q5; q6];
    T = solve_kinematics(q_aug, joint_to_com, rot);

    % world frame
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);

    handle = {};
    handle = plot_single_legged_robot(T, dim, 0);
    drawnow;
    
    vecsize_fix_joint = 0.01;
    time = 0;
    tic
    for ts=1:n_speedup
        t = ts * speedup;
        % delete previous plot
        for i=1:numel(handle)
            for k=1:numel(handle{i})
                delete(handle{i}{k});
            end
        end
        % display now config
        q_aug = [0; 0; x_traj(1,t); x_traj(2,t); 0; x_traj(3,t); 0; x_traj(4,t); q5; q6];
        T = solve_kinematics(q_aug, joint_to_com, rot);
        handle = plot_single_legged_robot(T, dim, 0);
        drawnow limitrate;
    end
    time = toc;
    disp("Simulation time = " + tmax + " s");
    disp("Total time = " + time + " s");
    disp("Speed up = Ttot/tmax " + time/tmax);
    disp("One figure takes " + time/n_speedup + " s");
end

% end