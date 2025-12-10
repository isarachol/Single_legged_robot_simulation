% function [results] = sim_dynamic_3joint()

close all;
clc;

dof = 5;
disp("Dynamic test (" + dof + " joints)")
addpath("Other_configs\5dof\")

%% Robot Descriptions
% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

%% Setup
% time
tmax = 2;
dt = 0.0001;
t_span = 0:dt:tmax-dt;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% initial conditions
q0 = [0; pi/2; 0; 0; 0]; % rad
dq0 = [10; 10; 10; 10; 10]; % rad/s
x0 = [q0;dq0];

% physical constants
damp = 0.1; % N/(rad/s)
damp_mat = damp * ones([dof,1]);

% Control goal
q1_ = 0;
q2_ = 0.2;
q3_ = pi-2*q2_;
q4_ = q2_-pi/2;
q5_ = 0;
xd = [q1_; q2_; q3_; q4_; q5_];

%% Solve for trajectory
filename = "xtraj_E_5dof_control_pd+.mat"; % "xtraj_KE_5dof.mat" "xtraj_KE_5dof_damp.mat" "xtraj_KE_5dof_dampgrav.mat"
if ~isfile(filename)
    grav = 1;

    x_traj = zeros(size(x0,1),n);
    u_traj = zeros(dof,n);
    E = zeros(2, n);
    x_traj(:,1) = x0;
    E(:,1) = ke_pe_test_5dof(x_traj(:,1), grav);
    
    for i=2:n
        res = solve_5dof_pdgrav(x_traj(:,i-1), xd, damp_mat, grav); %solve_4dof_pdgrav(x_traj(:,i-1), xd, damp_mat, grav);
        bolddotx_t = res.bolddotx;
        u_traj(:,i) = res.u;
        x_traj(:,i) = x_traj(:,i-1) + bolddotx_t * dt; % forward euler
        E(:,i) =  ke_pe_test_5dof(x_traj(:,i), grav); % ke_test_5dof(x_traj(:,i));
    end

    % Shift PE so that it's referenced to 0
    PE_ref = min(E(2,:));
    E(2,:) = E(2,:) - PE_ref;
    error = xd - x_traj(1:dof,:);
    
    save(filename, "x_traj", "E", "u_traj", "error");
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
legend(["KE", "PE", "E_{tot}"]);
set(gcf, 'Position', [100 100 800 200]);
fontname("Times New Roman");
fontsize(16, "points");

figure()
plot(t_span, u_traj(1,:), 'LineWidth', 2);
hold on
plot(t_span, u_traj(2,:), 'LineWidth', 2);
plot(t_span, u_traj(3,:), 'LineWidth', 2);
plot(t_span, u_traj(4,:), 'LineWidth', 2);
plot(t_span, u_traj(5,:), 'LineWidth', 2);
xlabel("Time (s)");
ylabel("Torque (Nm)");
legend(["T1", "T2", "T3", "T4", "T5"]);
set(gcf, 'Position', [100 100 800 200]);
fontname("Times New Roman");
fontsize(16, "points");

figure()
plot(t_span, error(1,:), 'LineWidth', 2);
hold on
plot(t_span, error(2,:), 'LineWidth', 2);
plot(t_span, error(3,:), 'LineWidth', 2);
plot(t_span, error(4,:), 'LineWidth', 2);
plot(t_span, error(5,:), 'LineWidth', 2);
xlabel("Time (s)");
ylabel("Error (rad)");
legend(["e1", "e2", "e3", "e4", "e5"]);
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
    % title("Pixaar simulation: dynamic (kinetic energy only)");
    axis equal;
    xlim([-0.2, 0.1]);
    ylim([-0.1, 0.1]);
    zlim([0, 0.2]);
    f.Position = [250,70,1000,800];
    view(135,15); %(135,15);
    vecsize = 0.02;
    cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";

    q_aug = [0; 0; x_traj(1,1); x_traj(2,1); 0; x_traj(3,1); 0; x_traj(4,1); x_traj(5,1); q6];
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
        q_aug = [0; 0; x_traj(1,t); x_traj(2,t); 0; x_traj(3,t); 0; x_traj(4,t); x_traj(5,1); q6];
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