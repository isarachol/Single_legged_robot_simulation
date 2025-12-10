function [results] = sim_static()

close all;
clc;

disp('Kinematics test')

% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

% Define neutral config
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
q = [0; 0; q1; q2; 0; q3; 0; q4; q5; q6];

% Plot 3D
show_plot_4views = 1;
if(show_plot_4views)
    plot_frame = 0;
    figure()
    % ============================= 3D =================================
    subplot(2,2,2)
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    % title("Pixaar simulation: static");
    axis equal;
    view (150,30);
    % cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";
    
    vecsize_body = 0.02;
    vecsize = [0,0,1,1,0,1,0,1,0,1] * vecsize_body;
    T = solve_kinematics(q, joint_to_com, rot);
    plot_single_legged_robot(T, dim, plot_frame, vecsize);
    % world frame
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);
    % ============================= Top =================================
    subplot(2,2,1)
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    axis equal;
    view (90,90);
    
    plot_single_legged_robot(T, dim, plot_frame, vecsize);
    % world frame
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);
    % ============================= Right =================================
    subplot(2,2,4)
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    axis equal;
    view (180,0);
    
    plot_single_legged_robot(T, dim, plot_frame, vecsize);
    % world frame
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);
    % ============================= Front =================================
    subplot(2,2,3)
    hold on
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    axis equal;
    view (90,0);
    
    plot_single_legged_robot(T, dim, plot_frame, vecsize);
    % world frame
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);
end

show_plot = 1;
if(show_plot)
    f = figure();
    hold on
    grid on;
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    % title("Pixaar simulation: static");
    axis equal;
    xlim([-0.2, 0.1]);
    ylim([-0.1, 0.1]);
    zlim([0, 0.2]);
    f.Position = [250,70,1000,800];
    view (135,15);
    cp = constantplane("z", 0);
    % cp = constantplane("z", 0);
    % cp.FaceColor = "#7CFC00";
    
    vecsize_body = 0.02;
    vecsize = [0,0,1,1,0,1,0,1,0,1] * vecsize_body;
    T = solve_kinematics(q, joint_to_com, rot);
    plot_single_legged_robot(T, dim, 0, vecsize);
    % world frame
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),1,0,0, 0.02, 'r', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,1,0, 0.02, 'g', "MaxHeadSize", 100, 'LineWidth', 2);
    quiver3(T{2}(1,4),T{2}(2,4),T{2}(3,4),0,0,1, 0.02, 'b', "MaxHeadSize", 100, 'LineWidth', 2);
end

end