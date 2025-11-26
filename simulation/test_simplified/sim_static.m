function [results] = sim_static()

close all;
clc;

disp('Kinematics test')

% load robot description (geometry)
desc_filename = 'robot_desc_v2.mat';
make_robot_description_v2(desc_filename);
load(desc_filename);

% Define configuration
% q1 = 0.1;
q1 = 0.5;
q2 = pi-2*q1;
% q4 = q2-pi/2;
% q5 = 0;
% q6 = 0;
q = [q1; 0; q2];

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
    T = solve_kinematics(q, joint_to_com, rot);
    plot_single_legged_robot(T, dim, 1);
    % for i=1:numel(T)
    %     plot_box_3d(T{i}, dim(:,i));
    % end
end

end