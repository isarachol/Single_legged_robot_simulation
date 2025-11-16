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
q1 = 0.1;
q2 = 0.2;
q3 = pi-2*q2;
q4 = q2-pi/2;
q5 = 0;
q6 = 0;
q = [0; 0; q1; q2; 0; q3; 0; q4; q5; q6];

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
    for i=1:numel(T)
        plot_box_3d(T{i}, dim(:,i));
    end
end

end