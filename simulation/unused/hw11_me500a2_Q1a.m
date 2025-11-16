function [results] = hw11_me500a2_Q1a()

% hw11_me500a2_Q1a.m
% (C) Andrew Sabelhaus, 2025

close all;
clc;

disp('Dynamics of a two-link planar robot arm: plotting examples.')

%% Autograder setup

% Check if we're running MATLAB (on your computer) versus on the autograder (not MATLAB)
licenses = license('inuse');
is_matlab = 0;
for i=1:size(licenses,2)
    if strcmp(licenses(i).feature, "matlab")
        is_matlab=1;
    end
end

if is_matlab
    disp('Running MATLAB, so we will plot the figures.')
else
    disp('Not running MATLAB, no figure plots, hopefully will run faster.')
end

%% Setup: constants and initial conditions

% Geometry of the bar:
a1 = 1.0;
a2 = 0.6;
ell1 = a1/2; % com is in the middle of the bar
ell2 = a2/2;
w1 = 0.1; % width of the bar
w2 = 0.1;

%%%%% Initial conditions:
% For this problem, we're only simulating the two unconstrained
% coordinates.
theta1_0 = pi/4; 
theta2_0 = pi/6;

q0 = [theta1_0; theta2_0];

% Calculate the locations in space for the joints of the robot - needed
% for plotting
jointpts = get_planartwolinkrobot_proximaljoints(q0, a1);

%% Plot if this is matlab, running on your computer.
% No plotting in the autograder.
if(is_matlab)

    figure;
    hold on;
    xlabel("E1");
    ylabel("E2");
    % limits should be if the arm is fully stuck out straight, plus an
    % extra bit of space.
    maxcoord = (a1+a2)*1.2;
    limits = [-maxcoord, maxcoord];
    xlim(limits);
    ylim(limits);
    % plot some lines to visualize the E1, E2 axes
    line(2*xlim, [0,0], [0,0]);
    line([0,0], 2*ylim, [0,0]);
    E1 = [1;0;0];
    E2 = [0;1;0];
    vecscale = 0.5;
    quiverarrowsize = 100;
    quiver3(0,0,0, E1(1), E1(2), E1(3), vecscale, 'Color', 'r');
    quiver3(0,0,0, E2(1), E2(2), E2(3), vecscale, 'Color', 'g');

    % now plot
    bar1_handles = plot_rectbar_2d_robotjoints(Rz(q0(1)), jointpts(:,1), a1, w1);
    % Note that the distal bar needs two rotations, since its angle q_2 is with
    % respect to the frame at the tip of the previous bar - NOT the world E1 axis!!!
    q2_total = q0(2) + q0(1); % Isara
    bar2_handles = plot_rectbar_2d_robotjoints(Rz(q2_total), jointpts(:,2), a2, w2);
    title('ME 500 - Planar Two-Link Robot Arm Plotting Example')
    view(0, 90);
end

% some variables to test with the autograder
results = jointpts;

end















