function [results] = hw11_me500a2_Q5b()

% hw11_me500a2_Q5b.m
% (C) Andrew Sabelhaus, 2025

close all;
clc;

disp('Dynamics of a two-link planar robot arm, spring and damping with gravity.')

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

tmax = 8;
dt = 0.0001;

n = tmax/dt; % number of timesteps, for loop-ing
% somehow matlab doesn't round correctly. sometimes...
n = round(n);

% We're given these quantities in the problem setup:
m1 = 2; % kg
m2 = 5;
damping = 10;
k1 = 100;
k2 = 10;
q1bar = pi/2;%pi/6;
q2bar = 0;%pi/12;
grav = 9.81;
tol = 10^4; % numerical tolerance for instability: if f(x) is greater than this, in any element, assume our simulation has encountered a big issue and stop integrating

% Geometry of the bar:
a1 = 1.0;
a2 = 0.6;
ell1 = a1/2; % com is in the middle of the bar
ell2 = a2/2;
w1 = 0.1; % width of the bar
w2 = 0.1;
Iz1 = (1/12)*m1*(a1^2 + w1^2);
Iz2 = (1/12)*m2*(a2^2 + w2^2);

% constants for the dynamics
c.m1 = m1;
c.m2 = m2;
c.grav = grav;
c.damping = damping;
c.k1 = k1;
c.k2 = k2;
c.q1bar = q1bar;
c.q2bar = q2bar;
c.a1 = a1;
c.a2 = a2;
c.ell1 = ell1;
c.ell2 = ell2;
c.Iz1 = Iz1;
c.Iz2 = Iz2;
c.tol = tol;

%%%%% Initial conditions:
% For this problem, we're only simulating the two unconstrained
% coordinates.
theta1_0 = pi/4; 
theta2_0 = pi/6;

dottheta1_0 = 2;
dottheta2_0 = 0;

q0 = [theta1_0; theta2_0];
dq0 = [dottheta1_0; dottheta2_0];

x0 = [q0; dq0];

%% Simulate to obtain trajectory

% insert our initial condition as the 1st element in traj
% note that traj will have n+1 columns.
x_traj = zeros(size(x0,1), n+1);
x_traj(:,1) = x0;

% actual simulation! Use Forward Euler for now.
for t=1:n
    %%%%% DYNAMICS
    bolddotx_t = f_twolinkrobot_springdampgrav(x_traj(:,t), c);
    % Forward Euler
    x_traj(:,t+1) = x_traj(:,t) + dt*bolddotx_t;
end

%% Plot if this is matlab, running on your computer.
% No plotting in the autograder.
if(is_matlab)

    % We will skip by a certain number of steps in order for the visualization
    % to go faster.
    speedup = 100;
    n_speedup = n/speedup;
    
    % set up the figure
    figure; hold on;
    grid on;
    title('ME 500 - Planar Two-Link Robot Arm Dynamics')
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

    % Initialize the first pose
    jointpos = get_planartwolinkrobot_proximaljoints(x_traj(1:2), a1);
    % noting that the joint angles are x(i,t), rot mat is Rz(x(i,t))
    
    bar1_handles = plot_rectbar_2d_robotjoints(Rz(x_traj(1,1)), jointpos(:,1), a1, w1);
    % Note that the distal bar needs two rotations, since its angle q_2 is with
    % respect to the frame at the tip of the previous bar - NOT the world E1
    % axis!!!
    q2_total = x_traj(1,1) + x_traj(2,1);
    bar2_handles = plot_rectbar_2d_robotjoints(Rz(q2_total), jointpos(:,2), a2, w2);
    
    % force MATLAB to render the whole figure before moving forward - computers
    % are funny
    drawnow;

    % plot over time - now, per speedup.
    for j=2:n_speedup
        % take the frames only at a certain interval.
        i=j*speedup;
        % this removes the lines and points from our plot
        for k=1:size(bar1_handles, 2)
            delete(bar1_handles{k});
        end
        for k=1:size(bar2_handles, 2)
            delete(bar2_handles{k});
        end
        % replot now at the i-th timestep in the trajectory
        jointpos = get_planartwolinkrobot_proximaljoints(x_traj(1:2, i), a1); 
        % noting that the joint angles are x(i,t), rot mat is Rz(x(i,t))
        bar1_handles = plot_rectbar_2d_robotjoints(Rz(x_traj(1,i)), jointpos(:,1), a1, w1);
        q2_total = x_traj(1,i) + x_traj(2,i);
        bar2_handles = plot_rectbar_2d_robotjoints(Rz(q2_total), jointpos(:,2), a2, w2);
        drawnow;
    end

    % % now plot
    % bar1_handles = plot_rectbar_2d_robotjoints(Rz(q0(1)), jointpts(:,1), a1, w1);
    % % Note that the distal bar needs two rotations, since its angle q_2 is with
    % % respect to the frame at the tip of the previous bar - NOT the world E1 axis!!!
    % q2_total = q0(1) + q0(2);
    % bar2_handles = plot_rectbar_2d_robotjoints(Rz(q2_total), jointpts(:,2), a2, w2);
    % title('ME 500 - Planar Two-Link Robot Arm Plotting Example')
    % view(0, 90);
end

% some variables to test with the autograder
results = x_traj;

end















