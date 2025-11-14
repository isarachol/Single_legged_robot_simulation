function [results] = test_script()

close all;
clc;

disp('Kinematics test')

% Constants
leg1_l = 0.15;
leg1_w = 0.01;
leg1_h = 0.01;
leg1_dim = [leg1_w; leg1_l; leg1_h];
leg1_j_to_com = [0; leg1_l/2; 0];

leg2_l = 0.15;
leg2_w = 0.01;
leg2_h = 0.01;
leg2_dim = [leg2_w; leg2_l; leg2_h];
leg2_j_to_com = [0; leg2_l/2; 0];

head_l = 0.10;
head_w = 0.05;
head_h = 0.05;
head_dim = [head_w; head_h; head_l];
head_j_to_com = [0; head_h/2; head_l/3];

foot_l = 0.10;
foot_w = 0.06;
foot_h = 0.02;
foot_dim = [foot_w; foot_l; foot_h];
foot_j_to_com = [0; head_h/2; head_l/3];

% Define configuration
q1 = 0;
q2 = 0.1;
q3 = pi - 2*q2;
q4 = 0;
q5 = 0;
q = [q1;q2;q3;q4;q5];

% Define kinematics 
a0 = [];
a1 = [0; leg1_l; 0];
a2 = [0; leg2_l; 0];
a3 = [0; head_l*2/3; 0];
a = [[0;0;0], a1, a2, [0;0;0], a3];

x = kinematics(q, a);
% x = kinematics3(q, a1, a2, a3);
figure()
plot3(x(1,:), x(2,:), x(3,:), 'blue');
hold on
scatter3(x(1,:), x(2,:), x(3,:), 'red');
xlabel("E1");
ylabel("E2");
zlabel("E3");
axis equal;
view (-135,45);

figure()
xlabel("E1");
ylabel("E2");
zlabel("E3");
axis equal;
view (45,45);
hold on;
handle = plot_rectbar_3d_robotjoints(Rx(q(2))*Ry(q(1)), leg1_dim, leg1_j_to_com, x(:,2));
handle = plot_rectbar_3d_robotjoints(Rx(q(3))*Rx(q(2))*Ry(q(1)), leg2_dim, leg2_j_to_com, x(:,3));
handle = plot_rectbar_3d_robotjoints(Rz(q(5))*Rx(q(4))*Rx(q(3))*Rx(q(2))*Ry(q(1)), head_dim, head_j_to_com, x(:,5));

end