function x = kinematics(q, r)
% kinematics solves for positions of each joint (frame) in Eucledean space
% Input q = [q1; q2; q3; q4; q5] where qi = scalar
%       r = [r12, r23, r34, r45, r56] where r is relative next frame r_ij = R3 vector
%       a = [a12, a23, a34, a45, a56] where a is relative centor of mass a_ij = R3 vector
% Output x = [x1, x2, x3, x4, x5, x6] where is position of each joint
%        xi = E3 vector


x = zeros(3,6); % x1 is always [0;0;0] for now
pnaught = [0; 0; 0; 1];

A_j1_j2 = [Ry(q(1)), Ry(q(1)) * r(:,1)];
A_j1_j2 = [A_j1_j2; 0 0 0 1];

A_j2_j3 = [Rx(q(2)), Rx(q(2)) * r(:,2)];
A_j2_j3 = [A_j2_j3; 0 0 0 1];

A_j3_j4 = [Rx(q(3)), Rx(q(3)) * r(:,3)];
A_j3_j4 = [A_j3_j4; 0 0 0 1];

A_j4_j5 = [Rx(q(4)), Rx(q(4)) * r(:,4)];
A_j4_j5 = [A_j4_j5; 0 0 0 1];

A_j5_j6 = [Rz(q(5)), Rz(q(5)) * r(:,5)];
A_j5_j6 = [A_j5_j6; 0 0 0 1];

vec_4to3 = [1 0 0 0; 0 1 0 0; 0 0 1 0];
x(:,2) = vec_4to3 * A_j1_j2 * pnaught;
x(:,3) = vec_4to3 * A_j1_j2 * A_j2_j3 * pnaught;
x(:,4) = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * pnaught;
x(:,5) = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * A_j4_j5 * pnaught;
x(:,6) = vec_4to3 * A_j1_j2 * A_j2_j3 * A_j3_j4 * A_j4_j5 * A_j5_j6 * pnaught;

end