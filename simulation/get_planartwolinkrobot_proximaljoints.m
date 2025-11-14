function jointpts = get_planartwolinkrobot_proximaljoints(q, a1)
%get_doublepend_proximaljoints Get the position vectors of the proximal
% joints for the rigid body double pendulum given, i.e., the translations
% to apply when plotting.
%
%   input:
%       q = 2-vector, joint angles, in radians
%       a_i = length of the rigid bodies
%
%   output:
%       coms = 3x2 matrix of two centers of mass (columns)

jointpts = zeros(3,2);

% The first joint's location in space is [0;0;0], so no changes need to be
% made.
% For the second joint, it's full distance of first link
T2 = Tz(q(1), a1);

% The homogeneous "zero" vector is
pnaught = [0; 0; 0; 1];

% Here's a trick to pull out the first three elements of a 4x1 vector:
I_cartesian = [eye(3), zeros(3,1)]; % like [1, 0, 0 0; ... 0, 0, 1, 0]

% so the rotation point is:
jointpts(:,2) = I_cartesian * T2 * pnaught; % Isara

end