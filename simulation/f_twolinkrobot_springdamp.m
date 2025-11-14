function [bolddotx] = f_twolinkrobot_springdamp(x, constants)
%f_doublepend The dynamics function f(x), in state-space form, for a
%system of N=2 rigid bodies as a double pendulum
%
%   This function implements the right-hand-side of the ordinary
%   differential equation \dot x = f(x)
%
%   Inputs:
%       x == state at time t, [theta1; theta2; dottheta1; dottheta2]
%       constants == a MATLAB struct containing the constants to plug into
%       the dynamics

% pick out the constants
m1 = constants.m1;
m2 = constants.m2;
Iz1 = constants.Iz1;
Iz2 = constants.Iz2;
% g = constants.g; % no gravity for now
k1 = constants.k1;
k2 = constants.k2;
q1bar = constants.q1bar;
q2bar = constants.q2bar;
a1 = constants.a1;
a2 = constants.a2; % Notice that a2 is length of distal segment, does not come into the dynamics - we only care about COM position, ell2.
ell1 = constants.ell1;
ell2 = constants.ell2;
damping = constants.damping;

% pick out variables from x so we don't get confused (just for convenience)
q1 = x(1);
q2 = x(2);
dq1 = x(3);
dq2 = x(4);

% equations are long so let's do them more carefully instead of one line.
bolddotx = zeros(size(x)); % result needs to be same size as x... 6x1
bolddotx(1) = dq1; % velocities
bolddotx(2) = dq2;

% Let's do this by adding each term separately, less confusing
% for me at least

% Coriolis terms:
% first the matrix...
C = -m2*a1*ell2*sin(q2) * [dq2,     dq1 + dq2;
                           -dq1,    0];
% ...then, it's really C(q, dq)*dq,
C = C*[dq1; dq2];

% Spring forces (generalized)
Fs = [k1 0; 0 k2] * [q1 - q1bar; q2 - q2bar]; 

% Damping forces (generalized)
Fd = damping * [dq1; dq2];

% From our matrix inverse symbolic solution... now we can call it as a
% function!
Minv = Minv_twolinkrobot(x(1:2), m1, m2, Iz1, Iz2, a1, ell1, ell2);

% Now as a matrix equation:
ddotq = Minv*(-C - Fs -Fd);
% the generalized accelerations are the last three elements
bolddotx(3:4) = ddotq;

% prevent numerical issues
bolddotx = numerical_check(bolddotx, constants.tol);

end