function Tz = Tz(angle, a)
%Tz Get the 4x4 homogeneous transformation matrix for a rotation around the E3 axis ("z")
%
%   input:
%       angle = amount of rotation, in radians
%       a = relative position from point i-1 to i
%
%   output:
%       Tz = 4x4 transformation matrix

R = Rz(angle);
p = R * a;
Tz = [R, p];
Tz = [Tz; 0 0 0 1;];

% Tz = [cos(angle) -sin(angle) 0 a*cos(angle);
%       sin(angle)  cos(angle) 0 a*sin(angle);
%       0           0          1 0;
%       0           0          0 1]; % Isara

end