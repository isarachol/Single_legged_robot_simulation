function R = Ry(angle)
%Ry Get the 3x3 rotation matrix around the E2 axis ("y")
%
%   input:
%       angle = amount of rotation, in radians
%
%   output:
%       R = 3x3 rotation matrix

R = [cos(angle) 0 sin(angle);
     0          1 0;
    -sin(angle) 0 cos(angle)];

end