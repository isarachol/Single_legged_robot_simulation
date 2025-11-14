function R = Rx(angle)
%Rx Get the 3x3 rotation matrix around the E1 axis ("x")
%
%   input:
%       angle = amount of rotation, in radians
%
%   output:
%       R = 3x3 rotation matrix

R = [1 0            0;
     0 cos(angle)  -sin(angle);
     0 sin(angle)   cos(angle)];

end