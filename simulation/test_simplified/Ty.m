function Ty = Ty(angle, a)
%Ry Get the 3x3 rotation matrix around the E2 axis ("y")
%
%   input:
%       angle = amount of rotation, in radians
%
%   output:
%       R = 3x3 rotation matrix

R = Ry(angle);
p = R * a;
Ty = [R, p];
Ty = [Ty; 0 0 0 1;];

end