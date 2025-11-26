function Tx = Tx(angle, a)
%Rx Get the 3x3 rotation matrix around the E1 axis ("x")
%
%   input:
%       angle = amount of rotation, in radians
%
%   output:
%       R = 3x3 rotation matrix

R = Rx(angle);
p = R * a;
Tx = [R, p];
Tx = [Tx; 0 0 0 1;];

end