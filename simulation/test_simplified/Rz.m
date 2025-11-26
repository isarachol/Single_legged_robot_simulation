function R = Rz(angle)
%Rz Get the 3x3 rotation matrix around the E3 axis ("z")
%
%   input:
%       angle = amount of rotation, in radians
%
%   output:
%       R = 3x3 rotation matrix

R = [cos(angle), -sin(angle), 0;
     sin(angle), cos(angle),  0;
     0, 0, 1];

tol = 10^(-15);
for i=1:3
    for j=1:3
        if isnumeric(R(i,j))
            if abs(R(i,j))<tol
                R(i,j) = 0;
            end
        end
    end
end

end