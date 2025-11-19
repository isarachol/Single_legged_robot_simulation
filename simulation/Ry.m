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