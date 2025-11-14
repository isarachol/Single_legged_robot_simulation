function handles = plot_rectbar_3d_robotjoints(R, side, joint_to_com, translate)
%plot_rectbar_2d_robotjoints Plot a rectangular bar in 2D given a rotation,
%translation, and some of its geometry. Assumes that rotation R is applied at the
%leftmost location on the bar, i.e., the neutral pose is a cantilevered
%beam. This is the constraint associated with a serial chain manipulator.
%
%   inputs:
%       R = 3x3 rotation matrix to apply to the bar
%       translate = the translation to apply after rotating. Still 3x1.
%       len = length of the bar
%       w = width of the bar
%
%   outputs:
%       handles = results of the plot function, for the caller's use later
%       if desired
%

% We assume that the caller has already prepared a figured window for us.

% The corners are these position vectors, treating the bar as uniform
% density means its C.o.M. is at the geometric center as well.

len = side(1); w = side(2); h = side(3);

corners = zeros(3,8);
corners(:,1) = [-len/2; -w/2; -h/2];
corners(:,2) = [-len/2; w/2; -h/2];
corners(:,3) = [len/2; w/2; -h/2];
corners(:,4) = [len/2; -w/2; -h/2];
corners(:,5) = [-len/2; -w/2; h/2];
corners(:,6) = [-len/2; w/2; h/2];
corners(:,7) = [len/2; w/2; h/2];
corners(:,8) = [len/2; -w/2; h/2];

% Translate the points on the bar so that the origin is the point of
% rotation
corners = corners + joint_to_com; % only move along x axis of the bar % Isara

% Rotate the bar around the origin
r = R * corners; % Isara
% Translate the bar into its final location in space
r = r + translate; % Isara

% finally, plot, and save the handles.
% an easy way to do so is to plot a line between each pair of points.
handles = {};
handlecount = 1;
% draw two rectangles
for i=1:8
    j = i+1;
    if i == 4
        j = 1;
    end
    if i==8
        j = 5;
    end
    X = [r(1,i); r(1,j)];
    Y = [r(2,i); r(2,j)];
    Z = [r(3,i); r(3,j)];
    handles{handlecount} = plot3(X, Y, Z, 'r');
    handlecount = handlecount+1;

    % connect two rectangles
    if i<5
        X = [r(1,i); r(1,i+4)];
        Y = [r(2,i); r(2,i+4)];
        Z = [r(3,i); r(3,i+4)];
        handles{handlecount} = plot3(X, Y, Z, 'r');
        handlecount = handlecount+1;
    end
end

