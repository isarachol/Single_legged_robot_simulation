function handles = plot_rectbar_3d_robotjoints(T, dim) %, shift_R_axis)
%plot_rectbar_3d_robotjoints Plot a rectangular bar in 2D given a rotation,
%translation, and some of its geometry. Assumes that rotation R is applied at the
%leftmost location on the bar, i.e., the neutral pose is a cantilevered
%beam. This is the constraint associated with a serial chain manipulator.
%
%   inputs:
%       T = 4x4 transformation matrix to apply to the bar (extrinsic)
%       translate = the translation to apply after rotating. Still 3x1.
%       dim = dimension of the box along xyz directions of local frame
%       shift_R_axis = shift of axis of rotation in xyz of local frame
%       trans = translation of box after rotating
%
%   outputs:
%       handles = results of the plot function, for the caller's use later
%       if desired
%

% We assume that the caller has already prepared a figured window for us.

% The corners are these position vectors, treating the bar as uniform
% density means its C.o.M. is at the geometric center as well.

len = dim(1); w = dim(2); h = dim(3);

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
corners = corners; % + shift_R_axis; % only move along x axis of the bar % Isara
corners = [corners; ones(1,8)]; % make it 4x1 vec

% Rotate the bar around the origin
r = T * corners;
r = r(1:3,:); % back to 3x1 vec
% Translate the bar into its final location in space
% r = r + trans; % Isara

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
% draw body frames
quiver3(T(1,4), T(2,4), T(3,4), T(1,1), T(2,1), T(3,1), 0.02, 'r', "MaxHeadSize", 100);
quiver3(T(1,4), T(2,4), T(3,4), T(1,2), T(2,2), T(3,2), 0.02, 'g', "MaxHeadSize", 100);
quiver3(T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3), 0.02, 'b', "MaxHeadSize", 100);
end
