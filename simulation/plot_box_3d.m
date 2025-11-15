function handles = plot_box_3d(T, dim, vecsize)
% plot_box_3d Plot a 3D box given a transformation and geometry 
% and optional argument vecsize for frame plotting.
% Assumes that transformation is extrinsic (x1 = R0_1 x0 + T0_1)
%
%   inputs:
%       T = 4x4 transformation matrix to apply to the bar (extrinsic)
%       dim = dimension of the box along xyz directions of local frame
%
%   outputs:
%       handles = results of the plot function, for the caller's use later
%       if desired
%

% We assume that the caller has already prepared a figured window for us.

% The corners are these position vectors, treating the bar as uniform
% density means its C.o.M. is at the geometric center as well.
arguments
    T
    dim
    vecsize (1,1) double = 0.02
end

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
quiver3(T(1,4), T(2,4), T(3,4), T(1,1), T(2,1), T(3,1), vecsize, 'r', "MaxHeadSize", 100);
quiver3(T(1,4), T(2,4), T(3,4), T(1,2), T(2,2), T(3,2), vecsize, 'g', "MaxHeadSize", 100);
quiver3(T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3), vecsize, 'b', "MaxHeadSize", 100);
end
