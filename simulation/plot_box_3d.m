function handles = plot_box_3d(T, dim, vecsize, plot_frame)
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
    plot_frame logical = 1
end

% compute box points
pts = get_box_points(T, dim);

handles = cell([5,1]); % know for sure
% draw two rectangles
handles{1} = plot3(pts(1,:), pts(2,:), pts(3,:), 'r');
% handlecount = handlecount+1;

% connect two rectangles
X = [pts(1,2); pts(1,7); pts(1,8); pts(1,3); pts(1,4); pts(1,9)];
Y = [pts(2,2); pts(2,7); pts(2,8); pts(2,3); pts(2,4); pts(2,9)];
Z = [pts(3,2); pts(3,7); pts(3,8); pts(3,3); pts(3,4); pts(3,9)];
handles{2} = plot3(X, Y, Z, 'r');

% draw body frames
if plot_frame
    handles{3} = quiver3(T(1,4), T(2,4), T(3,4), T(1,1), T(2,1), T(3,1), vecsize, 'r', "MaxHeadSize", 100);
    handles{4} = quiver3(T(1,4), T(2,4), T(3,4), T(1,2), T(2,2), T(3,2), vecsize, 'g', "MaxHeadSize", 100);
    handles{5} = quiver3(T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3), vecsize, 'b', "MaxHeadSize", 100);
end
end
