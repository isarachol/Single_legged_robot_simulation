function pts = get_box_points(T, dim)
% Transform the box (rotate and translate) with T
% input T = transformation matrix
%       dim = dimensions of the box

len = dim(1); w = dim(2); h = dim(3);

% corners = zeros(3,8);
% corners(:,1) = [-len/2; -w/2; -h/2];
% corners(:,2) = [-len/2; w/2; -h/2];
% corners(:,3) = [len/2; w/2; -h/2];
% corners(:,4) = [len/2; -w/2; -h/2];
% corners(:,5) = [-len/2; -w/2; h/2];
% corners(:,6) = [-len/2; w/2; h/2];
% corners(:,7) = [len/2; w/2; h/2];
% corners(:,8) = [len/2; -w/2; h/2];
corners = zeros(3,10);
corners(:,1) = [-len/2; -w/2; -h/2];
corners(:,2) = [-len/2; w/2; -h/2];
corners(:,3) = [len/2; w/2; -h/2];
corners(:,4) = [len/2; -w/2; -h/2];
% corners(:,5) = [-len/2; -w/2; -h/2];
corners(:,5) = [-len/2; -w/2; h/2];
corners(:,6) = [-len/2; w/2; h/2];
corners(:,7) = [len/2; w/2; h/2];
corners(:,8) = [len/2; -w/2; h/2];
% corners(:,10) = [-len/2; -w/2; h/2];

% Translate the points on the bar so that the origin is the point of
% rotation
corners = corners; % + shift_R_axis; % only move along x axis of the bar % Isara
corners = [corners; ones(1,size(corners,2))]; % make it 4x1 vec

% Rotate the bar around the origin
pts = T * corners;
pts = pts(1:3,:); % back to 3x1 vec

end