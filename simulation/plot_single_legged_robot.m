function T = plot_single_legged_robot(T, dim)
% Assumes a figure is already initialized

for i=1:numel(T)
    plot_box_3d(T{i}, dim(:,i));
end

end