function handles = plot_single_legged_robot(T, dim, plot_frame, no_plot)
% Assumes a figure is already initialized

    arguments
        T
        dim
        plot_frame logical = 0
        no_plot {mustBeNumeric} = zeros(1,numel(T))
    end
    
    handles = {};

    for i=1:numel(T)
        if ~ismember(i, no_plot)
            handles{i} = plot_box_3d(T{i}, dim(:,i), plot_frame);
        end
    end

end