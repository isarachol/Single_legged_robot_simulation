function handles = plot_single_legged_robot(T, dim, plot_frame, vecsize, no_plot, force)
% Assumes a figure is already initialized

    arguments
        T
        dim
        plot_frame logical = 0
        vecsize {mustBeNumeric} = zeros(1,numel(T))
        no_plot {mustBeNumeric} = zeros(1,numel(T))
        force {mustBeNumeric} = zeros(3,numel(T))
    end
    
    handles = {};

    for i=1:numel(T)
        if ~ismember(i, no_plot)
            if plot_frame
                if vecsize(i)==0
                    plot_frame_i = 0;
                else
                    plot_frame_i = 1;
                end
            else
                plot_frame_i = plot_frame;
            end

            if norm(force(:,i)) == 0
                handles{i} = plot_box_3d(T{i}, dim(:,i), plot_frame_i, vecsize(i));
            else
                handles{i} = plot_box_3d(T{i}, dim(:,i), plot_frame_i, vecsize(i), force(:,i));
            end
        end
    end

end