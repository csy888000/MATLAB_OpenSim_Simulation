classdef VTgt0
    properties
        res_map
        res_get
        rng_get
        map
        map_rng_xy
        vtgt
        vtgt_interp_x
        vtgt_interp_y
    end
    
    methods
% -----------------------------------------------------------------------------------------------------------------
        function vtgt00 = VTgt0(rng_xy_, res_map_, rng_get_, res_get_ )
            if nargin < 4
                rng_xy=[[-30, 30]; [-30, 30]];
                res_map=[2, 2];
                rng_get=[[-5, 5]; [-5, 5]];
                res_get=[2, 2];
            else
                rng_xy = rng_xy_;
                res_map = res_map_;
                rng_get=rng_get_;
                res_get=res_get_;
            end
            % set parameters
            vtgt00.res_map = res_map;
            vtgt00.res_get = res_get;
            vtgt00.rng_get = rng_get;

            % map coordinate and vtgt
            vtgt00 = vtgt00.create_map(rng_xy);
            vtgt00.vtgt = -1*vtgt00.map;
        end

    % -----------------------------------------------------------------------------------------------------------------
        function del_(vtgt00)
            nn = "empty";
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vtgt00 = create_map(vtgt00, rng_xy)
            vtgt00.map_rng_xy = rng_xy;
            vtgt00.map = vtgt00.generate_grid(rng_xy, vtgt00.res_map);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function grid = generate_grid(vtgt00, rng_xy_, res_)
            if nargin < 3
                rng_xy=[[-10, 10]; [-10, 10]];
                res=[2, 2];
            else
                rng_xy = rng_xy_;
                res = res_;
            end
            xo = .5*(rng_xy(1,1)+rng_xy(1,2));
            x_del = (rng_xy(1,2)-xo)*res(1);
            yo = .5*(rng_xy(2,1)+rng_xy(2,2));
            y_del = (rng_xy(2,2)-yo)*res(2);
%             grid = cell(1,2);
            [grid_2, grid_1] = meshgrid(-x_del:x_del, -y_del:y_del);
%             grid_2 = grid_1';
            grid(1,:,:) = grid_1/res(1) + xo;
            grid(2,:,:) = grid_2/res(2) + yo;
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vtgt = get_vtgt(vtgt00, xy) % in the global frame
%             vtgt_x = vtgt00.vtgt_interp_x(xy(1), xy(2));
%             vtgt_y = vtgt00.vtgt_interp_y(xy(1), xy(2));
            [vtgt_x, vtgt_y] = interp_sink(vtgt00, xy);
            
            
            vtgt = [vtgt_x, vtgt_y];
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vtgt_ = get_vtgt_field_local(vtgt00, pose)
            xy = pose(1:2);
            th = pose(3);

            % create query map
            get_map0 = vtgt00.generate_grid(vtgt00.rng_get, vtgt00.res_get);
            get_map_x = cos(th)*get_map0(1,:,:) - sin(th)*get_map0(2,:,:) + xy(1);
            get_map_y = sin(th)*get_map0(1,:,:) + cos(th)*get_map0(2,:,:) + xy(2);

            % get vtgt
%             vtgt_x0 = reshape([vtgt00.vtgt_interp_x(x, y) for x, y in zip(get_map_x(:), get_map_y(:))], size(get_map_x))
%             vtgt_y0 = reshape([vtgt00.vtgt_interp_y(x, y) for x, y in zip(get_map_x(:), get_map_y(:))], size(get_map_y)) 
            interp_x_array = zeros(1,length(get_map_x(:)));
            interp_y_array = zeros(1,length(get_map_y(:)));
            for i = 1:length(get_map_x(:))
                x = get_map_x(i);
                y = get_map_y(i);
                xy = [x, y];
                [interp_x, interp_y] = interp_sink(vtgt00, xy);
                interp_x_array(i) = interp_x;
                interp_y_array(i) = interp_y;
%                 vtgt_x0 = reshape(vtgt00.vtgt_interp_x(x, y), size(get_map_x));
%                 vtgt_y0 = reshape(vtgt00.vtgt_interp_y(x, y), size(get_map_y));
            end
            
            vtgt_x0 = reshape(interp_x_array, size(get_map_x));
            vtgt_y0 = reshape(interp_y_array, size(get_map_y));
            vtgt_x = cos(-th)*vtgt_x0 - sin(-th)*vtgt_y0;
            vtgt_y = sin(-th)*vtgt_x0 + cos(-th)*vtgt_y0;

            % debug
    %         """
    %         if xy[0] > 10:
    %             import matplotlib.pyplot as plt
    %             plt.figure(100)
    %             plt.axes([.025, .025, .95, .95])
    %             plt.plot(get_map_x, get_map_y, '.')
    %             plt.axis('equal')
    % 
    %             plt.figure(101)
    %             plt.axes([.025, .025, .95, .95])
    %             R = np.sqrt(vtgt_x0**2 + vtgt_y0**2)
    %             plt.quiver(get_map_x, get_map_y, vtgt_x0, vtgt_y0, R)
    %             plt.axis('equal')
    % 
    %             plt.show()
    %         """

            vtgt_ = [vtgt_x; vtgt_y];
        end
    end
end
            


