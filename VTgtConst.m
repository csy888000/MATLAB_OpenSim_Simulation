classdef VTgtConst < VTgt0
    properties
    end
    
    methods
% -----------------------------------------------------------------------------------------------------------------
        function vgconst = VTgtConst(v_tgt_, rng_xy_, res_map_, rng_get_, res_get_)
            if nargin < 5
                v_tgt = [1.4, 0];
                rng_xy = [[-30, 30]; [-30, 30]];
                res_map = [2, 2];
                rng_get = [[-5, 5]; [-5, 5]];
                res_get = [2, 2];
            else
                v_tgt = v_tgt_;
                rng_xy = rng_xy_;
                res_map = res_map_;
                rng_get = rng_get_;
                res_get = res_get_;
            end
            
            vgconst = vgconst@VTgt0(rng_xy, res_map, rng_get, res_get);
            vgconst.vtgt = 1*vgconst.map;
%             vgconst.vtgt(1)%.fill(v_tgt(1))
%             vgconst.vtgt(2)%.fill(v_tgt(2))
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vgconst = create_vtgt_const(vgconst, v_tgt)
%             vgconst.vtgt(1)%.fill(v_tgt(1))
%             vgconst.vtgt(2)%.fill(v_tgt(2))        

            vgconst.vtgt_interp_x = py.interpolate.interp2d(vgconst.map(1,:,1), vgconst.map(2,1,:), (vgconst.vtgt(1))');
            vgconst.vtgt_interp_y = py.interpolate.interp2d(vgconst.map(1,:,1), vgconst.map(2,1,:), (vgconst.vtgt(2))');
        end
    end
end