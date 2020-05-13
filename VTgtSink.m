classdef VTgtSink < VTgt0
    properties
%         vtgt
%         map
    end
    
    methods
% -----------------------------------------------------------------------------------------------------------------
        function vgsink = VTgtSink(rng_xy_, res_map_, rng_get_, res_get_)
            if nargin < 4
                rng_xy=[[-30, 30]; [-30, 30]];
                res_map=[2, 2];
                rng_get=[[-5, 5]; [-5, 5]];
                res_get=[2, 2];
            else
                rng_xy = rng_xy_;
                res_map = res_map_;
                rng_get = rng_get_;
                res_get = res_get_;
            end
            
            vgsink = vgsink@VTgt0(rng_xy, res_map, rng_get, res_get);
            vgsink.vtgt = -1*vgsink.map;
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vgsink = create_vtgt_sink(vgsink, p_sink, d_sink, v_amp_rng, v_phase0_)
            if nargin < 5
                v_phase0= (pi+pi).*rand(1) + -pi;
            else
                v_phase0 = v_phase0_;
            end

            % set vtgt orientations
            rng_xy = (-p_sink + (vgsink.map_rng_xy)')';
            vgsink.vtgt = -vgsink.generate_grid(rng_xy, vgsink.res_map);

            % set vtgt amplitudes
            vgsink = vgsink.set_sink_vtgt_amp(p_sink, d_sink, v_amp_rng, v_phase0);

%             vgsink.vtgt_interp_x = py.interpolate.interp2d(squeeze(vgsink.map(1,:,1)), squeeze(vgsink.map(2,1,:)), (vgsink.vtgt(1,:,:))');
%             vgsink.vtgt_interp_y = py.interpolate.interp2d(squeeze(vgsink.map(1,:,1)), squeeze(vgsink.map(2,1,:)), (vgsink.vtgt(2,:,:))');
%             vgsink.vtgt_interp_x = interp2(squeeze(vgsink.map(1,:,1)), squeeze(vgsink.map(2,1,:)), squeeze(vgsink.vtgt(1,:,:))', x_, y_);
%             vgsink.vtgt_interp_y = interp2(squeeze(vgsink.map(1,:,1)), squeeze(vgsink.map(2,1,:)), squeeze(vgsink.vtgt(2,:,:))', x_, y_);
        end
        
        function [interp_x, interp_y] = interp_sink(vgsink, xy_)
            x_ = xy_(1);
            y_ = xy_(2);
            a = squeeze(vgsink.map(1,:,1));
            b = squeeze(vgsink.map(2,1,:));
            c = squeeze(vgsink.vtgt(1,:,:))';
            interp_x = interp2(squeeze(vgsink.map(1,:,1)), squeeze(vgsink.map(2,1,:)), squeeze(vgsink.vtgt(1,:,:))', x_, y_);
            interp_y = interp2(squeeze(vgsink.map(1,:,1)), squeeze(vgsink.map(2,1,:)), squeeze(vgsink.vtgt(2,:,:))', x_, y_);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vgsink = set_sink_vtgt_amp(vgsink, p_sink, d_sink, v_amp_rng, v_phase0, d_dec_)
            if nargin < 6
                d_dec = 1;
            else
                d_dec = d_dec_;
            end            
            
            
            % d_dec: start to decelerate within d_dec of sink

            for i_x = 1:length(vgsink.map(1,:,1))
                x = vgsink.map(1,i_x,1);
                for i_y = 1:length(vgsink.map(2,1,:))
                    y = vgsink.map(2,1,i_y);
                    d = norm([x-p_sink(1), y-p_sink(2)]);
                    if d > d_sink + d_dec
                        v_amp = v_amp_rng(2);
                    elseif d > d_dec
                        v_phase = v_phase0 + d/d_sink*2*pi;
                        v_amp = .5*diff(v_amp_rng)*sin(v_phase) + mean(v_amp_rng);
                    else
                        v_phase = v_phase0 + d_dec/d_sink*2*pi;
                        v_amp0 = .5*diff(v_amp_rng)*sin(v_phase) + mean(v_amp_rng);
                        v_amp = d*v_amp0;
                    end

                    amp_norm = norm(vgsink.vtgt(:,i_x,i_y));
                    vgsink.vtgt(1,i_x,i_y)= v_amp*vgsink.vtgt(1,i_x,i_y)/amp_norm;
                    vgsink.vtgt(2,i_x,i_y) = v_amp*vgsink.vtgt(2,i_x,i_y)/amp_norm;
                end
            end
        end
    end
end
                    
                    
                    
                    
                    