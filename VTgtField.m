classdef VTgtField
    properties
        nn_get = [11, 11]; % vtgt_field_local data is nn_get*nn_get = 121

        ver = containers.Map();
        % v00: constant forward velocities
    %     ver('ver00') = {}

        ver00 = containers.Map();


        % v01: consecutive sinks forward for walking
    %     ver('ver01') = {}
        ver01 = containers.Map();


        % v02: consecutive sinks for walking (-90 < th < 90)
    %     ver('ver02') = {}
        ver02 = containers.Map();


        % v03: consecutive sinks for walking (-180 < th < 180)
        %       vs. v02: larger range; n_target = 2
    %     ver('ver03') = {}
        ver03 = containers.Map();


        vtgt_space = [ repmat([-10], 1, 2*11*11); repmat([10], 1, 2*11*11) ];
        rng_xy0
        v_amp_rng
        rng_p_sink_r_th
        r_target
        rng_get
        res_map
        n_new_target
        dt
        visualize
        dt_visualize
        di_visualize
        t
        i
        pose_agent
        vtgt_obj
        p_sink
        i_target
        t_target
        res_get
        rng_xy
        path_th
        t0_target
    end
    
% -----------------------------------------------------------------------------------------------------------------
    methods
        function vg = VTgtField(visualize_, version_, dt_, dt_visualize_)
            if nargin < 4
                visualize = true;
                version = 1;
                dt=.01;
                dt_visualize=0.5;
            else
                visualize = visualize_;
                version = version_;
                dt=dt_;
                dt_visualize=dt_visualize_;
            end
            
            
            vg.ver00('res_map') = [2, 2];
            vg.ver00('rng_xy0') = [[-20, 20]; [-20, 20]];
            vg.ver00('rng_get') = [[-5, 5]; [-5, 5]];
            vg.ver00('v_amp_rng') = {};
            vg.ver00('rng_p_sink_r_th') = {};
            vg.ver00('r_target') = {};
            vg.ver00('v_amp') = [1.4, 0];
            vg.ver00('n_new_target') = 1;            

            vg.ver01('res_map') = [2, 2];
            vg.ver01('rng_xy0') = [[-20, 20]; [-20, 20]];
            vg.ver01('rng_get') = [[-5, 5]; [-5, 5]];
            vg.ver01('v_amp_rng') = [.8, 1.8];
            vg.ver01('rng_p_sink_r_th') = [[5, 7]; [0, 0]];
            vg.ver01('r_target') = .3;
            vg.ver01('n_new_target') = Inf;            

            vg.ver02('res_map') = [2, 2];
            vg.ver02('rng_xy0') = [[-20, 20]; [-20, 20]];
            vg.ver02('rng_get') = [[-5, 5]; [-5, 5]];
            vg.ver02('v_amp_rng') = [.8, 1.8];
            vg.ver02('rng_p_sink_r_th') = [[5, 7]; [-90*pi/180, 90*pi/180]];
            vg.ver02('r_target') = .3;
            vg.ver02('n_new_target') = Inf;           
            
            
            vg.ver03('res_map') = [2, 2];
            vg.ver03('rng_xy0') = [[-20, 20]; [-20, 20]];
            vg.ver03('rng_get') = [[-5, 5]; [-5, 5]];
            vg.ver03('v_amp_rng') = [.8, 1.8];
            vg.ver03('rng_p_sink_r_th') = [[5, 7]; [-180*pi/180, 180*pi/180]];
            vg.ver03('r_target') = .3;
            vg.ver03('n_new_target') = 2;


            vg.dt = dt;
            vg.visualize = visualize;
            vg.dt_visualize = dt_visualize;
            vg.di_visualize = round(dt_visualize/dt);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vg = reset(vg, version_, seed_, pose_agent_)
            
            if nargin < 4
                version = 1;
                seed = [];
                pose_agent = [0, 0, 0];
            else
                version = version_;
                seed = seed_;
                pose_agent=pose_agent_;
            end
            
                        
            vg.t = 0;
            vg.i = 0;
            vg.i_target = 1;
            vg.t_target = 0;

%             if version not in [0, 1, 2, 3]:
%                 raise ValueError("vtgt version should be in [0, 1, 2].")
            vg.ver('version') = version;
            % set parameters
            if version == 0
                s_ver = vg.ver00;
            elseif version == 1
                s_ver = vg.ver01;
            elseif version == 2
                s_ver = vg.ver02;
            elseif version == 3
                s_ver = vg.ver03;
            end
                
            
            
            
%             s_ver = 'ver{}'.format(str(version).rjust(2,'0'))
            vg.rng_xy0 = s_ver('rng_xy0');
            vg.v_amp_rng = s_ver('v_amp_rng');
            vg.rng_p_sink_r_th = s_ver('rng_p_sink_r_th');
            vg.r_target = s_ver('r_target');
            vg.rng_get = s_ver('rng_get');
            vg.res_map = s_ver('res_map');
            vg.n_new_target = s_ver('n_new_target');

            vg.res_get = [(vg.rng_get(1,2)-vg.rng_get(1,1)+1)/vg.nn_get(1),...
                          (vg.rng_get(2,2)-vg.rng_get(2,1)+1)/vg.nn_get(2)];
            vg.pose_agent = pose_agent;
            vg.rng_xy = (vg.pose_agent(1:2) + (vg.rng_xy0)')';

            if vg.ver('version') == 0
                % map coordinate and vtgt
                v_tgt=vg.ver00('v_amp');
                rng_xy=vg.rng_x;
                res_map_=vg.res_map;
                rng_get_=vg.rng_get;
                res_get=vg.res_get;
                vg.vtgt_obj = VTgtConst(v_tgt, rng_xy, res_map_, rng_get_, res_get);
                v_tgt_=vg.ver00('v_amp');
                vg = vg.create_vtgt_const(v_tgt_);
            elseif ismember(vg.ver('version'),[1, 2, 3])
                % map coordinate and vtgt
                rng_xy=vg.rng_xy;
                res_map_=vg.res_map;
                rng_get_=vg.rng_get;
                res_get=vg.res_get;
                vg.vtgt_obj = VTgtSink(rng_xy, res_map_, rng_get_, res_get);
                if seed
                    rng('default')
                    rand(seed)
                end

                % create first sink
                del_p_sink_r = (vg.rng_p_sink_r_th(1,2)-vg.rng_p_sink_r_th(1,1)).*rand(1) + vg.rng_p_sink_r_th(1,2);
                del_p_sink_th = (vg.rng_p_sink_r_th(2,2)-vg.rng_p_sink_r_th(2,1)).*rand(1) + vg.rng_p_sink_r_th(2,2);
                del_p_sink_x = cos(del_p_sink_th)*del_p_sink_r;
                del_p_sink_y = sin(del_p_sink_th)*del_p_sink_r;
                vg.path_th = del_p_sink_th;
                vg.p_sink = vg.pose_agent(1:2) + [del_p_sink_x, del_p_sink_y];
                vg = vg.create_vtgt_sink(vg.v_amp_rng);
            end
            
            
            if vg.visualize
%                 import matplotlib.pyplot as plt
%                 if hasattr(vg, 'vis'):
%                     vg.vis['plt'].close(vg.vis['hndl'])
%                 end
%                 vg.vis = containers.Map();
%                 vg.vis['plt'] = plt
%                 vg.vis['hndl'], vg.vis['axes'] = vg.vis['plt'].subplots(2,1, figsize=(5, 6))
%                 X = vg.vtgt_obj.map[0]
%                 Y = vg.vtgt_obj.map[1]
%                 U = vg.vtgt_obj.vtgt[0]
%                 V = vg.vtgt_obj.vtgt[1]
%                 R = np.sqrt(U**2 + V**2)
%                 vg.vis['q0'] = vg.vis['axes'][0].quiver(X, Y, U, V, R)
%                 vg.vis['axes'][0].axis('equal')
%                 vg.vis['axes'][0].set_title('v$_{tgt}$ (global)')
%                 vg.vis['axes'][0].set_xlabel('x')
%                 vg.vis['axes'][0].set_ylabel('y')
% 
%                 v_tgt_field = vg.vtgt_obj.get_vtgt_field_local(pose_agent)
%                 X, Y = vg.vtgt_obj._generate_grid(vg.vtgt_obj.rng_get, vg.vtgt_obj.res_get)
%                 U = v_tgt_field[0]
%                 V = v_tgt_field[1]
%                 R = np.sqrt(U**2 + V**2)
%                 vg.vis['q1'] = vg.vis['axes'][1].quiver(X, Y, U, V, R)
%                 vg.vis['axes'][1].axis('equal')
%                 vg.vis['axes'][1].set_title('v$_{tgt}$ (body)')
%                 vg.vis['axes'][1].set_xlabel('forward')
%                 vg.vis['axes'][1].set_ylabel('leftward')
% 
%                 vg.vis['plt'].tight_layout()
%                 vg.vis['plt'].pause(0.0001)
            end
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vg = create_vtgt_const(vg, v_tgt)
            vg.vtgt_obj.create_vtgt_const(v_tgt);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vg = create_vtgt_sink(vg, v_amp_rng)
            d_sink = norm(vg.p_sink - vg.pose_agent(1:2));
            v_phase0 = (pi+pi).*rand(1) + -pi;
            vg.t0_target = (4-2).*rand(1) + 2;
%             v_phase0=v_phase0;
            vg.vtgt_obj = vg.vtgt_obj.create_vtgt_sink(vg.p_sink, d_sink, v_amp_rng, v_phase0);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function [vg, v_tgt_field, flag_target_achieved] = update(vg, pose)
            vg.t = vg.t + vg.dt;
            vg.i = vg.i + 1;

            vg.pose_agent = pose;

            if ~ isprop(vg, 'p_sink')
                flag_target_achieved = 0;
            else
                if norm(vg.p_sink - vg.pose_agent(1:2)) < vg.r_target
                    vg.t_target = vg.t_target + vg.dt;
                else % reset t_target if agent goes out of 
                    vg.t_target = 0;
                end

                flag_target_achieved = 0;
                if (vg.t_target > vg.t0_target... % stayed at the target
                    && vg.i_target <= vg.n_new_target) % on a new target
                    if vg.i_target < vg.n_new_target % if ... create new target
                        del_p_sink_r = (vg.rng_p_sink_r_th(1,2)-vg.rng_p_sink_r_th(1,1)).*rand(1) + vg.rng_p_sink_r_th(1,1);
                        del_p_sink_th = (vg.rng_p_sink_r_th(2,2)-vg.rng_p_sink_r_th(2,1)).*rand(1000,1) + vg.rng_p_sink_r_th(2,1);
                        vg.path_th = vg.path_th + del_p_sink_th;
                        del_p_sink_x = cos(vg.path_th)*del_p_sink_r;
                        del_p_sink_y = sin(vg.path_th)*del_p_sink_r;
                        vg.p_sink = vg.p_sink + [del_p_sink_x, del_p_sink_y];
                        vg.rng_xy = (vg.pose_agent(1:2) + (vg.rng_xy0)')';
                        vg.vtgt_obj.create_map(vg.rng_xy)
                        vg.create_vtgt_sink(vg.v_amp_rng)
                    end
                    vg.i_target = vg.i_target + 1;
                    vg.t_target = 0;
                    flag_target_achieved = 1;
                end
            end
                    

            v_tgt_field = vg.vtgt_obj.get_vtgt_field_local(pose);
            if vg.visualize
                if flag_target_achieved
%                     vg.vis['q0'].remove()
%                     X = vg.vtgt_obj.map[0]
%                     Y = vg.vtgt_obj.map[1]
%                     U = vg.vtgt_obj.vtgt[0]
%                     V = vg.vtgt_obj.vtgt[1]
%                     R = np.sqrt(U**2 + V**2)
%                     vg.vis['q0'] = vg.vis['axes'][0].quiver(X, Y, U, V, R)
%                     vg.vis['axes'][0].axis('equal')
                end


                if vg.di_visualize == 1 || vg.i %vg.di_visualize==1 or vg.t == vg.dt:
%                     vg.vis['axes'][0].plot(pose[0], pose[1], 'k.')
% 
%                     X, Y = vg.vtgt_obj._generate_grid(vg.vtgt_obj.rng_get, vg.vtgt_obj.res_get)
%                     U = v_tgt_field[0]
%                     V = v_tgt_field[1]
%                     R = np.sqrt(U**2 + V**2)
%                     vg.vis['q1'].remove()
%                     vg.vis['q1'] = vg.vis['axes'][1].quiver(X, Y, U, V, R)
%                     vg.vis['axes'][1].plot(0, 0, 'k.')
%                     vg.vis['axes'][1].axis('equal')
% 
%                     vg.vis['plt'].pause(0.0001)
                end
            end

%             return v_tgt_field, flag_target_achieved
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vtgt_ = get_vtgt(vg, xy)
            vtgt_ = vg.vtgt_obj.get_vtgt(xy);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function vtgt_field_local_ = get_vtgt_field_local(vg, pose)
            vtgt_field_local_ = vg.vtgt_obj.get_vtgt_field_local(pose);
        end
        
    end
end


