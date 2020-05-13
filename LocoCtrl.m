% - [x y z] -> [anterior lateral superior]
%         (<-> [posterior medial inferior])


classdef LocoCtrl
    properties
        DEBUG = 0;

        RIGHT = 0; % r_leg
        LEFT = 1; % l_leg

        % (todo) use these when handling angles
        % THETA0 = 0*pi/180 % trunk angle when standing straight
        % S_THETA = 1 % 1: leaning forward > 0; -1: leaning backward > 0
        % HIP0 = 0*pi/180 % hip angle when standing straight
        % S_HIP = 1 % 1: extension > 0; -1: flexion > 0
        % KNEE0 = 0*pi/180 % knee angle when standing straight
        % S_KNEE = 1 % 1: extension > 0; -1: flexion > 0
        % ANKLE0 = 0*pi/180 % ankle angle when standing straight
        % S_ANKLE = 1 % 1: plantar flexion > 0; -1: dorsiflexion > 0

        % muscle names
        m_keys = {'HAB', 'HAD', 'HFL', 'GLU', 'HAM', 'RF', 'VAS', 'BFSH', 'GAS', 'SOL', 'TA'};
        % body sensor data
        s_b_keys = {'theta', 'd_pos', 'dtheta'};
            % theta[0]: around local x axis (pointing anterior)
            % theta[1]: around local y axis (pointing leftward)
            % theta[2]: around local z axis (pointing upward)
            % pos[0]: local x
            % pos[1]: local y
            % pos[2]: local z
        % leg sensor data
        % anglular values follow the Song2015 convention
        s_l_keys = {
            'contact_ipsi', 'contact_contra', 'load_ipsi', 'load_contra', ...
            'alpha', 'alpha_f', 'dalpha', ...
            'phi_hip', 'phi_knee', 'phi_ankle', 'dphi_knee', ...
            'F_RF', 'F_VAS', 'F_GAS', 'F_SOL', ...
            };
        % control states
        cs_keys = {
            'ph_st', ... % leg in stance
            'ph_st_csw', ... % leg in stance ^ contra-leg in swing
            'ph_st_sw0', ... % leg in stance ^ initial swing
            'ph_sw', ... % leg in swing
            'ph_sw_flex_k', ... % leg in swing ^ flex knees
            'ph_sw_hold_k', ... % leg in swing ^ hold knee
            'ph_sw_stop_l', ... % leg in swing ^ stop leg
            'ph_sw_hold_l' ... % leg in swing ^ hold leg
            };
        % control parameters
        cp_keys = {
            'theta_tgt', 'c0', 'cv', 'alpha_delta', ...
            'knee_sw_tgt', 'knee_tgt', 'knee_off_st', 'ankle_tgt', ...
            'HFL_3_PG', 'HFL_3_DG', 'HFL_6_PG', 'HFL_6_DG', 'HFL_10_PG', ...
            'GLU_3_PG', 'GLU_3_DG', 'GLU_6_PG', 'GLU_6_DG', 'GLU_10_PG', ...
            'HAM_3_GLU', 'HAM_9_PG', ...
            'RF_1_FG', 'RF_8_DG_knee', ...
            'VAS_1_FG', 'VAS_2_PG', 'VAS_10_PG', ...
            'BFSH_2_PG', 'BFSH_7_DG_alpha', 'BFSH_7_PG', 'BFSH_8_DG', 'BFSH_8_PG', ...
            'BFSH_9_G_HAM', 'BFSH_9_HAM0', 'BFSH_10_PG', ...
            'GAS_2_FG', ...
            'SOL_1_FG', ...
            'TA_5_PG', 'TA_5_G_SOL', ...
            'theta_tgt_f', 'c0_f', 'cv_f', ...
            'HAB_3_PG', 'HAB_3_DG', 'HAB_6_PG', ...
            'HAD_3_PG', 'HAD_3_DG', 'HAD_6_PG' ...
            };

        par_space = ...
            [[0.0, -1.0, 0.0, 0.0, ...
            -2.0, -90/15, -1.0, 0.0, ...
            0.0, 0.0, 0.0, 0.0, 0.0, ...
            0.0, 0.0, 0.0, 0.0, 0.0, ...
            0.0, 0.0, ...
            0.0, 0.0, ...
            0.0, 0.0, 0.0, ...
            0.0, 0.0, 0.0, 0.0, 0.0, ...
            0.0, 0.0, 0.0, ...
            0.0, ...
            0.0, ...
            0.0, 0.0, ...
            0.0, 0.0, ...
            0.0, 0.0, 0.0, ...
            0.0, 0.0, 0.0]; ...
            [6.0, 3.0, 5.0, 3.0, ...
            3.0, 20/15, 15/10, 3.0, ...
            3.0, 3.0, 3.0, 3.0, 3.0, ...
            3.0, 3.0, 3.0, 3.0, 3.0, ...
            3.0, 3.0, ...
            3.0, 3.0, ...
            3.0, 3.0, 3.0, ...
            3.0, 3.0, 3.0, 3.0, 3.0, ...
            3.0, 3.0, 3.0, ...
            3.0, ...
            3.0, ...
            3.0, 3.0, ...
            2.0, 3.0, ...
            3.0, 3.0, 3.0, ...
            3.0, 3.0, 3.0]];

        m_map
        s_b_map
        s_l_map
        cs_map
        cp_map
        control_mode
        control_dimension
        brain_control_on
        spinal_control_phase
        in_contact
        brain_command
        stim
        n_par
        cp
        sensor_data
    end
    
    methods
    % -----------------------------------------------------------------------------------------------------------------
        function lc = LocoCtrl(TIMESTEP_, control_mode_, control_dimension_, params_)
            if nargin < 3
                control_mode = 1;
                control_dimension = 2;
%                 params = ones(1,length(lc.cp_keys));
                params = ones(1,37);
            else
                TIMESTEP = TIMESTEP_;
                control_mode = control_mode_;
                control_dimension = control_dimension_;
                params = params_;
            end
            if lc.DEBUG
                disp("===========================================")
                disp("locomotion controller created in DEBUG mode")
                disp("===========================================")
            end
            

            lc.m_map = containers.Map(lc.m_keys, 1:(length(lc.m_keys)));
            lc.s_b_map = containers.Map(lc.s_b_keys, 1:(length(lc.s_b_keys)));
            lc.s_l_map = containers.Map(lc.s_l_keys, 1:(length(lc.s_l_keys)));
            lc.cs_map = containers.Map(lc.cs_keys, 1:(length(lc.cs_keys)));
            lc.cp_map = containers.Map(lc.cp_keys, 1:(length(lc.cp_keys)));
        
            lc.control_mode = control_mode;
            % 0: spinal control (no brain control)
            % 1: full control
            lc.control_dimension = control_dimension; % 2D or 3D

            if lc.control_mode == 0
                lc.brain_control_on = 0;
            elseif lc.control_mode == 1
                lc.brain_control_on = 1;
            end

            lc.spinal_control_phase = containers.Map();
            lc.in_contact = containers.Map();
            lc.brain_command = containers.Map();
            lc.stim = containers.Map();
            
            lc.n_par = length(lc.cp_keys);
%             lc.n_par = length(LocoCtrl.cp_keys);
            if lc.control_dimension == 2
                lc.n_par = 37;
                lc.par_space = [lc.par_space(1,1:37), lc.par_space(2,1:37)];
            end
            lc.cp = containers.Map();

            lc = lc.reset(params);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function lc = reset(lc, params_)
            if nargin < 2
                params = [];
            else
                params = params_;
            end
            lc.in_contact('r_leg') = 1;
            lc.in_contact('l_leg') = 0;

            spinal_control_phase_r = containers.Map();
            spinal_control_phase_r('ph_st') = 1;
            spinal_control_phase_r('ph_st_csw') = 0;
            spinal_control_phase_r('ph_st_sw0') = 0;
            spinal_control_phase_r('ph_st_st') = 0;
            spinal_control_phase_r('ph_sw') = 0;
            spinal_control_phase_r('ph_sw_flex_k') = 0;
            spinal_control_phase_r('ph_sw_hold_k') = 0;
            spinal_control_phase_r('ph_sw_stop_l') = 0;
            spinal_control_phase_r('ph_sw_hold_l') = 0;
            lc.spinal_control_phase('r_leg') = spinal_control_phase_r;

            spinal_control_phase_l = containers.Map();
            spinal_control_phase_l('ph_st') = 0;
            spinal_control_phase_l('ph_st_csw') = 0;
            spinal_control_phase_l('ph_st_sw0') = 0;
            spinal_control_phase_l('ph_st_st') = 0;
            spinal_control_phase_l('ph_sw') = 1;
            spinal_control_phase_l('ph_sw_flex_k') = 1;
            spinal_control_phase_l('ph_sw_hold_k') = 0;
            spinal_control_phase_l('ph_sw_stop_l') = 0;
            spinal_control_phase_l('ph_sw_hold_l') = 0;
            lc.spinal_control_phase('l_leg') = spinal_control_phase_l;
            
            for i = 1:length(lc.m_keys)
                stim_r_leg = containers.Map(lc.m_keys, 0.01*ones(1,length(lc.m_keys)));
                stim_l_leg = containers.Map(lc.m_keys, 0.01*ones(1,length(lc.m_keys)));
                lc.stim('r_leg') = stim_r_leg;
                lc.stim('l_leg') = stim_l_leg;
            end
            if ~isempty(params)
                lc.set_control_params(params)
            end
        end

    % -----------------------------------------------------------------------------------------------------------------
        function lc = set_control_params(lc, params)
            if length(params) == lc.n_par
                lc = lc.set_control_params_RL('r_leg', params);
                lc = lc.set_control_params_RL('l_leg', params);
            elseif length(params) == 2*lc.n_par
                lc = lc.set_control_params_RL('r_leg', params(1:lc.n_par));
                lc = lc.set_control_params_RL('l_leg', params(lc.n_par:end));
            else
                disp('error in the number of params!!')
            end
        end

    % -----------------------------------------------------------------------------------------------------------------
        function lc = set_control_params_RL(lc, s_leg, params)
            cp_ = containers.Map();
            cp_map_ = lc.cp_map;

            cp_('theta_tgt') = params(cp_map_('theta_tgt')) *10*pi/180;
            cp_('c0') = params(cp_map_('c0')) *20*pi/180 +55*pi/180;
            cp_('cv') = params(cp_map_('cv')) *2*pi/180;
            cp_('alpha_delta') = params(cp_map_('alpha_delta')) *5*pi/180;
            cp_('knee_sw_tgt') = params(cp_map_('knee_sw_tgt')) *20*pi/180 +120*pi/180;
            cp_('knee_tgt') = params(cp_map_('knee_tgt')) *15*pi/180 +160*pi/180;
            cp_('knee_off_st') = params(cp_map_('knee_off_st')) *10*pi/180 +165*pi/180;
            cp_('ankle_tgt') = params(cp_map_('ankle_tgt')) *20*pi/180 +60*pi/180;

            cp_('HFL_3_PG') = params(cp_map_('HFL_3_PG')) *2.0;
            cp_('HFL_3_DG') = params(cp_map_('HFL_3_DG')) *1.0;
            cp_('HFL_6_PG') = params(cp_map_('HFL_6_PG')) *1.0;
            cp_('HFL_6_DG') = params(cp_map_('HFL_6_DG')) *.1;
            cp_('HFL_10_PG') = params(cp_map_('HFL_10_PG')) *1.0;

            cp_('GLU_3_PG') = params(cp_map_('GLU_3_PG')) *2.0;
            cp_('GLU_3_DG') = params(cp_map_('GLU_3_DG')) *0.5;
            cp_('GLU_6_PG') = params(cp_map_('GLU_6_PG')) *1.0;
            cp_('GLU_6_DG') = params(cp_map_('GLU_6_DG')) *0.1;
            cp_('GLU_10_PG') = params(cp_map_('GLU_10_PG')) *.5;

            cp_('HAM_3_GLU') = params(cp_map_('HAM_3_GLU')) *1.0;
            cp_('HAM_9_PG') = params(cp_map_('HAM_9_PG')) *2.0;

            cp_('RF_1_FG') = params(cp_map_('RF_1_FG')) *0.3;
            cp_('RF_8_DG_knee') = params(cp_map_('RF_8_DG_knee')) *0.1;

            cp_('VAS_1_FG') = params(cp_map_('VAS_1_FG')) *1.0;
            cp_('VAS_2_PG') = params(cp_map_('VAS_2_PG')) *2.0;
            cp_('VAS_10_PG') = params(cp_map_('VAS_10_PG')) *.3;

            cp_('BFSH_2_PG') = params(cp_map_('BFSH_2_PG')) *2.0;
            cp_('BFSH_7_DG_alpha') = params(cp_map_('BFSH_7_DG_alpha')) *0.2;
            cp_('BFSH_7_PG') = params(cp_map_('BFSH_7_PG')) *2.0;
            cp_('BFSH_8_DG') = params(cp_map_('BFSH_8_DG')) *1.0;
            cp_('BFSH_8_PG') = params(cp_map_('BFSH_8_DG')) *1.0;
            cp_('BFSH_9_G_HAM') = params(cp_map_('BFSH_9_G_HAM')) *2.0;
            cp_('BFSH_9_HAM0') = params(cp_map_('BFSH_9_HAM0')) *0.3;
            cp_('BFSH_10_PG') = params(cp_map_('BFSH_10_PG')) *2.0;

            cp_('GAS_2_FG') = params(cp_map_('GAS_2_FG')) *1.2;

            cp_('SOL_1_FG') = params(cp_map_('SOL_1_FG')) *1.2;

            cp_('TA_5_PG') = params(cp_map_('TA_5_PG')) *2.0;
            cp_('TA_5_G_SOL') = params(cp_map_('TA_5_G_SOL')) *0.5;

            if lc.control_dimension == 3
                if length(params) ~= 46
                    disp('error in the number of params!!')
                end
                cp_('theta_tgt_f') = params(cp_map_('theta_tgt_f')) *5.0*pi/180;
                cp_('c0_f') = params(cp_map_('c0_f')) *20*pi/180 + 60*pi/180;
                cp_('cv_f') = params(cp_map_('cv_f')) *10*pi/180;
                cp_('HAB_3_PG') = params(cp_map_('HAB_3_PG')) *10.0;
                cp_('HAB_3_DG') = params(cp_map_('HAB_3_DG')) *1;
                cp_('HAB_6_PG') = params(cp_map_('HAB_6_PG')) *2.0;
                cp_('HAD_3_PG') = params(cp_map_('HAD_3_PG')) *2.0;
                cp_('HAD_3_DG') = params(cp_map_('HAD_3_DG')) *0.3;
                cp_('HAD_6_PG') = params(cp_map_('HAD_6_PG')) *2.0;
            elseif lc.control_dimension == 2
                if length(params) ~= 37
                    disp('error in the number of params!!')
                end
            end

            lc.cp(s_leg) = cp_;
        end

    % -----------------------------------------------------------------------------------------------------------------
        function [lc, stim] = update(lc, sensor_data)
            lc.sensor_data = sensor_data;

            if lc.brain_control_on
                % update lc.brain_command
                lc = lc.brain_control(sensor_data);
            end

            % updates lc.stim
            lc = lc.spinal_control(sensor_data);
            stim_r_leg = lc.stim('r_leg');
            stim_l_leg = lc.stim('l_leg');
            
            stim = [stim_r_leg('HFL'), stim_r_leg('GLU'), ...
                stim_r_leg('HAM'), stim_r_leg('RF'), ...
                stim_r_leg('VAS'), stim_r_leg('BFSH'), ...
                stim_r_leg('GAS'), stim_r_leg('SOL'), ...
                stim_r_leg('TA'), ...
                stim_l_leg('HFL'), stim_l_leg('GLU'), ...
                stim_l_leg('HAM'), stim_l_leg('RF'), ...
                stim_l_leg('VAS'), stim_l_leg('BFSH'), ...
                stim_l_leg('GAS'), stim_l_leg('SOL'), ...
                stim_l_leg('TA') ...
                ];
            % todo: lc._flaten(lc.stim)
        end

    % -----------------------------------------------------------------------------------------------------------------
        function lc = brain_control(lc, sensor_data_)
            if nargin < 2
                sensor_data = 0;
            else
                sensor_data = sensor_data_;
            end
            
%             s_b = sensor_data('body');
%             s_data_r_leg = sensor_data('r_leg');
%             s_data_l_leg = sensor_data('l_leg');
            
            s_b = sensor_data{1};
            s_data_r_leg = sensor_data{2};
            s_data_l_leg = sensor_data{3};
            
            s_b_d_pos = s_b('d_pos');
            s_b_theta = s_b('theta');
            cp_ = lc.cp;

%             lc.brain_command('r_leg') = containers.Map();
%             lc.brain_command('l_leg') = containers.Map();
            brain_command_r_leg = containers.Map();
            brain_command_l_leg = containers.Map();
            
            leg_ = {'r_leg', 'l_leg'};
               
            for i = 1:length(leg_)
                s_leg = leg_{i};
                cp_s_leg = cp_(s_leg);
                brain_command_s_leg = containers.Map();
                
                if lc.control_dimension == 3
                    
                    brain_command_s_leg = lc.brain_command(s_leg);
                    
                    brain_command_s_leg('theta_tgt_f') = cp_s_leg('theta_tgt_f');
                    if strcmp(s_leg,'r_leg')
                        sign_frontral = 1;
                    else
                        sign_frontral = -1;
                    end
                    alpha_tgt_global_frontal = cp_s_leg('c0_f') + sign_frontral*cp_s_leg('cv_f')*s_b_d_pos(2);
                    theta_f = sign_frontral*s_b_theta(1);
                    brain_command_s_leg('alpha_tgt_f') = alpha_tgt_global_frontal - theta_f;
                end
            
                    

                brain_command_s_leg('theta_tgt') = cp_s_leg('theta_tgt');

                alpha_tgt_global = cp_s_leg('c0') - cp_s_leg('cv')*s_b_d_pos(1);
                brain_command_s_leg('alpha_tgt') = alpha_tgt_global - s_b_theta(2);
                brain_command_s_leg('alpha_delta') = cp_s_leg('alpha_delta');
                brain_command_s_leg('knee_sw_tgt') = cp_s_leg('knee_sw_tgt');
                brain_command_s_leg('knee_tgt') = cp_s_leg('knee_tgt');
                brain_command_s_leg('knee_off_st') = cp_s_leg('knee_off_st');
                brain_command_s_leg('ankle_tgt') = cp_s_leg('ankle_tgt');
                % alpha = hip - 0.5*knee
                brain_command_s_leg('hip_tgt') = brain_command_s_leg('alpha_tgt') + 0.5*brain_command_s_leg('knee_tgt');
                
                if i == 1
                    brain_command_r_leg = brain_command_s_leg;
                else
                    brain_command_l_leg = brain_command_s_leg;
                end            
            end
            
            % select which leg to swing
%             brain_command_r_leg = lc.brain_command('r_leg');
%             brain_command_l_leg = lc.brain_command('l_leg');
            
            brain_command_r_leg('swing_init') = 0;
            brain_command_l_leg('swing_init') = 0;
            if s_data_r_leg('contact_ipsi') && s_data_l_leg('contact_ipsi')
                r_delta_alpha = s_data_r_leg('alpha') - brain_command_r_leg('alpha_tgt');
                l_delta_alpha = s_data_l_leg('alpha') - brain_command_l_leg('alpha_tgt');
                if r_delta_alpha > l_delta_alpha
                    brain_command_r_leg('swing_init') = 1;
                else
                    brain_command_l_leg('swing_init') = 1;
                end
            end
            
            lc.brain_command('r_leg') = brain_command_r_leg;
            lc.brain_command('l_leg') = brain_command_l_leg;
            
            
        end
        
        

    % -----------------------------------------------------------------------------------------------------------------
        function lc = spinal_control(lc, sensor_data)
            leg_ = {'r_leg', 'l_leg'};
            for i = 1:length(leg_)
                s_leg = leg_{i};
                lc = lc.updatespinal_control_phase(s_leg, sensor_data);
                stim_ = lc.stim(s_leg);
                stim_ = lc.spinal_control_leg(s_leg, sensor_data);
                lc.stim(s_leg) = stim_;
            end
        end

    % -----------------------------------------------------------------------------------------------------------------
        function lc = updatespinal_control_phase(lc, s_leg, sensor_data)
            if isequal(s_leg, 'r_leg')
                s_l = sensor_data{2};
            elseif isequal(s_leg,'l_leg')
                s_l = sensor_data{3};
            end
            
            brain_command_s_leg = lc.brain_command(s_leg);
            
            alpha_tgt = brain_command_s_leg('alpha_tgt');
            alpha_delta = brain_command_s_leg('alpha_delta');
            knee_sw_tgt = brain_command_s_leg('knee_sw_tgt');
            
            spinal_ctrl_phase_s_leg = lc.spinal_control_phase(s_leg);
            % when foot touches ground
            if ~lc.in_contact(s_leg) && s_l('contact_ipsi')
                % initiate stance control
                spinal_ctrl_phase_s_leg('ph_st') = 1;
                % swing control off
                spinal_ctrl_phase_s_leg('ph_sw') = 0;
                spinal_ctrl_phase_s_leg('ph_sw_flex_k') = 0;
                spinal_ctrl_phase_s_leg('ph_sw_hold_k') = 0;
                spinal_ctrl_phase_s_leg('ph_sw_stop_l') = 0;
                spinal_ctrl_phase_s_leg('ph_sw_hold_l') = 0;
            end

            % during stance control
            if spinal_ctrl_phase_s_leg('ph_st')
                % contra-leg in swing (single stance phase)
                spinal_ctrl_phase_s_leg('ph_st_csw') = ~s_l('contact_contra');
                % initiate swing
                spinal_ctrl_phase_s_leg('ph_st_sw0') = brain_command_s_leg('swing_init');
                % do not initiate swing
                spinal_ctrl_phase_s_leg('ph_st_st') = ~spinal_ctrl_phase_s_leg('ph_st_sw0');
            end

            % when foot loses contact
            if lc.in_contact(s_leg) && ~s_l('contact_ipsi')
                % stance control off
                spinal_ctrl_phase_s_leg('ph_st') = 0;
                spinal_ctrl_phase_s_leg('ph_st_csw') = 0;
                spinal_ctrl_phase_s_leg('ph_st_sw0') = 0;
                spinal_ctrl_phase_s_leg('ph_st_st') = 0;
                % initiate swing control
                spinal_ctrl_phase_s_leg('ph_sw') = 1;
                % flex knee
                spinal_ctrl_phase_s_leg('ph_sw_flex_k') = 1;
            end

            % during swing control
            if spinal_ctrl_phase_s_leg('ph_sw')
                if spinal_ctrl_phase_s_leg('ph_sw_flex_k')
                    if s_l('phi_knee') < knee_sw_tgt % knee flexed
                        spinal_ctrl_phase_s_leg('ph_sw_flex_k') = 0;
                        % hold knee
                        spinal_ctrl_phase_s_leg('ph_sw_hold_k') = 1;
                    end
                else
                    if spinal_ctrl_phase_s_leg('ph_sw_hold_k')
                        if s_l('alpha') < alpha_tgt % leg swung enough
                            spinal_ctrl_phase_s_leg('ph_sw_hold_k') = 0;
                        end
                    end
                    if s_l('alpha') < alpha_tgt + alpha_delta % leg swung enough
                        % stop leg
                        spinal_ctrl_phase_s_leg('ph_sw_stop_l') = 1;
                    end
                    if spinal_ctrl_phase_s_leg('ph_sw_stop_l') && s_l('dalpha') > 0 % leg started to retract
                        % hold leg
                        spinal_ctrl_phase_s_leg('ph_sw_hold_l') = 1;
                    end
                end
            end

            lc.in_contact(s_leg) = s_l('contact_ipsi');
            
            lc.spinal_control_phase(s_leg) = spinal_ctrl_phase_s_leg;
                        
        end

    % -----------------------------------------------------------------------------------------------------------------
        function stim = spinal_control_leg(lc, s_leg, sensor_data)
            if isequal(s_leg, 'r_leg')
                s_l = sensor_data{2};
            elseif isequal(s_leg,'l_leg')
                s_l = sensor_data{3};
            end            
            
            
%             s_l = sensor_data(s_leg);
%             s_b = sensor_data('body');
            s_b = sensor_data{1};
            cp_ = lc.cp(s_leg);
            
            spinal_ctrl_phase_s_leg = lc.spinal_control_phase(s_leg);
            
            ph_st = spinal_ctrl_phase_s_leg('ph_st');
            ph_st_csw = spinal_ctrl_phase_s_leg('ph_st_csw');
            ph_st_sw0 = spinal_ctrl_phase_s_leg('ph_st_sw0');
            ph_st_st = spinal_ctrl_phase_s_leg('ph_st_st');
            ph_sw = spinal_ctrl_phase_s_leg('ph_sw');
            ph_sw_flex_k = spinal_ctrl_phase_s_leg('ph_sw_flex_k');
            ph_sw_hold_k = spinal_ctrl_phase_s_leg('ph_sw_hold_k');
            ph_sw_stop_l = spinal_ctrl_phase_s_leg('ph_sw_stop_l');
            ph_sw_hold_l = spinal_ctrl_phase_s_leg('ph_sw_hold_l');
            
            s_b_theta = s_b('theta');
            s_b_dtheta = s_b('dtheta');
            theta = s_b_theta(2);
            dtheta = s_b_dtheta(2);
            
            if strcmp(s_leg,'r_leg')
                sign_frontral = 1;
            else
                sign_frontral = -1;
            end
            theta_f = sign_frontral*s_b_theta(1);
            dtheta_f = sign_frontral*s_b_dtheta(1);
            
            brain_command_s_leg = lc.brain_command(s_leg);
            
            theta_tgt = brain_command_s_leg('theta_tgt');
            alpha_tgt = brain_command_s_leg('alpha_tgt');
            alpha_delta = brain_command_s_leg('alpha_delta');
            hip_tgt = brain_command_s_leg('hip_tgt');
            knee_tgt = brain_command_s_leg('knee_tgt');
            knee_sw_tgt = brain_command_s_leg('knee_sw_tgt');
            knee_off_st = brain_command_s_leg('knee_off_st');
            ankle_tgt = brain_command_s_leg('ankle_tgt');

            stim = containers.Map();
            pre_stim = 0.01;

            if lc.control_dimension == 3
                theta_tgt_f = brain_command_s_leg('theta_tgt_f');
                alpha_tgt_f = brain_command_s_leg('alpha_tgt_f');
                    
                S_HAB_3 = ph_st*s_l('load_ipsi')*max( - cp_('HAB_3_PG')*(theta_f-theta_tgt_f) - cp_('HAB_3_DG')*dtheta_f, 0);
                    
                S_HAB_6 = (ph_st_sw0*s_l('load_contra') + ph_sw)*max( cp_('HAB_6_PG')*(s_l('alpha_f') - alpha_tgt_f), 0);
                stim('HAB') = S_HAB_3 + S_HAB_6;

                S_HAD_3 = ph_st*s_l('load_ipsi')*max( cp_('HAD_3_PG')*(theta_f-theta_tgt_f) + cp_('HAD_3_DG')*dtheta_f, 0);
                S_HAD_6 = (ph_st_sw0*s_l('load_contra') + ph_sw)*max( - cp_('HAD_6_PG')*(s_l('alpha_f') - alpha_tgt_f), 0);
                stim('HAD') = S_HAD_3 + S_HAD_6;
            end

            S_HFL_3 = ph_st*s_l('load_ipsi')*max( - cp_('HFL_3_PG')*(theta-theta_tgt) - cp_('HFL_3_DG')*dtheta, 0);
            S_HFL_6 = (ph_st_sw0*s_l('load_contra') + ph_sw)*max( cp_('HFL_6_PG')*(s_l('alpha')-alpha_tgt) + cp_('HFL_6_DG')*s_l('dalpha'), 0);
            S_HFL_10 = ph_sw_hold_l*max( cp_('HFL_10_PG')*(s_l('phi_hip') - hip_tgt) , 0);
            stim('HFL') = pre_stim + S_HFL_3 + S_HFL_6 + S_HFL_10;

            S_GLU_3 = ph_st*s_l('load_ipsi')*max( cp_('GLU_3_PG')*(theta-theta_tgt) + cp_('GLU_3_DG')*dtheta, 0);
            S_GLU_6 = (ph_st_sw0*s_l('load_contra') + ph_sw)*max( - cp_('GLU_6_PG')*(s_l('alpha')-alpha_tgt) - cp_('GLU_6_DG')*s_l('dalpha'), 0);
            S_GLU_10 = ph_sw_hold_l*max( - cp_('GLU_10_PG')*(s_l('phi_hip') - hip_tgt), 0);
            stim('GLU') = pre_stim + S_GLU_3 + S_GLU_6 + S_GLU_10;

            S_HAM_3 = cp_('HAM_3_GLU')*S_GLU_3;
            S_HAM_9 = ph_sw_stop_l*max( - cp_('HAM_9_PG')*(s_l('alpha') - (alpha_tgt + alpha_delta)), 0);
            stim('HAM') = pre_stim + S_HAM_3 + S_HAM_9;

            S_RF_1 = (ph_st_st + ph_st_sw0*(1-s_l('load_contra')))*max( cp_('RF_1_FG')*s_l('F_RF'), 0);
            S_RF_8 = ph_sw_hold_k*max( - cp_('RF_8_DG_knee')*s_l('dphi_knee') , 0);
            stim('RF') = pre_stim + S_RF_1 + S_RF_8;

            S_VAS_1 = (ph_st_st + ph_st_sw0*(1-s_l('load_contra')))*max( cp_('VAS_1_FG')*s_l('F_VAS'), 0);
            S_VAS_2 = -(ph_st_st + ph_st_sw0*(1-s_l('load_contra')))*max( cp_('VAS_2_PG')*(s_l('phi_knee') - knee_off_st), 0);
            S_VAS_10 = ph_sw_hold_l*max( - cp_('VAS_10_PG')*(s_l('phi_knee') - knee_tgt), 0);
            stim('VAS') = pre_stim + S_VAS_1 + S_VAS_2 + S_VAS_10;

            S_BFSH_2 = (ph_st_st + ph_st_sw0*(1-s_l('load_contra')))*max( cp_('BFSH_2_PG')*(s_l('phi_knee') - knee_off_st), 0);
            S_BFSH_7 = (ph_st_sw0*(s_l('load_contra')) + ph_sw_flex_k)*max( - cp_('BFSH_7_DG_alpha')*s_l('dalpha') + cp_('BFSH_7_PG')*(s_l('phi_knee') - knee_sw_tgt), 0);
            S_BFSH_8 = ph_sw_hold_k*max( cp_('BFSH_8_DG')*(s_l('dphi_knee') - knee_off_st)*cp_('BFSH_8_PG')*(s_l('alpha') - alpha_tgt), 0);
            S_BFSH_9 = max( cp_('BFSH_9_G_HAM')*(S_HAM_9 - cp_('BFSH_9_HAM0')), 0);
            S_BFSH_10 = ph_sw_hold_l*max( cp_('BFSH_10_PG')*(s_l('phi_knee') - knee_tgt), 0);
            stim('BFSH') = pre_stim + S_BFSH_2 + S_BFSH_7 + S_BFSH_8 + S_BFSH_9 + S_BFSH_10;

            S_GAS_2 = ph_st*max( cp_('GAS_2_FG')*s_l('F_GAS'), 0);
            stim('GAS') = pre_stim + S_GAS_2;
            S_SOL_1 = ph_st*max( cp_('SOL_1_FG')*s_l('F_SOL'), 0);
            stim('SOL') = pre_stim + S_SOL_1;

            S_TA_5 = max( cp_('TA_5_PG')*(s_l('phi_ankle') - ankle_tgt), 0);
            S_TA_5_st = -ph_st*max( cp_('TA_5_G_SOL')*S_SOL_1, 0);
            stim('TA') = pre_stim + S_TA_5 + S_TA_5_st;

            for i = 1:length(stim)
                MUS = stim.keys;
                mus = stim.values;
                m_key = MUS{i};
                m_value = mus{i};
                m_value(m_value>1.0) = 1.0;
                m_value(m_value<0.01) = 0.01;
                stim(MUS{i}) = m_value;
            end


        end
    end
end


                
            