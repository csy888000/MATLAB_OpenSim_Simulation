classdef OsimReflexCtrl
    properties
        dt
        t
        mode
        n_par
        cp_map
        par_space
        ctrl
    end
    
    methods
        function orc = OsimReflexCtrl(mode_, dt_)
            if nargin < 2
                dt = 0.01;
                mode = '3D';
            else
                dt = dt_;
                mode = mode_;
            end

            orc.dt = dt;
            orc.t = 0;
            orc.mode = mode;

%             loco = LocoCtrl();
            if strcmp(orc.mode,'3D')
                orc.n_par = length(LocoCtrl().cp_keys);
                control_dimension = 3;
            elseif strcmp(orc.mode,'2D')
                orc.n_par = 37;
                control_dimension = 2;
            end
%             orc.cp_map = lc.cp_map;
            orc.cp_map = LocoCtrl().cp_map;
            params = ones(1, orc.n_par);
            control_mode = 1;
            orc.ctrl = LocoCtrl(orc.dt, control_mode, control_dimension, params);
            orc.par_space = orc.ctrl.par_space;
        end

    % -----------------------------------------------------------------------------------------------------------------
        function orc = reset(orc)
            orc.ctrl.reset()
        end

    % -----------------------------------------------------------------------------------------------------------------
        function [orc, reflexstim_] = update(orc, obs)
            orc.t = orc.t + orc.dt;
            sensor_data = orc.obs2reflexobs(obs);
            [orc.ctrl, stim] = orc.ctrl.update(sensor_data);
            %for s_l in ['r', 'l']:
            %    str_control_phase = s_l + ':'
            %    for ctrl_ph in orc.ctrl.spinal_control_phase['{}_leg'.format(s_l)]:
            %        if orc.ctrl.spinal_control_phase['{}_leg'.format(s_l)][ctrl_ph]:
            %            str_control_phase = str_control_phase + '  ' + ctrl_ph
            %    print(str_control_phase)
            reflexstim_ =  orc.reflexstim2stim();
        end

    % -----------------------------------------------------------------------------------------------------------------
        function orc = set_control_params(orc, params)
            orc.ctrl = orc.ctrl.set_control_params(params);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function orc = set_control_params_RL(orc, s_leg, params)
            orc.ctrl = orc.ctrl.set_control_params_RL(s_leg, params);
        end

    % -----------------------------------------------------------------------------------------------------------------
        function sensor_data_ = obs2reflexobs(orc, obs_dict)
            % refer to LocoCtrl.s_b_keys and LocoCtrl.s_l_keys
            % coordinate in body frame
            %   [0] x: forward
            %   [1] y: leftward
            %   [2] z: upward

%             sensor_data = {'body':{}, 'r_leg':{}, 'l_leg':{}}
            sensor_data_body = containers.Map();
            sensor_data_r_leg = containers.Map();
            sensor_data_l_leg = containers.Map();
            sensor_data_body('theta') = [obs_dict('pelvis_roll'),... % around local x axis
                                            obs_dict('pelvis_pitch')]; % around local y axis
                                        
            peilvis_vel_ = obs_dict('pelvis_vel');
            sensor_data_body('d_pos') = [peilvis_vel_(1),... % local x (+) forward
                                            peilvis_vel_(2)]; % local y (+) leftward

            sensor_data_body('dtheta') = [peilvis_vel_(5),... % around local x axis
                                            peilvis_vel_(4)]; % around local y axis
            r_leg_ground_reaction_forces_ = obs_dict('r_leg_ground_reaction_forces');
            l_leg_ground_reaction_forces_ = obs_dict('l_leg_ground_reaction_forces');
            sensor_data_r_leg('load_ipsi') = r_leg_ground_reaction_forces_(3);
            sensor_data_l_leg('load_ipsi') = l_leg_ground_reaction_forces_(3);

                
            if sensor_data_r_leg('load_ipsi') > 0.1 
                sensor_data_r_leg('contact_ipsi') = 1;
            else
                sensor_data_r_leg('contact_ipsi') = 0;
            end

            if sensor_data_l_leg('load_ipsi') > 0.1
                sensor_data_r_leg('contact_contra') = 1;
            else
                sensor_data_r_leg('contact_contra') = 0;
            end
            sensor_data_r_leg('load_contra') = sensor_data_l_leg('load_ipsi');

            sensor_data_r_leg('phi_hip') = obs_dict('joint_hip_r') + pi;
            sensor_data_r_leg('phi_knee') = obs_dict('joint_knee_r') + pi;
            sensor_data_r_leg('phi_ankle') = obs_dict('joint_ankle_r') + .5*pi;
            sensor_data_r_leg('dphi_knee') = obs_dict('d_joint_knee_r');

            % alpha = hip - 0.5*knee
            sensor_data_r_leg('alpha') = sensor_data_r_leg('phi_hip') - .5*sensor_data_r_leg('phi_knee');
            dphi_hip = obs_dict('d_joint_hip_r');
            sensor_data_r_leg('dalpha') = dphi_hip - .5*sensor_data_r_leg('dphi_knee');
            sensor_data_r_leg('alpha_f') = -obs_dict('d_joint_hip_abd_r') + .5*pi;

%                 sensor_data[s_leg]['F_RF'] = obs_dict[s_leg]['RF']['f']
%                 sensor_data[s_leg]['F_VAS'] = obs_dict[s_leg]['VAS']['f']
%                 sensor_data[s_leg]['F_GAS'] = obs_dict[s_leg]['GAS']['f']
%                 sensor_data[s_leg]['F_SOL'] = obs_dict[s_leg]['SOL']['f']

            if sensor_data_l_leg('load_ipsi') > 0.1 
                sensor_data_l_leg('contact_ipsi') = 1;
            else
                sensor_data_l_leg('contact_ipsi') = 0;
            end

            if sensor_data_r_leg('load_ipsi') > 0.1
                sensor_data_l_leg('contact_contra') = 1;
            else
                sensor_data_l_leg('contact_contra') = 0;
            end
            sensor_data_l_leg('load_contra') = sensor_data_r_leg('load_ipsi');

            sensor_data_l_leg('phi_hip') = obs_dict('joint_hip_l') + pi;
            sensor_data_l_leg('phi_knee') = obs_dict('joint_knee_l') + pi;
            sensor_data_l_leg('phi_ankle') = obs_dict('joint_ankle_l') + .5*pi;
            sensor_data_l_leg('dphi_knee') = obs_dict('d_joint_knee_l');

            % alpha = hip - 0.5*knee
            sensor_data_l_leg('alpha') = sensor_data_l_leg('phi_hip') - .5*sensor_data_l_leg('phi_knee');
            dphi_hip = obs_dict('d_joint_hip_l');
            sensor_data_l_leg('dalpha') = dphi_hip - .5*sensor_data_l_leg('dphi_knee');
            sensor_data_l_leg('alpha_f') = -obs_dict('d_joint_hip_abd_l') + .5*pi;
            
            
%             
            sensor_data_r_leg('F_RF') = obs_dict('RF_r');
            sensor_data_r_leg('F_VAS') = obs_dict('VAS_r');
            sensor_data_r_leg('F_GAS') = obs_dict('GAS_r');
            sensor_data_r_leg('F_SOL') = obs_dict('SOL_r');
%             
            sensor_data_l_leg('F_RF') = obs_dict('RF_l');
            sensor_data_l_leg('F_VAS') = obs_dict('VAS_l');
            sensor_data_l_leg('F_GAS') = obs_dict('GAS_l');
            sensor_data_l_leg('F_SOL') = obs_dict('SOL_l');            
            
            
            % {'body', 'r_leg', 'l_leg'}
            sensor_data = cell(1,3);
            sensor_data{1} = sensor_data_body;
            sensor_data{2} = sensor_data_r_leg;
            sensor_data{3} = sensor_data_l_leg;
            sensor_data_ = sensor_data;
        end

    % -----------------------------------------------------------------------------------------------------------------
        function stim = reflexstim2stim(orc)
            
            stim_r_leg = orc.ctrl.stim('r_leg');
            stim_l_leg = orc.ctrl.stim('l_leg');
            stim = [stim_r_leg('HFL'),... % (iliopsoas_r)
                    stim_r_leg('GLU'),... % (glut_max_r)
                    stim_r_leg('HAM'),... % (hamstring_r)
                    stim_r_leg('RF'),... % (rect_fem_r)
                    stim_r_leg('VAS'),... % (vasti_r)
                    stim_r_leg('BFSH'),... % (bifemsh_r)
                    stim_r_leg('GAS'),... % (gastroc_r)
                    stim_r_leg('SOL'),... % (soleus_r)
                    stim_r_leg('TA'),... % (tib_ant_r)
                    stim_l_leg('HFL'),... % (iliopsoas_l)
                    stim_l_leg('GLU'),... % (glut_max_l)
                    stim_l_leg('HAM'),... % (hamstring_l)
                    stim_l_leg('RF'),... % (rect_fem_l)
                    stim_l_leg('VAS'),... % (vasti_l)
                    stim_l_leg('BFSH'),... % (bifemsh_l)
                    stim_l_leg('GAS'),... % (gastroc_l)
                    stim_l_leg('SOL'),... % (soleus_l)
                    stim_l_leg('TA')]; % (tib_ant_l)

            if strcmp(orc.mode,'3D')
                stim.insert(0, stim_r_leg('HAB')); % (abd_r)
                stim.insert(1, stim_r_leg('HAD')); % (add_r)
                stim.insert(11, stim_l_leg('HAB')); % (abd_r)
                stim.insert(12, stim_l_leg('HAD')); % (add_r)
            elseif strcmp(orc.mode,'2D')
                Fmax_ABD = 4460.290481;
                Fmax_ADD = 3931.8;

                stim = [0.1, stim];
                stim = [stim(1), .1*Fmax_ADD/Fmax_ABD, stim(2:end)];
                stim = [stim(1:11), .1, stim(12:end)];
                stim = [stim(1:12),.1*Fmax_ADD/Fmax_ABD, stim(13:end)];
            end

%             return stim
        end
    end
end

            
            
            
            
            
            