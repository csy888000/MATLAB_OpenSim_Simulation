classdef L2M2019Env < OsimEnv
    % to change later:
    % muscle v: normalize by max_contraction_velocity, 15 lopt / s
    properties
        model = '2D'
        model_paths
%         model_path
        % from gait14dof22musc_20170320.osim
        MASS = 75.16460000000001 % 11.777 + 2*(9.3014 + 3.7075 + 0.1 + 1.25 + 0.2166) + 34.2366
        G = 9.80665 % from gait1dof22muscle

        LENGTH0 = 1 % leg length

        footstep = containers.Map({'n','new','r_contact','l_contact'},{0, false, 0, 0});

        dict_muscle =  containers.Map({'abd', 'add', 'iliopsoas', 'glut_max', 'hamstrings',...
            'rect_fem', 'vasti', 'bifemsh', 'gastroc', 'soleus', 'tib_ant'},...
            {'HAB', 'HAD', 'HFL', 'GLU', 'HAM', 'RF', 'VAS', 'BFSH', 'GAS', 'SOL', 'TA'});

        act2mus = [0, 1, 4, 7, 3, 2, 5, 6, 8, 9, 10, 11, 12, 15, 18, 14, 13, 16, 17, 19, 20, 21]
        % maps muscle order in action to muscle order in gait14dof22musc_20170320.osim
        % muscle order in action
        %    HAB, HAD, HFL, GLU, HAM, RF, VAS, BFSH, GAS, SOL, TA 
        % muscle order in gait14dof22musc_20170320.osim
        %    HAB, HAD, HAM, BFSH, GLU, HFL, RF, VAS, GAS, SOL, TA
        %    or abd, add, hamstrings, bifemsh, glut_max, iliopsoas, rect_fem, vasti, gastroc, soleus, tib_ant

        INIT_POSE = [
            0; % forward speed
            0; % rightward speed
            0.94; % pelvis height
            0*pi/180; % trunk lean
            0*pi/180; % [right] hip adduct
            0*pi/180; % hip flex
            0*pi/180; % knee extend
            0*pi/180; % ankle flex
            0*pi/180; % [left] hip adduct
            0*pi/180; % hip flex
            0*pi/180; % knee extend
            0*pi/180]'; % ankle flex

        obs_vtgt_space = [repmat(-10,1,242); repmat(10,1,242)];

        obs_body_space = [repmat(-1.0,1,97); repmat(1.0,1,97)];
%         obs_body_space
        
        Fmax
        lopt
%         visualize = true;
%         integrator_accuracy = 5e-5;
        difficulty = 3;
        report = false;
%         seed = 0;
        t
        d_reward
        v_tgt_field
        flag_new_v_tgt_field
        vtgt
        pose
    end
    
    methods
        function model_ = get_model_key(lm)
            model_ = lm.model;
        end
        
        function lm = set_difficulty(lm, difficulty)
            lm.difficulty = difficulty;
            if difficulty == 0
                lm.time_limit = 1000;
            elseif difficulty == 1
                lm.time_limit = 1000;
            elseif difficulty == 2
                lm.time_limit = 1000;
                fprintf("difficulty 2 for Round 1")
            elseif difficulty == 3
                lm.time_limit = 2500; % 25 sec
                fprintf("difficulty 3 for Round 2")
            end
%             lm.spec.timestep_limit = lm.time_limit;
        end
        
        

        function lm = L2M2019Env(visualize_, integrator_accuracy_, difficulty_, seed_, report_)
            if nargin < 5
                visualize = true;
                integrator_accuracy = 5e-5;
                difficulty = 3;
                seed = 0;
                report = [];
            else
                visualize = visualize_;
                integrator_accuracy = integrator_accuracy_;
                difficulty = difficulty_;
                seed = seed_;
                report = report_;
            end           
            
            lm = lm@OsimEnv(visualize, integrator_accuracy);
            
            lm.obs_body_space(:,1) = [0; 3]; % pelvis height
            lm.obs_body_space(:,2) = [-pi; pi]; % pelvis pitch
            lm.obs_body_space(:,3) = [-pi; pi]; % pelvis roll
            lm.obs_body_space(:,4) = [-20; 20]; % pelvis vel (forward)
            lm.obs_body_space(:,5) = [-20; 20]; % pelvis vel (leftward)
            lm.obs_body_space(:,6) = [-20; 20]; % pelvis vel (upward)
            lm.obs_body_space(:,7) = [-10*pi; 10*pi]; % pelvis angular vel (pitch)
            lm.obs_body_space(:,8) = [-10*pi; 10*pi]; % pelvis angular vel (roll)
            lm.obs_body_space(:,9) = [-10*pi; 10*pi]; % pelvis angular vel (yaw)
            for i = [10, 54]
                lm.obs_body_space(:,i) = [-5, 5]'; % (r,l) ground reaction force normalized to bodyweight (forward)
                lm.obs_body_space(:,i+1) = [-5, 5]'; % (r, l) ground reaction force normalized to bodyweight (rightward)
                lm.obs_body_space(:,i+2) = [-10, 10]'; % (r, l) ground reaction force normalized to bodyweight (upward)
                lm.obs_body_space(:,i+3) = [-45*pi/180, 90*pi/180]'; % (r, l) joint: (+) hip abduction
                lm.obs_body_space(:,i+4) = [-180*pi/180, 45*pi/180]'; % (r, l) joint: (+) hip extension
                lm.obs_body_space(:,i+5) = [-180*pi/180, 15*pi/180]'; % (r, l) joint: (+) knee extension
                lm.obs_body_space(:,i+6) = [-45*pi/180, 90*pi/180]'; % (r, l) joint: (+) ankle extension (plantarflexion)
                lm.obs_body_space(:,i+7) = [-5*pi, 5*pi]'; % (r, l) joint: (+) hip abduction
                lm.obs_body_space(:,i+8) = [-5*pi, 5*pi]'; % (r, l) joint: (+) hip extension
                lm.obs_body_space(:,i+9) = [-5*pi, 5*pi]'; % (r, l) joint: (+) knee extension
                lm.obs_body_space(:,i+10) = [-5*pi, 5*pi]'; % (r, l) joint: (+) ankle extension (plantarflexion)              
            end
            for i = [21 + (0:3:30), 21 + (44:3:74)]
                lm.obs_body_space(:,i) = [0, 3]'; % (r, l) muscle forces, normalized to maximum isometric force
                lm.obs_body_space(:,i+1) = [0, 3]'; % (r, l) muscle lengths, normalized to optimal length
                lm.obs_body_space(:,i+2) = [-50, 50]'; % (r, l) muscle velocities, normalized to optimal length per second
            end
            
            lm.observation_space = [lm.obs_vtgt_space, lm.obs_body_space];
%             lm.model_paths = containers.Map();
%             lm.model_paths('3D') = 'Model/gait14dof22musc_planar_20170320.osim';
%             lm.model_paths('2D') = 'Model/gait14dof22musc_planar_20170320.osim';
%             lm.model_path = lm.model_paths(lm.get_model_key());
            
            
            lm.model_path = 'Model/gait14dof22musc_planar_20170320.osim';
%             py.super(L2M2019Env, lm).OsimEnv(visualize, integrator_accuracy)
        
            lm.Fmax = containers.Map;
            lm.lopt = containers.Map;
            
            MUS = ['HAB', 'HAD', 'HFL', 'GLU', 'HAM', 'RF', 'VAS', 'BFSH', 'GAS', 'SOL', 'TA'];
            mus = ['abd', 'add', 'iliopsoas', 'glut_max', 'hamstrings', 'rect_fem', 'vasti', 'bifemsh', 'gastroc', 'soleus', 'tib_ant'];
            % ['r_leg', 'l_leg']
            for j = 0:(lm.osim_model.muscleSet.getSize()-1)
                muscle = lm.osim_model.muscleSet.get(j);
%                 Fmax = muscle.getMaxIsometricForce();
%                 lopt = muscle.getOptimalFiberLength();
                name = muscle.getName;
                lm.Fmax(char(name)) = muscle.getMaxIsometricForce();
                lm.lopt(char(name)) = muscle.getOptimalFiberLength();
            end

% 
            lm = lm.set_difficulty(difficulty);
% 
            if report
%                 bufsize = 0
%                 lm.observations_file = open('%s-obs.csv' % (report,),'w', bufsize)
%                 lm.actions_file = open('%s-act.csv' % (report,),'w', bufsize)
%                 lm.get_headers()
            end
% 
            % create target velocity field
%             from envs.target import VTgtField
%             lm.vtgt = VTgtField(visualize=visualize, version=lm.difficulty, dt=lm.osim_model.stepsize)            
            lm.vtgt = VTgtField(visualize, lm.difficulty, lm.osim_model.stepsize, 0.5);
            lm.obs_vtgt_space = lm.vtgt.vtgt_space;
        end
        
        function [lm, observation_] = reset(lm, project_, seed_, obs_as_dict_, init_pose_)
            if nargin == 1
                project = true;
                seed = [];
                init_pose = [];
                obs_as_dict = true;
            else
                project=project_;
                seed = seed_;
                init_pose = init_pose_;
                obs_as_dict = obs_as_dict_;
            end
            
            lm.t = 0;
            lm = lm.init_reward();
%             lm.vtgt.reset(version=lm.difficulty, seed=seed);
            lm.vtgt = lm.vtgt.reset(lm.difficulty, seed, [0, 0, 0]);
            
            % initialize statelm.
            lm.osim_model.state = lm.osim_model.model.initializeState();
            if isempty(init_pose)
                init_pose = lm.INIT_POSE;
            end
            state = lm.osim_model.get_state();
            QQ = state.getQ();
            QQDot = state.getQDot();
            for i = 0:16
%                 QQDot(i) = 0;
                QQDot.set(i, 0);
            end
%             QQ(4) = 0; % x: (+) forward
%             QQ(6) = 0; % z: (+) right
%             QQ(2) = 0*pi/180; % roll
%             QQ(3) = 0*pi/180; % yaw
%             QQDot(4) = init_pose(1); % forward speed
%             QQDot(6) = init_pose(2); % forward speed
%             QQ(5) = init_pose(3); % pelvis height
%             QQ(1) = -init_pose(4); % trunk lean: (+) backward
%             QQ(8) = -init_pose(5); % right hip abduct
%             QQ(7) = -init_pose(6); % right hip flex
%             QQ(14) = init_pose(7); % right knee extend
%             QQ(16) = -init_pose(8); % right ankle flex
%             QQ(11) = -init_pose(9); % left hip adduct
%             QQ(10) = -init_pose(10); % left hip flex
%             QQ(15) = init_pose(11); % left knee extend
%             QQ(17) = -init_pose(12); % left ankle flex

            QQ.set(3, 0); % x: (+) forward
            QQ.set(5, 0); % z: (+) right
            QQ.set(1, 0*pi/180); % roll
            QQ.set(2, 0*pi/180); % yaw
            QQDot.set(3, init_pose(1)); % forward speed
            QQDot.set(5, init_pose(2)); % forward speed
            QQ.set(4, init_pose(3)); % pelvis height
            QQ.set(0, -init_pose(4)); % trunk lean: (+) backward
            QQ.set(7, -init_pose(5)); % right hip abduct
            QQ.set(6, -init_pose(6)); % right hip flex
            QQ.set(13, init_pose(7)); % right knee extend
            QQ.set(15, -init_pose(8)); % right ankle flex
            QQ.set(10, -init_pose(9)); % left hip adduct
            QQ.set(9, -init_pose(10)); % left hip flex
            QQ.set(14, init_pose(11)); % left knee extend
            QQ.set(16, -init_pose(12)); % left ankle flex

            
            
            state.setQ(QQ)
            state.setU(QQDot)
            lm.osim_model = lm.osim_model.set_state(state);
            lm.osim_model.model.equilibrateMuscles(lm.osim_model.state)
% 
            lm.osim_model.state.setTime(0)
            lm.osim_model.istep = 0;
% 
%             lm.osim_model.reset_manager();
            lm.osim_model = lm.osim_model.reset_manager();
            
%             lm.dd = py.super(L2M2019Env, lm).get_state_desc();
%             osimenv_ = OsimEnv();
%             lm.dd = osimenv_.get_state_desc();
%             obj = obj@MySuperClass(SuperClassArguments);
%             dd = dd@OsimEnv
%             
%             get_state_desc@OsimEnv(lm)
            use_superclass = true;
            [lm, dd] = lm.get_state_desc(use_superclass);

%             TmpClassdd = OsimEnv(false, 5e-5);
%             lm.dd = TmpClassdd.get_state_desc();
            
%             dd = d@OsimEnv(visualize, integrator_accuracy).get_state_desc();
            dd1 = dd{4}('pelvis');
            dd2 = -dd{4}('pelvis');
            dd3 = dd{1}('ground_pelvis');


            pose_ = [dd1(1), dd2(3), dd3(3)];
            [lm.vtgt, lm.v_tgt_field, lm.flag_new_v_tgt_field] = lm.vtgt.update(pose_);
% 
            if project == false
                observation_ = lm.get_state_desc();
            elseif obs_as_dict
                observation_ = lm.get_observation_dict();
            else
                lm.get_observation()
            end
        end
            
        function lm = load_model(lm , model_path_)
            if nargin < 2
                model_path = 'Model/gait14dof22musc_planar_20170320.osim';

            else
                model_path = model_path_;
            end
%             py.super(L2M2019Env, lm).load_model(model_path)
            
            lm = load_model@OsimEnv(lm, model_path);
            
            observation_space = [lm.obs_vtgt_space, lm.obs_body_space];
%             lm.observation_space = convert_to_gym(observation_space);
            lm.observation_space = observation_space;
        end
        
        function [lm, obs, reward, done, info] = step(lm, action, project_, obs_as_dict_)
            if nargin < 3
                project = true;
                obs_as_dict = true;
            else
                project=project_;
                obs_as_dict = obs_as_dict_;
            end
            
            action_mapped = zeros(1, length(lm.act2mus));
            for i = 1: length(lm.act2mus)
                action_mapped(i) = action(lm.act2mus(i)+1);
            end

%             [obs_, reward, done, info] = py.super(L2M2019Env, lm).step(action_mapped, project, obs_as_dict);
%             [obs_, reward, done, info] = py.super(L2M2019Env, lm).step(action_mapped, project, obs_as_dict);
            [lm, obs_, reward, done, info] = step@OsimEnv(lm, action_mapped, project, obs_as_dict);
            
            lm.t = lm .t + lm.osim_model.stepsize;
            lm.update_footstep()
            
%             ddd = py.super(L2M2019Env, lm).get_state_desc();
            [~, ddd] = lm.get_state_desc(true);
            
            dd1 = ddd{4}('pelvis');
            dd2 = -ddd{4}('pelvis');
            dd3 = ddd{1}('ground_pelvis');
            lm.pose = [dd1(1), dd2(3), dd3(3)];
            [lm.v_tgt_field, lm.flag_new_v_tgt_field] = lm.vtgt.update(lm.pose);            
            
            if project
                if obs_as_dict
                    obs = lm.get_observation_dict();
                else
                    obs = lm.get_observation();
                end
            else
                obs = lm.get_state_desc();
            end
        end
        
        function lm = change_model(lm, model, difficulty, seed)
            if lm.model ~= model
                lm.model = model;
%                 lm.load_model(lm.model_paths[lm.get_model_key()]);
                lm = lm.load_model(lm.model_path);
            end
            lm.set_difficulty(difficulty)
        end
        
        function done_ = is_done(lm)
            [~, state_desc] = lm.get_state_desc(false);
            pelvis_pos_ = state_desc{4}('pelvis');
            done_ = pelvis_pos_(2) < 0.6;
        end
        
        function lm = update_footstep(lm)
            [~, state_desc] = lm.get_state_desc(false);

            % update contact
            
            right_foot_force = state_desc{10}('foot_r');
            left_foot_force = state_desc{10}('foot_l');
            if right_foot_force(2) < -0.05*(lm.MASS*lm.G)
                r_contact = true;
            else
                r_contact = false;
            end
            
            if left_foot_force(2) < -0.05*(lm.MASS*lm.G)
                l_contact = true;
            else
                l_contact = false;
            end

            lm.footstep('new') = false;
            if (~ lm.footstep('r_contact') && r_contact) || (~ lm.footstep('l_contact') && l_contact)
                lm.footstep('new') = true;
                lm.footstep('n') = lm.footstep('n') + 1;
            end

            lm.footstep('r_contact') = r_contact;
            lm.footstep('l_contact') = l_contact;
        end
        
        
        
        function obs_dict_ = get_observation_dict(lm)
            [lm, state_desc] = lm.get_state_desc(false);

            obs_dict = containers.Map();

            obs_dict('v_tgt_field') = state_desc{16};

            % pelvis state (in local frame)
    %         obs_dict('pelvis') = {}
            pelvis_pos = state_desc{4}('pelvis');
            ground_pelvis_angle = state_desc{1}('ground_pelvis');
            obs_dict('pelvis_height') = pelvis_pos(2);
            obs_dict('pelvis_pitch') = -ground_pelvis_angle(1); % (+) pitching forward
            obs_dict('pelvis_roll') = ground_pelvis_angle(2); % (+) rolling around the forward axis (to the right)
            yaw = ground_pelvis_angle(3);

            body_vel_pelvis = state_desc{5}('pelvis');
            [dx_local, dy_local] = rotate_frame(body_vel_pelvis(1), body_vel_pelvis(3), yaw);
            dz_local = body_vel_pelvis(2);

            joint_vel_ground_pelvis = state_desc{2}('ground_pelvis');
            obs_dict('pelvis_vel') = [dx_local,... % (+) forward
                                         -dy_local,... % (+) leftward
                                         dz_local,... % (+) upward
                                         -joint_vel_ground_pelvis(1),... % (+) pitch angular velocity
                                         joint_vel_ground_pelvis(2),... % (+) roll angular velocity
                                         joint_vel_ground_pelvis(3)]; % (+) yaw angular velocity

            % leg state

%             obs_dict('leg') = {};

            force_foot_r = state_desc{10}('foot_r');
            force_foot_l = state_desc{10}('foot_l');
            grf_r = zeros(1,3);
            grm_r = zeros(1,3);
            grf_l = zeros(1,3);
            grm_l = zeros(1,3);
            for i = 1:3
                grf_r(i) = force_foot_r(i)/(lm.MASS*lm.G); % forces normalized by bodyweight
                grm_r(i) = force_foot_r(i+3)/(lm.MASS*lm.G); % moments normalized by bodyweight
                grf_l(i) = force_foot_l(i)/(lm.MASS*lm.G); % forces normalized by bodyweight
                grm_l(i) = force_foot_l(i+3)/(lm.MASS*lm.G); % moments normalized by bodyweight
            end
            [grfx_local_r, grfy_local_r] = rotate_frame(-grf_r(1), -grf_r(3), yaw);
            [grfx_local_l, grfy_local_l] = rotate_frame(-grf_l(1), -grf_l(3), yaw);
            obs_dict('r_leg_ground_reaction_forces') = [ grfx_local_r,... % (+) forward
                                                         grfy_local_r,... % (+) lateral (rightward)
                                                         -grf_r(2)]; % (+) upward

            obs_dict('l_leg_ground_reaction_forces') = [ grfx_local_l,... % (+) forward
                                                         grfy_local_l,... % (+) lateral (rightward)
                                                         -grf_l(2)]; % (+) upward

            % joint angles
            joint_pos_hip_r = state_desc{1}('hip_r');
            joint_pos_hip_l = state_desc{1}('hip_l');
            joint_pos_knee_r = state_desc{1}('knee_r');
            joint_pos_knee_l = state_desc{1}('knee_l');
            joint_pos_angle_r = state_desc{1}('ankle_r');
            joint_pos_angle_l = state_desc{1}('ankle_l');
            
            obs_dict('joint_hip_abd_r') = -joint_pos_hip_r(2);
            obs_dict('joint_hip_abd_l') = -joint_pos_hip_l(2);
            
            obs_dict('joint_hip_r') = -joint_pos_hip_r(1);
            obs_dict('joint_hip_l') = -joint_pos_hip_l(1);
            
            obs_dict('joint_knee_r') = joint_pos_knee_r(1);
            obs_dict('joint_knee_l') = joint_pos_knee_l(1);
            
            obs_dict('joint_ankle_r') = -joint_pos_angle_r(1);
            obs_dict('joint_ankle_l') = -joint_pos_angle_l(1);
            
            d_joint_pos_hip_r = state_desc{2}('hip_r');
            d_joint_pos_hip_l = state_desc{2}('hip_l');
            d_joint_pos_knee_r = state_desc{2}('knee_r');
            d_joint_pos_knee_l = state_desc{2}('knee_l');
            d_joint_pos_angle_r = state_desc{2}('ankle_r');
            d_joint_pos_angle_l = state_desc{2}('ankle_l');
            
            obs_dict('d_joint_hip_abd_r') = -d_joint_pos_hip_r(2);
            obs_dict('d_joint_hip_abd_l') = -d_joint_pos_hip_l(2);
            
            obs_dict('d_joint_hip_r') = -d_joint_pos_hip_r(1);
            obs_dict('d_joint_hip_l') = -d_joint_pos_hip_l(1);
            
            obs_dict('d_joint_knee_r') = d_joint_pos_knee_r(1);
            obs_dict('d_joint_knee_l') = d_joint_pos_knee_l(1);
            
            obs_dict('d_joint_ankle_r') = -d_joint_pos_angle_r(1);
            obs_dict('d_joint_ankle_l') = -d_joint_pos_angle_l(1);

            % muscles
            MUS = {'HAB', 'HAD', 'HFL', 'GLU', 'HAM', 'RF', 'VAS', 'BFSH', 'GAS', 'SOL', 'TA'};
            mus = {'abd', 'add', 'iliopsoas', 'glut_max', 'hamstrings', 'rect_fem', 'vasti', 'bifemsh', 'gastroc', 'soleus', 'tib_ant'};
            

            side_leg = {'_r','_l'};
            for i = 1:2
%                 side = side_leg(i);
                for j = 1:length(MUS)
%                     MUSCLE_((i-1)*11+j) = MUS{j};
%                     muscle_((i-1)*11+j) = mus{j};
                    obs_dict([MUS{j}, side_leg{i}]) = state_desc{14}([mus{j}, side_leg{i}])/lm.Fmax([mus{j}, side_leg{i}]);
                end
            end
            
%             for MUS, mus in zip(    ['HAB', 'HAD', 'HFL', 'GLU', 'HAM', 'RF', 'VAS', 'BFSH', 'GAS', 'SOL', 'TA'],
%                                     ['abd', 'add', 'iliopsoas', 'glut_max', 'hamstrings', 'rect_fem', 'vasti', 'bifemsh', 'gastroc', 'soleus', 'tib_ant']):
%                 obs_dict[leg][MUS] = {}
%                 obs_dict[leg][MUS]['f'] = state_desc['muscles']['{}_{}'.format(mus,side)]['fiber_force']/lm.Fmax[leg][MUS]
% %                 obs_dict[leg][MUS]['l'] = state_desc['muscles']['{}_{}'.format(mus,side)]['fiber_length']/lm.lopt[leg][MUS]
% %                 obs_dict[leg][MUS]['v'] = state_desc['muscles']['{}_{}'.format(mus,side)]['fiber_velocity']/lm.lopt[leg][MUS]
%             end
%             
%             for j = 0:(lm.osim_model.muscleSet.getSize()-1)
%                 muscle = lm.osim_model.muscleSet.fget(j);
% 
%                 name = muscle.getName;
%                 obs_dict(char(name)) = muscle.getMaxIsometricForce();
%                 obs_dict(char(name)) = muscle.getOptimalFiberLength();
%             end
            
            obs_dict_ = obs_dict;
        end
        
        %% Values in the observation vector
        % 'vtgt_field': vtgt vectors in body frame (2*11*11 = 242 values)
        % 'pelvis': height, pitch, roll, 6 vel (9 values)
        % for each 'r_leg' and 'l_leg' (*2)
        %   'ground_reaction_forces' (3 values)
        %   'joint' (4 values)
        %   'd_joint' (4 values)
        %   for each of the eleven muscles (*11)
        %       normalized 'f', 'l', 'v' (3 values)
        % 242 + 9 + 2*(3 + 4 + 4 + 11*3) = 339
        function res = get_observation(lm)
            obs_dict = lm.get_observation_dict();

            % Augmented environment from the L2R challenge
            res = [];

            % target velocity field (in body frame)
            v_tgt = obs_dict('v_tgt_field');
            res = res + v_tgt.tolist();
            
            peilvis_vel_ = obs_dict('pelvis_vel');
            res = [res obs_dict('pelvis_height')];
            res = [res obs_dict('pelvis_pitch')];
            res = [res obs_dict('pelvis_roll')];
            res = [res peilvis_vel_(1)/lm.LENGTH0];
            res = [res peilvis_vel_(2)/lm.LENGTH0];
            res = [res peilvis_vel_(3)/lm.LENGTH0];
            res = [res peilvis_vel_(4)];
            res = [res peilvis_vel_(5)];
            res = [res peilvis_vel_(6)];
 
            res = [res obs_dict('r_leg_ground_reaction_forces')];
            res = [res obs_dict('l_leg_ground_reaction_forces')];
            
            res = [res obs_dict('joint_hip_abd_r')];
            res = [res obs_dict('joint_hip_abd_l')];
            res = [res obs_dict('joint_hip_r')];
            res = [res obs_dict('joint_hip_l')];
            res = [res obs_dict('joint_knee_r')];
            res = [res obs_dict('joint_knee_l')];
            res = [res obs_dict('joint_ankle_r')];
            res = [res obs_dict('joint_ankle_l')]; 
            
            res = [res obs_dict('d_joint_hip_abd_r')];
            res = [res obs_dict('d_joint_hip_abd_l')];
            res = [res obs_dict('d_joint_hip_r')];
            res = [res obs_dict('d_joint_hip_l')];
            res = [res obs_dict('d_joint_knee_r')];
            res = [res obs_dict('d_joint_knee_l')];
            res = [res obs_dict('d_joint_ankle_r')];
            res = [res obs_dict('d_joint_ankle_l')];             

            
            
%             for MUS in ['HAB', 'HAD', 'HFL', 'GLU', 'HAM', 'RF', 'VAS', 'BFSH', 'GAS', 'SOL', 'TA']:
%                     res.append(obs_dict[leg][MUS]['f'])
%                     res.append(obs_dict[leg][MUS]['l'])
%                     res.append(obs_dict[leg][MUS]['v'])
%             return res
%             end
        end
        
        function obs = get_observation_clipped(lm)
            obs = lm.get_observation();           
            obs(obs>lm.observation_space.high) = lm.observation_space.high;
            obs(obs<lm.observation_space.low) = lm.observation_space.low;
        end
        
        function size_ = get_observation_space_size(lm)
        	size_ = 339;
        end
        
        
        function [lm, d] = get_state_desc(lm, use_superclass)
            [lm, d] = get_state_desc@OsimEnv(lm);
%             d = super(L2M2019Env, self).get_state_desc()
%             load_model@OsimEnv(lm, model_path);
            %state_desc['joint_pos']
            %state_desc['joint_vel']
            %state_desc['joint_acc']
            %state_desc['body_pos']
            %state_desc['body_vel']
            %state_desc['body_acc']
            %state_desc['body_pos_rot']
            %state_desc['body_vel_rot']
            %state_desc['body_acc_rot']
            %state_desc['forces']
            %state_desc['muscles']
            %state_desc['markers']
            %state_desc['misc']
            if ~use_superclass
                if ismember(lm.difficulty, [0, 1, 2, 3])
                    v_tgt_d = containers.Map();
                    v_tgt_d('v_tgt_field') = lm.v_tgt_field;
                    d{end+1} = v_tgt_d; % shape: (2, 11, 11)
                end
            end
        end
        
        function lm = init_reward(lm)
            lm = lm.init_reward_1();
        end
        
        function lm = init_reward_1(lm)
            lm.d_reward = containers.Map;

%             lm.d_reward('weight') = {}
            lm.d_reward('weight_footstep') = 10;
            lm.d_reward('weight_effort') = 1;
            lm.d_reward('weight_v_tgt') = 1;
            lm.d_reward('weight_v_tgt_R2') = 3;

            lm.d_reward('alive') = 0.1;
            lm.d_reward('effort') = 0;

%             lm.d_reward('footstep') = {}
            lm.d_reward('footstep_effort') = 0;
            lm.d_reward('footstep_del_t') = 0;
            lm.d_reward('footstep_del_v') = 0;
        end
        
        function [lm, reward_] = get_reward(lm)
            if lm.difficulty == 3 % Round 2
                [lm, reward_] = lm.get_reward_2();
            else
            [lm, reward_] = lm.get_reward_1();
            end
        end
        
        function [lm, reward_] = get_reward_1(lm) % for L2M2019 Round 1
            state_desc = lm.get_state_desc();
            if isempty(lm.get_prev_state_desc())
                reward_ = 0;
            end

            reward = 0;
            dt = lm.osim_model.stepsize;

            % alive reward
            % should be large enough to search for 'success' solutions (alive to the end) first
            reward = reward + lm.d_reward('alive');

            % effort ~ muscle fatigue ~ (muscle activation)^2 
            ACT2 = 0;
            for i = 0:(lm.muscleSet.getSize()-1)
                name = lm.muscleSet.get(i).getName();
                ACT2 = ACT2 + (state_desc{11}(name)).^2;        
            end
            
            lm.d_reward('effort') = lm.d_reward('effort') + ACT2*dt;
            lm.d_reward('footstep_effort') = lm.d_reward('footstep_effort') + ACT2*dt;

            lm.d_reward('footstep_del_t') = lm.d_reward('footstep_del_t') + dt;

            % reward from velocity (penalize from deviating from v_tgt)

            body_pos_pelvis = state_desc{4}('pelvis');
            body_vel_pelvis = state_desc{5}('pelvis');
            
            p_body = [body_pos_pelvis(1), - body_pos_pelvis(3)];
            v_body = [body_vel_pelvis(1), - body_vel_pelvis(3)];
            v_tgt = lm.vtgt.get_vtgt(p_body).T;

            lm.d_reward('footstep_del_v') = lm.d_reward('footstep_del_v') + (v_body - v_tgt)*dt;

            % footstep reward (when made a new step)
            if lm.footstep('new')
                % footstep reward: so that solution does not avoid making footsteps
                % scaled by del_t, so that solution does not get higher rewards by making unnecessary (small) steps
                reward_footstep_0 = lm.d_reward('weight_footstep')*lm.d_reward('footstep_del_t');

                % deviation from target velocity
                % the average velocity a step (instead of instantaneous velocity) is used
                % as velocity fluctuates within a step in normal human walking
                %reward_footstep_v = -lm.reward_w['v_tgt']*(lm.footstep['del_vx']**2)
                reward_footstep_v = -lm.d_reward('weight_v_tgt')*norm(lm.d_reward('footstep_del_v'))/lm.LENGTH0;

                % panalize effort
                reward_footstep_e = -lm.d_reward('weight_effort')*lm.d_reward('footstep_effort');

                lm.d_reward('footstep_del_t') = 0;
                lm.d_reward('footstep_del_v') = 0;
                lm.d_reward('footstep_effort') = 0;

                reward = reward + reward_footstep_0 + reward_footstep_v + reward_footstep_e;
            end
            
            % success bonus
            if (~lm.is_done()) && (lm.osim_model.istep >= lm.spec.timestep_limit) %and self.failure_mode is 'success':
                % retrieve reward (i.e. do not penalize for the simulation terminating in a middle of a step)
                %reward_footstep_0 = self.d_reward['weight']['footstep']*self.d_reward['footstep']['del_t']
                %reward += reward_footstep_0 + 100
                reward = reward + reward_footstep_0 + 10;
            end
            reward_ = reward;
        end
        
        
        
        function [lm, reward_] = get_reward_2(lm) % for L2M2019 Round 2
            [~, state_desc] = lm.get_state_desc(false);
            if isempty(lm.get_prev_state_desc())
                reward_ = 0;
            end

            reward = 0;
            dt = lm.osim_model.stepsize;

            % alive reward
            % should be large enough to search for 'success' solutions (alive to the end) first
            reward = reward + lm.d_reward('alive');

            % effort ~ muscle fatigue ~ (muscle activation)^2 
            ACT2 = 0;
            for i = 0:(lm.osim_model.muscleSet.getSize()-1)
                name = lm.osim_model.muscleSet.get(i).getName();
                ACT2 = ACT2 + (state_desc{11}(char(name))).^2;     
            end
            
            lm.d_reward('effort') = lm.d_reward('effort') + ACT2*dt;
            lm.d_reward('footstep_effort') = lm.d_reward('footstep_effort') + ACT2*dt;

            lm.d_reward('footstep_del_t') = lm.d_reward('footstep_del_t') + dt;

            % reward from velocity (penalize from deviating from v_tgt)

            body_pos_pelvis = state_desc{4}('pelvis');
            body_vel_pelvis = state_desc{5}('pelvis');
            
            p_body = [body_pos_pelvis(1), - body_pos_pelvis(3)];
            v_body = [body_vel_pelvis(1), - body_vel_pelvis(3)];
            v_tgt = lm.vtgt.get_vtgt(p_body)';

            lm.d_reward('footstep_del_v') = lm.d_reward('footstep_del_v') + (v_body - v_tgt)*dt;

            % simulation ends successfully
            flag_success = (~lm.is_done()... % model did not fall down
                && (lm.osim_model.istep >= lm.spec.timestep_limit)... % reached end of simulatoin
                && lm.footstep('n') > 5); % took more than 5 footsteps (to prevent standing still)

            % footstep reward (when made a new step)
            if lm.footstep('new') || flag_success
                % footstep reward: so that solution does not avoid making footsteps
                % scaled by del_t, so that solution does not get higher rewards by making unnecessary (small) steps
                reward_footstep_0 = lm.d_reward('weight_footstep')*lm.d_reward('footstep_del_t');

                % deviation from target velocity
                % the average velocity a step (instead of instantaneous velocity) is used
                % as velocity fluctuates within a step in normal human walking
                %reward_footstep_v = -lm.reward_w['v_tgt']*(lm.footstep['del_vx']**2)
                reward_footstep_v = -lm.d_reward('weight_v_tgt')*norm(lm.d_reward('footstep_del_v'))/lm.LENGTH0;

                % panalize effort
                reward_footstep_e = -lm.d_reward('weight_effort')*lm.d_reward('footstep_effort');

                lm.d_reward('footstep_del_t') = 0;
                lm.d_reward('footstep_del_v') = 0;
                lm.d_reward('footstep_effort') = 0;

                reward = reward + reward_footstep_0 + reward_footstep_v + reward_footstep_e;
            end

            % task bonus: if stayed enough at the first target
            if lm.flag_new_v_tgt_field
                reward = reward + 500;
            end

            reward_ = reward;
        end
        
        
        
        
    end
end


function [x_rot, y_rot] = rotate_frame(x, y, theta)
    x_rot = cos(theta)*x - sin(theta)*y;
    y_rot =sin(theta)*x + cos(theta)*y;
    
end