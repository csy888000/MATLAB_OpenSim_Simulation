classdef OsimEnv %< gymEnv
    properties
        action_space
        observation_space
        osim_model
        istep = 0;
        verbose = false;

        visualize = true;
        integrator_accuracy = 5e-5;
        
        spec
        time_limit = 1e10

        prev_state_desc = [];

        model_path = 'Model/gait14dof22musc_planar_20170320.osim'; % os.path.join(os.path.dirname(__file__), '../models/MODEL_NAME.osim')    

    end
    
    methods
        function get_reward(oEnv)
            disp('Not Implemented Error')
        end

        function is_done_ = is_done(oEnv)
            is_done_ = false;
        end

        function oEnv = OsimEnv(visualize_, integrator_accuracy_)
            if nargin == 0
                oEnv.visualize = true;
                oEnv.integrator_accuracy = 5e-5;               
            else
                oEnv.visualize = visualize_;
                oEnv.integrator_accuracy = integrator_accuracy_;
            end
            oEnv = oEnv.load_model();
        end
        

        function oEnv = load_model(oEnv, model_path_)
            if nargin < 2
                oEnv.model_path = 'Model/gait14dof22musc_planar_20170320.osim';
            else
                oEnv.model_path = model_path_;
            end
            
            oEnv.osim_model = OsimModel(oEnv.model_path, oEnv.visualize, oEnv.integrator_accuracy);

            % Create specs, action and observation spaces mocks for compatibility with OpenAI gym
            oEnv.spec = Spec();
            oEnv.spec.timestep_limit = oEnv.time_limit;

            oEnv.action_space = [ zeros(1,oEnv.osim_model.get_action_space_size()); ones(1,oEnv.osim_model.get_action_space_size())];
    %        oEnv.observation_space = ( [-math.pi*100] * oEnv.get_observation_space_size(), [math.pi*100] * oEnv.get_observation_space_s
            oEnv.observation_space = [ zeros(1,oEnv.get_observation_space_size()); zeros(1,oEnv.get_observation_space_size()) ];

%             oEnv.action_space = convert_to_gym(oEnv.action_space);
%             oEnv.observation_space = convert_to_gym(oEnv.observation_space);
        end

        function [oEnv, state_desc_] = get_state_desc(oEnv)
            [oEnv.osim_model, state_desc_] = oEnv.osim_model.get_state_desc();
        end

        function prev_state_desc_ = get_prev_state_desc(oEnv)
            prev_state_desc_ = oEnv.prev_state_desc;
        end

        function observation_ = get_observation(oEnv)
            % This one will normally be overwritten by the environments
            % In particular, for the gym we want a vector and not a dictionary
            observation_ = oEnv.osim_model.get_state_desc();
        end

        function observation_dict_ = get_observation_dict(oEnv)
            observation_dict_ = oEnv.osim_model.get_state_desc();
        end

        function observation_space_size_ = get_observation_space_size(oEnv)
            observation_space_size_ = 0;
        end

        function action_space_size_ = get_action_space_size(oEnv)
            action_space_size_ = oEnv.osim_model.get_action_space_size();
        end

        function output_ = reset(oEnv, project, obs_as_dict)
            % need to correct
            if (nargin<3)
                project = true;
                obs_as_dict = true;
            end
            oEnv.osim_model.reset();

            if project == false
                output_ = oEnv.get_state_desc();
            elseif obs_as_dict
                output_ = oEnv.get_observation_dict();
            else
                output_ = oEnv.get_observation();
            end
        end

        function [oEnv, obs, reward_, is_done_, out_of_limit_] = step(oEnv, action, project, obs_as_dict)
            % need to correct
            if (nargin<3)
                project = true;
                obs_as_dict = true;
            end
            oEnv.prev_state_desc = oEnv.get_state_desc(false);      
            oEnv.osim_model = oEnv.osim_model.actuate(action);
            oEnv.osim_model = oEnv.osim_model.integrate();

            if project
                if obs_as_dict
                    obs = oEnv.get_observation_dict();
                else
                    obs = oEnv.get_observation();
                end
            else
                obs = oEnv.get_state_desc();
            end
            
            [oEnv, reward_] = oEnv.get_reward();
            is_done_ = oEnv.is_done();
            out_of_limit_ = oEnv.osim_model.istep >= oEnv.spec.timestep_limit;
        end
                
%         function space_ = convert_to_gym(space)
%             space_ =  spaces.Box((space[0]), (space[1]) );
%         end

%         function env = gymify_env(env)
%             env.action_space = convert_to_gym(env.action_space);
%             env.observation_space = convert_to_gym(env.observation_space);
% 
% %             env.spec = Specification(env.timestep_limit);
% %             env.spec.action_space = env.action_space;
% %             env.spec.observation_space = env.observation_space;
%         end
    
    
    end
end
