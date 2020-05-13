classdef OsimModel
    % Initialize simulation  
    properties (Constant)      
        stepsize = 0.01; 
    end
    
    
    properties
        model_path = 'Model/gait14dof22musc_planar_20170320.osim';
        visualize = true;
        integrator_accuracy = 5e-5;
        model
        model_state
        brain
        muscleSet
        forceSet
        bodySet
        jointSet
        markerSet
        contactGeometrySet
        noutput
        last_action
        state
        manager
        istep
        state_desc
        maxforces = [];
        curforces = [];
        state_desc_istep = [];
        prev_state_desc
        vtgt
    end

    methods
        function om = OsimModel(model_path_, visualize_, integrator_accuracy_)
            if nargin == 0
%                 om.model_path = 'Model/gait9dof18musc.osim';
                om.model_path = 'Model/gait14dof22musc_planar_20170320.osim';
                om.visualize = true;
                om.integrator_accuracy = 5e-5;               
            else
                % Provide values for superclass constructor
                % and initialize other inputs
                om.model_path = model_path_;
                om.visualize = visualize_;
                om.integrator_accuracy = integrator_accuracy_;
            end

            om.model = org.opensim.modeling.Model(om.model_path);
            om.model_state = om.model.initSystem();
            om.brain = org.opensim.modeling.PrescribedController();
            
            om.model.setUseVisualizer(om.visualize)            
            
            om.muscleSet = om.model.getMuscles();
            om.forceSet = om.model.getForceSet();
            om.bodySet = om.model.getBodySet();
            om.jointSet = om.model.getJointSet();
            om.markerSet = om.model.getMarkerSet();
            om.contactGeometrySet = om.model.getContactGeometrySet();
          
            for j = 0:(om.muscleSet.getSize()-1)
                func = org.opensim.modeling.Constant(1.0);
                om.brain.addActuator(om.muscleSet.get(j));
                om.brain.prescribeControlForActuator(j, func);

%                 om.maxforces.append(om.muscleSet.get(j).getMaxIsometricForce());
%                 om.curforces.append(1.0);
            end

            om.noutput = om.muscleSet.getSize();

            om.model.addController(om.brain);
            om.model_state = om.model.initSystem();         
        end
    

        function list_elements(om)

            fprintf("-------JOINTS-------\n")
            for i = 0:(om.jointSet.getSize()-1)
                fprintf('%3d ',i+1)
                disp(om.jointSet.get(i).getName())
                fprintf('%c%c', 8, 8);
                fprintf('\n');
            end
            fprintf("\n-------BODIES-------\n")
            for i = 0:(om.bodySet.getSize()-1)
                fprintf('%3d ',i+1)
                disp(om.bodySet.get(i).getName())
                fprintf('%c%c', 8, 8);
                fprintf('\n');
            end
            fprintf("\n-------MUSCLES-------\n")
            for i = 0:(om.muscleSet.getSize()-1)
                fprintf('%3d ',i+1)
                disp(om.muscleSet.get(i).getName())
                fprintf('%c%c', 8, 8);
                fprintf('\n');
            end
            fprintf("\n-------FORCES-------\n")
            for i = 0:(om.forceSet.getSize()-1)
                fprintf('%3d ',i+1)
                disp(om.forceSet.get(i).getName())
                fprintf('%c%c', 8, 8);
                fprintf('\n');
            end
            fprintf("\n-------MARKERS-------\n")
            for i = 0:(om.markerSet.getSize()-1)
                fprintf('%3d ',i+1)
                disp(om.markerSet.get(i).getName())
                fprintf('%c%c', 8, 8);
                fprintf('\n');
            end
        end       

    
        function om = actuate(om, action)
            action(action>1) = 1;
            action(action<0) = 0;
            om.last_action = action;

            om.brain = org.opensim.modeling.PrescribedController.safeDownCast(om.model.getControllerSet().get(0));
            functionSet = om.brain.get_ControlFunctions();

            for j =0:(functionSet.getSize()-1)
                func = org.opensim.modeling.Constant.safeDownCast(functionSet.get(j));
                func.setValue( double(action(j+1)) );
            end
        end
        
        function om = set_activations(om, activations)
            for j = 0:(om.muscleSet.getSize()-1)
                om.muscleSet.get(j).setActivation(om.state, activations(j+1))
                om.reset_manager()
            end

        end
        
        function cur_activations  = get_activations(om)
            for j = 0:(om.muscleSet.getSize()-1)
                cur_activations = om.muscleSet.get(j).getActivation(om.state);
            end
        end
            
            
        
        
        function [res, om] = compute_state_desc(om)
            om.model.realizeAcceleration(om.state)

            % Joints
            joint_pos = containers.Map();
            joint_vel = containers.Map();
            joint_acc = containers.Map();
            joint_pos_value = zeros(om.jointSet.getSize(),3);
            joint_vel_value = zeros(om.jointSet.getSize(),3);
            joint_acc_value = zeros(om.jointSet.getSize(),3);
            for i = 0:(om.jointSet.getSize()-1)
                joint = om.jointSet.get(i);
                name = joint.getName();
                for j = 0:(joint.numCoordinates()-1)
                    joint_pos_value(i+1,j+1) = joint.get_coordinates(j).getValue(om.state);
                    joint_vel_value(i+1,j+1) = joint.get_coordinates(j).getSpeedValue(om.state);
                    joint_acc_value(i+1,j+1) = joint.get_coordinates(j).getAccelerationValue(om.state);
                end
                joint_pos(char(name)) = joint_pos_value(i+1,:);
                joint_vel(char(name)) = joint_vel_value(i+1,:);
                joint_acc(char(name)) = joint_acc_value(i+1,:); 
            end
            

            %% Bodies
            body_pos = containers.Map();
            body_vel = containers.Map();
            body_acc = containers.Map();
            body_pos_rot = containers.Map();
            body_vel_rot = containers.Map();
            body_acc_rot = containers.Map();
            body_pos_value = zeros(om.bodySet.getSize(),3);
            body_vel_value = zeros(om.bodySet.getSize(),3);
            body_acc_value = zeros(om.bodySet.getSize(),3);
            body_pos_rot_value = zeros(om.bodySet.getSize(),3);
            body_vel_rot_value = zeros(om.bodySet.getSize(),3);
            body_acc_rot_value = zeros(om.bodySet.getSize(),3);           
            for i = 0:(om.bodySet.getSize()-1)
                body = om.bodySet.get(i);
                name = body.getName();
                
                for j = 0:2
                    body_pos_value(i+1,j+1) = body.getTransformInGround(om.state).p().get(j);
                    body_vel_value(i+1,j+1) = body.getVelocityInGround(om.state).get(1).get(j);
                    body_acc_value(i+1,j+1) = body.getAccelerationInGround(om.state).get(1).get(j);
                    body_pos_rot_value(i+1,j+1) = body.getTransformInGround(om.state).R().convertRotationToBodyFixedXYZ().get(j);
                    body_vel_rot_value(i+1,j+1) = body.getVelocityInGround(om.state).get(0).get(j);
                    body_acc_rot_value(i+1,j+1) = body.getAccelerationInGround(om.state).get(0).get(j);
                end
                                
                body_pos(char(name)) = body_pos_value(i+1,:);
                body_vel(char(name)) = body_vel_value(i+1,:);
                body_acc(char(name)) = body_acc_value(i+1,:);
                body_pos_rot(char(name)) = body_pos_rot_value(i+1,:);
                body_vel_rot(char(name)) = body_vel_rot_value(i+1,:);
                body_acc_rot(char(name)) = body_acc_rot_value(i+1,:);
            end

            %% Forces

            forces = containers.Map();
            for i = 22:(om.forceSet.getSize()-1)
                force = om.forceSet.get(i);
                name = force.getName();
                values = force.getRecordValues(om.state);
                value_array = zeros(1,values.size);
                for j = 0:(values.size-1)
                    value_array(j+1) = values.get(j);
                end
                forces(char(name)) = value_array;
            end

            %% Muscles
            muscles = containers.Map();
            muscles_activation = containers.Map();
            muscles_fiber_length = containers.Map();
            muscles_fiber_velocity = containers.Map();
            muscles_fiber_force = containers.Map();
            for i = 0:(om.muscleSet.getSize()-1)
                muscle = om.muscleSet.get(i);
                name = muscle.getName();
                muscles(char(name)) = {};
                muscles_activation(char(name)) = muscle.getActivation(om.state);
                muscles_fiber_length(char(name)) = muscle.getFiberLength(om.state);
                muscles_fiber_velocity(char(name)) = muscle.getFiberVelocity(om.state);
                muscles_fiber_force(char(name)) = muscle.getFiberForce(om.state);
                % We can get more properties from here http://myosin.sourceforge.net/2125/classOpenSim_1_1Muscle.html 
            end

%             %% Markers
%             markers = containers.Map();


            %% Mass center
            mass_center = containers.Map();
            for i = 0:2
                mass_center_pos_vec = om.model.calcMassCenterPosition(om.state);
                mass_center_vel_vec = om.model.calcMassCenterVelocity(om.state);
                mass_center_acc_vec = om.model.calcMassCenterAcceleration(om.state);
            end
            mass_center_pos_value = eval(mass_center_pos_vec.toString().substring(1));
            mass_center_vel_value = eval(mass_center_vel_vec.toString().substring(1));
            mass_center_acc_value = eval(mass_center_acc_vec.toString().substring(1));
            mass_center('mass_center_pos') = mass_center_pos_value;
            mass_center('mass_center_vel') = mass_center_vel_value;
            mass_center('mass_center_acc') = mass_center_acc_value;
            

            
            res = {joint_pos, joint_vel, joint_acc,...
                body_pos, body_vel, body_acc, body_pos_rot, body_vel_rot, body_acc_rot,...
                forces,...
                muscles_activation, muscles_fiber_length, muscles_fiber_velocity, muscles_fiber_force,...
                mass_center};
                
                
        end

        
   
        function [om, state_desc_] = get_state_desc(om)
            if ~isequal(om.state_desc_istep, om.istep)
                om.prev_state_desc = om.state_desc;
                om.state_desc = om.compute_state_desc();
                om.state_desc_istep = om.istep;
            end                
            state_desc_ = om.state_desc;

        end
        

        function set_strength(om, strength)
            om.curforces = strength;
            for i = 0:(length(om.curforces)-1)
                om.muscleSet.get(i).setMaxIsometricForce(om.curforces(i+1) * om.maxforces(i+1))    
            end
        end
        
        
        
        
        
        function body_ = get_body(om, name)
            body_ = om.bodySet.get(name);
        end

        function jointSet_ = get_joint(om, name)
            jointSet_ = om.jointSet.get(name);
        end

        function muscleSet_ = get_muscle(om, name)
            muscleSet_  = om.muscleSet.get(name);
        end
        
        function markerSet_ = get_marker(om, name)
            markerSet_ = om.markerSet.get(name);
        end
        
        function contactGeometrySet_ = get_contact_geometry(om, name)
            contactGeometrySet_ = om.contactGeometrySet.get(name);
        end
        
        function forceSet_ = get_force(om, name)
            forceSet_ = om.forceSet.get(name);
        end
        
        function noutput_ = get_action_space_size(om)
            noutput_ = om.noutput;
        end
        
        function om = set_integrator_accuracy(om, integrator_accuracy)
            om.integrator_accuracy = integrator_accuracy;       
        end
        
        
        function om = reset_manager(om)
            om.manager = org.opensim.modeling.Manager(om.model);
            om.manager.setIntegratorAccuracy(om.integrator_accuracy);
            om.manager.initialize(om.state);
        end
        
        function om = reset(om)
            om.state = om.model.initializeState();
            om.model.equilibrateMuscles(om.state);
            om.state.setTime(0);
            om.istep = 0;

            om.reset_manager()
        end

        function cur_state = get_state(om)
            cur_state = org.opensim.modeling.State(om.state);
        end

        function om = set_state(om, state)
            om.state = state;
            om.istep = fix(om.state.getTime() / om.stepsize) ;
            om.reset_manager();
        end

        function om = integrate(om)
            % Define the new endtime of the simulation
            om.istep = om.istep + 1;

            % Integrate till the new endtime
%             om.state = om.manager.integrate(om.stepsize * om.istep);
%             om.manager = org.opensim.modeling.Manager(om.model);
%             om.manager.initialize(om.state);
%             
            endTime = om.stepsize * om.istep;
            om.state = om.manager.integrate(endTime);
        end
        
           
        
            
    end
end







        
        