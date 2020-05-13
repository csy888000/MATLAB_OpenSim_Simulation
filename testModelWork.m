


%% Set some properties for the simulation
endTime   = 0.5;
visualize = true;
plotResults = true;

%% Define the intial coordinate values and speeds for the model. Translations
% coordinates are in meters, rotations are in radians.
% %%Initial stand pose
% pelvisTilt = -2.865;
% pelvisTYValue = 0.91;
% pelvisTYSpeed = 0;
% pelvisTXValue = 0;
% pelvisTXSpeed  = 0;
% rHipValue = deg2rad(0.0);
% rKneeValue = deg2rad(0.0);
% rAnkleValue = deg2rad(0.0);
% 
% lHipValue = deg2rad(0);
% lKneeValue = deg2rad(0.0);
% lAnkleValue = deg2rad(0.0);
% 
% rHipSpeed = deg2rad(0);
% rKneeSpeed = deg2rad(0);
% rAnkleSpeed = deg2rad(0);
% 
% lHipSpeed = deg2rad(0);
% lKneeSpeed = deg2rad(0);
% lAnkleSpeed = deg2rad(0);
%   

%% Initial walking pose
pelvisTilt = 2.012303881285582852; %trunk lean
pelvisTYValue = 0.9023245653983965608; %pelvis height
pelvisTYSpeed = 0; 
pelvisTXValue = 0;
% pelvisTXSpeed  = 1.699999999999999956;
pelvisTXSpeed  = 1.7; %forward speed

rHipValue = 6.952390849304798115e-01;
rKneeValue = -3.231075259785813891e-01;
rAnkleValue = 1.709011708233401095e-01;

lHipValue = -5.282323914341899296e-02;
lAnkleValue = -1.745329251994329478e-01;
lKneeValue = -8.041966456860847323e-01;

rHipSpeed = deg2rad(0);
rKneeSpeed = deg2rad(0);
rAnkleSpeed = deg2rad(0);

lHipSpeed = deg2rad(0);
lKneeSpeed = deg2rad(0);
lAnkleSpeed = deg2rad(0);



%% Initial walking pose
% pelvisTilt = 2.012303881285582852; %trunk lean
% pelvisTYValue = 0.9023245653983965608; %pelvis height
% pelvisTYSpeed = 0; 
% pelvisTXValue = 0;
% pelvisTXSpeed  = 0; %forward speed
% 
% rHipValue = 0;
% rKneeValue = 0;
% rAnkleValue = 0;
% 
% lHipValue = 0;
% lAnkleValue = 0;
% lKneeValue = 0;
% 
% rHipSpeed = deg2rad(0);
% rKneeSpeed = deg2rad(0);
% rAnkleSpeed = deg2rad(0);
% 
% lHipSpeed = deg2rad(0);
% lKneeSpeed = deg2rad(0);
% lAnkleSpeed = deg2rad(0);




%% Import OpenSim Libraries
import org.opensim.modeling.*;

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
% model_path = 'Model/gait9dof18musc.osim';
model_path = 'Model/gait14dof22musc_planar_20170320.osim';

%% Instantiate the Model
osimModel = Model(model_path);

% Set the visualizer use.
osimModel.setUseVisualizer(visualize)




%% Add a controller that specifies the excitation of the muscle.

action = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, ...
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

action(action>1) = 1;
action(action<0) = 0;
last_action = action;


brain = PrescribedController();  
muscleSet = osimModel.getMuscles();

for j = 0: muscleSet.getSize()-1
    disp(muscleSet.get(j));
end



for j = 0: muscleSet.getSize()-1
    func = Constant(0.0);
    brain.addActuator(muscleSet.get(j));
    brain.prescribeControlForActuator(j, func);
end
noutput = muscleSet.getSize();
            
osimModel.addController(brain)

        
        

% brain.prescribeControlForActuator('biceps',StepFunction(0.5, 3.0, 0.0, 0.1));
% brain.prescribeControlForActuator('rect_fem_r',Constant(1));

brain = PrescribedController.safeDownCast(osimModel.getControllerSet().get(0));
functionSet = brain.get_ControlFunctions();

for j = 0: functionSet.getSize()-1
    func = Constant.safeDownCast(functionSet.get(j));
    
    func.setValue( double(action(j+1)) );
end




%% Initialize the underlying computational system and get a reference to
% the system state.
state = osimModel.initSystem();



%% Change the initial Coordinate values and speeds of the model
% osimModel.updCoordinateSet().get('pelvis_ty').setValue(state, pelvisTYValue);
% osimModel.updCoordinateSet().get('pelvis_ty').setLocked(state, pelvisTYSpeed);
% osimModel.updCoordinateSet().get('pelvis_tx').setValue(state, pelvisTXValue);
% osimModel.updCoordinateSet().get('pelvis_tx').setSpeedValue(state, pelvisTXSpeed);
osimModel.updCoordinateSet().get('hip_flexion_r').setValue(state, rHipValue);
osimModel.updCoordinateSet().get('hip_flexion_r').setLocked(state, rHipSpeed);
osimModel.updCoordinateSet().get('hip_flexion_l').setValue(state, lHipValue);
osimModel.updCoordinateSet().get('hip_flexion_l').setLocked(state, lHipSpeed);
osimModel.updCoordinateSet().get('knee_angle_r').setValue(state, rKneeValue);
osimModel.updCoordinateSet().get('knee_angle_r').setLocked(state, rKneeSpeed)
osimModel.updCoordinateSet().get('knee_angle_l').setValue(state, lKneeValue);
osimModel.updCoordinateSet().get('knee_angle_l').setLocked(state, lKneeSpeed);
osimModel.updCoordinateSet().get('ankle_angle_r').setValue(state, rAnkleValue);
osimModel.updCoordinateSet().get('ankle_angle_r').setLocked(state, rAnkleSpeed);
osimModel.updCoordinateSet().get('ankle_angle_l').setValue(state, lAnkleValue);
osimModel.updCoordinateSet().get('ankle_angle_l').setLocked(state, lAnkleSpeed);


%% Set the Vizualizer parameters
if visualize
    sviz = osimModel.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
    % Show "ground and sky" background instead of just a black background.
    sviz.setBackgroundTypeByInt(1);
    % Set the default ground height down so that the walker platform is
    % viewable.
    sviz.setGroundHeight(0)
    % Set the initial position of the camera
    sviz.setCameraTransform(Transform(Rotation(),Vec3(0.5,1,5)));
    % Show help text in the visualization window.
    help = DecorativeText('ESC or CMD-Q to quit.');
    help.setIsScreenText(true);
    sviz.addDecoration(0, Transform(Vec3(0, 0, 0)), help);
end




            
            
            
%% Run a fwd simulation using the manager
manager = Manager(osimModel);
state.setTime(0);
manager.initialize(state);
state = manager.integrate(endTime);



%% Get the states table from the manager and print the results.
sTable = manager.getStatesTable();
stofiles = STOFileAdapter();
if ~isdir('ResultsFWD')
    mkdir ResultsFWD
end
stofiles.write(sTable, 'ResultsFWD/simulation_states.sto');

%% Use the provided plotting function to plot some results.
if plotResults
    PlotOpenSimData;
end

%% Display Messages
display('Forward Tool Finished.');
display('Output files were written to the /Results/FWD directory:')
