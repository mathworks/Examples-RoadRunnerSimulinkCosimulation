function helperSLTrajectoryFollowerWithRRScenarioSetup(nvp)
%helperSLTrajectoryFollowerWithRRScenarioSetup initializes parameters for
%the model.
% 
% MaxPathPoints name-value argument defines the upper limit on the number
% of path points to read from RoadRunner Scenario.
%
% NOTE: The name of this helper function and it's functionality may
% change without notice in a future release, or the helper function
% itself may be removed.
%

% Copyright 2021-2023 The MathWorks, Inc.

%%
arguments
    nvp.MaxPathPoints = 5000;
    nvp.scenarioSimulationObj = [];
    nvp.rrAppObj = [];
    nvp.behaviorName = "";
    nvp.controlMode = 'stanley';
    nvp.numReferencePose = 1;
    nvp.MultibodySetting = false;
end

if ~isempty(nvp.scenarioSimulationObj)
    % Open scenario
    % openScenario(nvp.rrAppObj, "scenario_LOF_LeadBreakDown.rrscenario");

    % Read actor profiles from RoadRunner Scenario
    worldActor = nvp.scenarioSimulationObj.getScenario();
    world = worldActor.actor_spec.world_spec;

    % Get ego ActorID
    for i = 1:length(world.behaviors)
        if contains(upper(world.behaviors(i).asset_reference),upper(nvp.behaviorName))
            egoBehavior = world.behaviors(i).id;
            break;
        end
    end
    egoActorID = 1;
    for i = 1:length(world.actors)
        id = str2double(world.actors(i).actor_spec.id);
        if isequal(world.actors(i).actor_spec.behavior_id,egoBehavior)
            egoActorID = id;
        end
    end

    egoSetSpeed = str2double(nvp.rrAppObj.getScenarioVariable('egoInitialSpeed')); % get speed, m/s
    egoInitialPose = helperGetActorPose(world.actors, egoActorID, egoSetSpeed);
    actorProfiles = helperGetActorProfiles(world.actors);

    egoVehDyn = egoVehicleDynamicsParamsFromRR(egoInitialPose, actorProfiles(egoActorID));
    assignin('base','egoInitialPose',egoInitialPose);
    assignin('base','egoVehDyn',egoVehDyn);
end

%% Path following controller parameters
PredictionHorizon = 10;
assignin('base','tau',             0.5);     % Time constant for longitudinal dynamics 1/s/(tau*s+1)
assignin('base','time_gap',        1.5);     % Time gap               (s)
assignin('base','default_spacing', 10);      % Default spacing        (m)
assignin('base','max_ac',          3);       % Maximum acceleration   (m/s^2)
assignin('base','min_ac',          -3);      % Minimum acceleration   (m/s^2)
assignin('base','max_steer',       0.26);    % Maximum steering       (rad)
assignin('base','min_steer',       -0.26);   % Minimum steering       (rad) 
assignin('base','PredictionHorizon', PredictionHorizon);    % Prediction horizon     
assignin('base','v0_ego', 0);                % Initial longitudinal velocity (m/s)
assignin('base','tau2', 0.07);               % Longitudinal time constant (brake)             (N/A)
assignin('base','max_dc', 10);               % Maximum deceleration   (m/s^2)
assignin('base','control_timeStep', 0.1);    % control time step (s)

%% Dynamics modeling parameters
lf = 1.5130;
lr = 1.3050;
assignin('base','m',  1575);                 % Total mass of vehicle                          (kg)
assignin('base','Iz', 2100);                 % Yaw moment of inertia of vehicle               (m*N*s^2)
assignin('base','Cf', 19000);                % Cornering stiffness of front tires             (N/rad)
assignin('base','Cr', 33000);                % Cornering stiffness of rear tires              (N/rad)
assignin('base','lf', lf);               % Longitudinal distance from c.g. to front tires (m)
assignin('base','lr', lr);               % Longitudinal distance from c.g. to rear tires  (m)
assignin('base', 'steeringGain', 0.8);       % Steering gain
assignin('base', 'steerLimit', 30);     % Max steering limit
assignin('base', 'steerGear', 18);     % steering gear ratio

%% parameters for reference points output (for helperReferencePoseOnPath)
% helperReferencePoseOnPath takes CG position but stanley control need
% front axle position, so lf is set.
% MPC case, it should be 0
assignin('base', 'refPts_offset', lf);       % reference point offset   (m)
assignin('base', 'controlMode', nvp.controlMode);  % default is stanley
assignin('base', 'numReferencePose', nvp.numReferencePose);     % default is one output, if this is increased, future trajectory is outputted
if nvp.controlMode == "mpc"
    assignin('base', 'refPts_offset', 0);       % reference point offset   (m)
    assignin('base', 'numReferencePose', PredictionHorizon);       % reference point offset   (m)
elseif nvp.controlMode == "mpc_mod"
    assignin('base', 'refPts_offset', lf);       % reference point offset   (m)
    assignin('base', 'numReferencePose', 1);       % reference point offset   (m)
    data_mat = load("pathFollowingUtil\mpc_base.mat");
    assignin('base', 'mpc_base', data_mat.mpc_base);
    data_mat = load("pathFollowingUtil\pfcsubsystem_data.mat");
    assignin('base', 'pfcsubsystem_data', data_mat.pfcsubsystem_data);
end
%% Create Simulink bus
helperCreateBusForTrajectoryFollowerWithRRScenario();
helperCreatePathTargetBus(nvp.MaxPathPoints);

if exist("egoInitialPose", "var")
    [egoInitialPoseCG, egoInitialPoseRear] = egoVehicleInfoForRear(egoInitialPose, lr);
    assignin('base', 'egoInitialPoseCG', egoInitialPoseCG);
    assignin('base', 'egoInitialPoseRear', egoInitialPoseRear);
    hegihtMap = "flat";
end

%% for multibody setting
if exist("egoInitialPose", "var") && nvp.MultibodySetting
    addpath(genpath('multibodyUtil'));
    [VehicleData, Camera] = setupMultibodyVehicleSpec();
    [InitVehicle, SceneData] = setMultibodyVehicleInitialPose(egoInitialPose, VehicleData);
    assignin('base',"InitVehicle",InitVehicle);
    assignin('base',"SceneData",SceneData);
    assignin('base',"VehicleData",VehicleData);
    assignin('base',"Camera",Camera);
    assignin('base',"vehicleVariant", "Multibody");
    assignin('base',"useDynamicsStanley", true);
end
end

function [InitVehicle, SceneData] = setMultibodyVehicleInitialPose(egoInitialPose, VehicleData)

    lf = evalin('base', 'lf');
    % lr = evalin('base', 'lr');

    rotMat = eul2rotm(deg2rad([egoInitialPose.Yaw, egoInitialPose.Pitch, egoInitialPose.Roll]), "ZYX");
    positionFA = transpose(rotMat * [lf; 0; 0]) + egoInitialPose.Position;

    % Defaults for vehicle initial position in World coordinate frame
    InitVehicle.Vehicle.px = positionFA(1);  % m
    InitVehicle.Vehicle.py = positionFA(2);  % m
    InitVehicle.Vehicle.pz = positionFA(3);  % m
    
    % Defaults for vehicle initial translational velocity
    % Represented in vehicle coordinates: 
    %    +vx is forward, +vy is left, +vz is up in initial vehicle frame
    InitVehicle.Vehicle.vx  = egoInitialPose.Velocity(1); %m/s
    InitVehicle.Vehicle.vy  =  egoInitialPose.Velocity(2);  %m/s
    InitVehicle.Vehicle.vz  =  egoInitialPose.Velocity(3);  %m/s
    
    % Defaults for vehicle initial orientation
    % Represented in vehicle coordinates, yaw-pitch-roll applied intrinsically
    InitVehicle.Vehicle.yaw   = deg2rad(egoInitialPose.Yaw);  % rad
    InitVehicle.Vehicle.pitch = deg2rad(egoInitialPose.Pitch);  % rad
    InitVehicle.Vehicle.roll  = deg2rad(egoInitialPose.Roll);  % rad
    
    % Default is flat road surface in x-y plane
    % SceneData = evalin('base','SceneData');
    SceneData.Reference.yaw   = 0 * pi/180;
    SceneData.Reference.pitch = 0 * pi/180;
    SceneData.Reference.roll  = 0 * pi/180;

    % VehicleData = evalin('base', 'VehicleData');
    
    % Set initial position and speed of vehicle and wheels
    InitVehicle.Wheel.wFL = InitVehicle.Vehicle.vx/VehicleData.TireDataF.param.DIMENSION.UNLOADED_RADIUS; %rad/s
    InitVehicle.Wheel.wFR = InitVehicle.Vehicle.vx/VehicleData.TireDataF.param.DIMENSION.UNLOADED_RADIUS; %rad/s
    InitVehicle.Wheel.wRL = InitVehicle.Vehicle.vx/VehicleData.TireDataR.param.DIMENSION.UNLOADED_RADIUS; %rad/s
    InitVehicle.Wheel.wRR = InitVehicle.Vehicle.vx/VehicleData.TireDataR.param.DIMENSION.UNLOADED_RADIUS; %rad/s
    
    % Update structures in MATLAB workspace
    assignin('base',"InitVehicle",InitVehicle)
    assignin('base',"SceneData",SceneData)
end

function [VehicleData, Camera] = setupMultibodyVehicleSpec()
    % Parameters for example sm_vehicle_2axle_heave_roll.slx
    % Copyright 2021-2024 The MathWorks, Inc.
    
    m = evalin('base','m');                 % Total mass of vehicle                          (kg)
    Iz = evalin('base','Iz');                 % Yaw moment of inertia of vehicle               (m*N*s^2)
    Cf = evalin('base','Cf');                % Cornering stiffness of front tires             (N/rad)
    Cr = evalin('base','Cr');                % Cornering stiffness of rear tires              (N/rad)
    lf = evalin('base','lf');               % Longitudinal distance from c.g. to front tires (m)
    lr = evalin('base','lr');               % Longitudinal distance from c.g. to rear tires  (m)
    steeringGain = evalin('base', 'steeringGain');       % Steering gain
    steerLimit = evalin('base', 'steerLimit');     % Max steering limit deg
    steerGear = evalin('base', 'steerGear');     % steering gear ratio
    % Vehicle body parameters
    % Vehicle reference point is point directly between 
    %         tire contact patches on front wheels
    VehicleData.Body.FA = 0;               % m
    VehicleData.Body.RA = -(lf+lr);              % m
    VehicleData.Body.CG = [-lf 0 0]; % m
    VehicleData.Body.Geometry = [1.25 0.9850 -0.2353]; % m
    VehicleData.Body.Mass          = m;            % kg
    VehicleData.Body.Inertias      = [600 3000 Iz]; % kg*m^2
    VehicleData.Body.Color     = [0.4 0.6 1.0]; % RGB
    VehicleData.Body.Color_2   = [0.87 0.57 0.14]; % RGB
    VehicleData.Body.Opacity = 1.0;
    
    % Front suspension parameters
    % Two degrees of freedom for axle (heave, roll), spin DOF for wheels
    % Ackermann steering
    VehicleData.SuspF.Heave.Stiffness = 40000;  % N/m
    VehicleData.SuspF.Heave.Damping   = 3500;   % N/m
    VehicleData.SuspF.Heave.EqPos     = -0.2;   % m
    VehicleData.SuspF.Heave.Height    = 0.1647; % m
    VehicleData.SuspF.Roll.Stiffness  = 66000;  % N*m/rad
    VehicleData.SuspF.Roll.Damping    = 2050;   % N*m/(rad/s)
    VehicleData.SuspF.Roll.Height     = 0.0647; % m
    VehicleData.SuspF.Roll.EqPos      = 0;      % rad
    
    % Separation of wheels on this axle
    VehicleData.SuspF.Track           = 1.6;    % m
    
    % Unsprung mass - radius and length for visualization only
    VehicleData.SuspF.UnsprungMass.Mass = 95;         % kg
    VehicleData.SuspF.UnsprungMass.Inertia = [1 1 1]; % kg*m^2
    VehicleData.SuspF.UnsprungMass.Height = 0.355;    % m
    VehicleData.SuspF.UnsprungMass.Radius = 0.1;      % m
    VehicleData.SuspF.UnsprungMass.Length = 1.6;      % m
    
    % Hub height should be synchronized with tire parameters
    VehicleData.TireDataF.filename = 'KT_MF_Tool_245_60_R16.tir';
    VehicleData.TireDataF.param    = simscape.multibody.tirread(which(VehicleData.TireDataF.filename));
    VehicleData.SuspF.Hub.Height   = VehicleData.TireDataF.param.DIMENSION.UNLOADED_RADIUS; % m
    % Rim mass and inertia typically not part of .tir file
    VehicleData.RimF.Mass          = 10;    % kg
    VehicleData.RimF.Inertia       = [1 2]; % kg*m^2
    
    VehicleData.SuspF.SteerRatio = steerGear; % m
    
    % Rear suspension parameters
    % Two degrees of freedom for axle (heave, roll), spin DOF for wheels
    VehicleData.SuspR.Heave.Stiffness  = 50000;  % N/m
    VehicleData.SuspR.Heave.Damping    = 3500;   % N/m
    VehicleData.SuspR.Heave.EqPos      = -0.16;  % m
    VehicleData.SuspR.Heave.Height     = 0.1647; % m
    VehicleData.SuspR.Roll.Stiffness   = 27500;  % N*m/rad
    VehicleData.SuspR.Roll.Damping     = 2050;   % N*m/(rad/s)
    VehicleData.SuspR.Roll.Height      = 0.1147; % m
    VehicleData.SuspR.Roll.EqPos      = 0;      % rad
    
    % Separation of wheels on this axle
    VehicleData.SuspR.Track            = 1.6;    % m
    
    % Unsprung mass - radius and length for visualization only
    VehicleData.SuspR.UnsprungMass.Mass    = 90;      % kg
    VehicleData.SuspR.UnsprungMass.Inertia = [1 1 1]; % kg*m^2
    VehicleData.SuspR.UnsprungMass.Height  = 0.355;   % m
    VehicleData.SuspR.UnsprungMass.Length  = 1.6;     % m
    VehicleData.SuspR.UnsprungMass.Radius  = 0.1;     % m
    
    % Hub height should be synchronized with tire parameters
    VehicleData.TireDataR.filename = 'KT_MF_Tool_245_60_R16.tir';
    VehicleData.TireDataR.param    = simscape.multibody.tirread(which(VehicleData.TireDataR.filename));
    VehicleData.SuspR.Hub.Height   = VehicleData.TireDataF.param.DIMENSION.UNLOADED_RADIUS; % m
    
    % Rim mass and inertia typically not part of .tir file
    VehicleData.RimR.Mass          = 10;    % kg
    VehicleData.RimR.Inertia       = [1 2]; % kg
    
    %% Camera data
    Camera =  sm_vehicle_camera_frames_car_3m;
end

function [egoInitialPoseCG, egoInitialPoseRear] = egoVehicleInfoForRear(egoInitialPose, lr)
    egoInitialPoseRear = Simulink.Bus.createMATLABStruct("BusVehicleInfo");

    %[x, y, yaw_deg, z, pitch_deg]
    poseMatrix = eul2rotm(deg2rad([egoInitialPose.Yaw, egoInitialPose.Pitch, egoInitialPose.Roll]), "ZYX");
    positionRear = egoInitialPose.Position + transpose(poseMatrix * [-lr; 0; 0]);

    poseRear = [positionRear(1), positionRear(2), egoInitialPose.Yaw, positionRear(3), egoInitialPose.Pitch];
    egoInitialPoseRear.CurrVelocity = norm(egoInitialPose.Velocity);
    egoInitialPoseRear.CurrYawRate = egoInitialPose.AngularVelocity(3);
    egoInitialPoseRear.Direction = 1;
    egoInitialPoseRear.CurrSteer = 0;
    egoInitialPoseRear.CurrPose = poseRear;

    poseCG = [egoInitialPose.Position(1), egoInitialPose.Position(2), egoInitialPose.Yaw, egoInitialPose.Position(3), egoInitialPose.Pitch];
    egoInitialPoseCG = egoInitialPoseRear;
    egoInitialPoseCG.CurrPose = poseCG;
end
function egoVehDyn = egoVehicleDynamicsParamsFromRR(ego, egoActor)
% Ego pose for vehicle dynamics from RoadRunner Scenario.
egoVehDyn.X0  =  ego.Position(1); % (m)
egoVehDyn.Y0  =  -ego.Position(2); % (m)
egoVehDyn.Z0  = -ego.Position(3); % (m)
egoVehDyn.VX0 =  ego.Velocity(1); % (m/s)
egoVehDyn.VY0 =  -ego.Velocity(2); % (m/s)

% Adjust sign and unit of yaw
egoVehDyn.Yaw0 = -deg2rad(ego.Yaw); % (rad)
egoVehDyn.Pitch0 = -deg2rad(ego.Pitch); % (rad)
egoVehDyn.Roll0 = deg2rad(ego.Roll); % (rad)

% Longitudinal velocity
egoVehDyn.VLong0 = hypot(egoVehDyn.VX0,egoVehDyn.VY0); % (m/sec)

% Distance from center of gravity to axles
egoVehDyn.CGToFrontAxle = egoActor.Length/2 - egoActor.FrontOverhang;
egoVehDyn.CGToRearAxle  = egoActor.Length/2 - egoActor.RearOverhang;
end

function actorPose = helperGetActorPose(worldActors, actorID, actorSpeed)
% helperGetActorPose calculates actor pose of the actor correspoding to
% provided actorID using scenario information from RoadRunner Scenario. It
% takes actor speed as an input which is used to update velocity in the
% actor pose.

% Find actor with ActorID = actorID
actorIDs = arrayfun(@(actors) str2double(actors.actor_runtime.id),worldActors,'UniformOutput',true);
idx = actorIDs == actorID;
if nnz(idx) == 1
    % Get pose matrix.
    m = worldActors(idx).actor_runtime.pose.matrix;
    c1 = m.col0;
    c2 = m.col1;
    c3 = m.col2;
    c4 = m.col3;

    pose = [c1.x c2.x c3.x c4.x; ...
        c1.y c2.y c3.y c4.y; ...
        c1.z c2.z c3.z c4.z; ...
        c1.w c2.w c3.w c4.w];

    position = pose(1:3,4)';
    heading = rotm2eul(pose(1:3, 1:3), "ZYX"); % The default order for Euler angle rotations is "ZYX"

    % Adjust yaw due to difference in the actor's starting orientation
    yaw = rad2deg(heading(1))+90;
    if yaw > 180
        yaw = yaw-360;
    end

    velocity = [actorSpeed*cosd(yaw) actorSpeed*sind(yaw) 0];

    actorPose = struct(...
        'ActorID', double(actorID), ...
        'Position', position, ...         % m
        'Velocity', velocity, ...         % m/s
        'Roll', rad2deg(heading(2)), ...  % deg 補正
        'Pitch', -rad2deg(heading(3)), ... % deg 正の時下向き、負の時上向き
        'Yaw', yaw, ...                   % deg
        'AngularVelocity', [0 0 0]);      % deg/s
else
    error("Actor with ActorID:%d does not exist",actorID);
end
end

function actorProfiles = helperGetActorProfiles(worldActors)
% helperGetActorProfiles calculates driving scenario actor profiles
% for all the actors using actor spec information from RoadRunner
% Scenario.

numActors = length(worldActors);

% Initialize actorProfiles struct.
actorProfile = struct(...
    'ActorID',0,...
    'ClassID',1,...
    'Length',0,...
    'Width',0,...
    'Height',0,...
    'OriginOffset',[0 0 0],...
    'FrontOverhang',0,...
    'RearOverhang',0,...
    'Color',[0 0 0],...
    'bbx',zeros(2,3));

actorProfiles = repmat(actorProfile, 1, numActors);

for i = 1:numActors
    id = str2double(worldActors(i).actor_spec.id);

    % Get actor bounding box
    min = worldActors(i).actor_spec.bounding_box.min;
    max = worldActors(i).actor_spec.bounding_box.max;
    bbx = [min.x min.y min.z; ...
        max.x max.y max.z];
    actorProfiles(i).ActorID = id;

    % Calculate length and width from bounding boxes
    actorProfiles(i).Length  = max.y - min.y;
    actorProfiles(i).Width   = max.x - min.x;
    actorProfiles(i).Height  = max.z;
    actorProfiles(i).bbx     = bbx;

    % Update the Color, FrontOverhang, RearOverhang and OriginOffset values
    % in the actor profiles if it is a vehicle. For character, these values
    % are not valid.
    if ~isempty(worldActors(i).actor_spec.vehicle_spec) && isempty(worldActors(i).actor_spec.character_spec)
        % Get actor color
        color = worldActors(i).actor_spec.vehicle_spec.paint_color;
        r = double(color.r)/255;
        g = double(color.g)/255;
        b = double(color.b)/255;
        actorProfiles(i).Color   = [r g b];
        % Calculate FrontOverhang and RearOverhang
        actorProfiles(i).FrontOverhang = actorProfiles(i).Length/2 - worldActors(i).actor_spec.vehicle_spec.wheels(1).wheel_offset.y;
        actorProfiles(i).RearOverhang = actorProfiles(i).Length/2 + worldActors(i).actor_spec.vehicle_spec.wheels(3).wheel_offset.y;
    end
end
end