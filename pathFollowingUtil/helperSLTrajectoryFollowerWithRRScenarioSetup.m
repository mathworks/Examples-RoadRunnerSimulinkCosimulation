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
assignin('base','tau',             0.5);     % Time constant for longitudinal dynamics 1/s/(tau*s+1)
assignin('base','time_gap',        1.5);     % Time gap               (s)
assignin('base','default_spacing', 10);      % Default spacing        (m)
assignin('base','max_ac',          2);       % Maximum acceleration   (m/s^2)
assignin('base','min_ac',          -3);      % Minimum acceleration   (m/s^2)
assignin('base','max_steer',       0.26);    % Maximum steering       (rad)
assignin('base','min_steer',       -0.26);   % Minimum steering       (rad) 
assignin('base','PredictionHorizon', 30);    % Prediction horizon     
assignin('base','v0_ego', 0);                % Initial longitudinal velocity (m/s)
assignin('base','tau2', 0.07);               % Longitudinal time constant (brake)             (N/A)
assignin('base','max_dc', 10);               % Maximum deceleration   (m/s^2)

%% Dynamics modeling parameters
assignin('base','m',  1575);                 % Total mass of vehicle                          (kg)
assignin('base','Iz', 2100);                 % Yaw moment of inertia of vehicle               (m*N*s^2)
assignin('base','Cf', 19000);                % Cornering stiffness of front tires             (N/rad)
assignin('base','Cr', 33000);                % Cornering stiffness of rear tires              (N/rad)
assignin('base','lf', 1.5130);               % Longitudinal distance from c.g. to front tires (m)
assignin('base','lr', 1.3050);               % Longitudinal distance from c.g. to rear tires  (m)
assignin('base', 'steeringGain', 0.8);       % Steering gain
assignin('base', 'steerLimit', 30);     % Max steering limit
assignin('base', 'steerGear', 18);     % steering gear ratio

%% Create Simulink bus
helperCreateBusForTrajectoryFollowerWithRRScenario
helperCreatePathTargetBus(nvp.MaxPathPoints)

end
function egoVehDyn = egoVehicleDynamicsParamsFromRR(ego, egoActor)
% Ego pose for vehicle dynamics from RoadRunner Scenario.
egoVehDyn.X0  =  ego.Position(2); % (m)
egoVehDyn.Y0  =  ego.Position(1); % (m)
egoVehDyn.Z0  = -ego.Position(3); % (m)
egoVehDyn.VX0 =  ego.Velocity(2); % (m/s)
egoVehDyn.VY0 =  -ego.Velocity(1); % (m/s)

% Adjust sign and unit of yaw
egoVehDyn.Yaw0 = -deg2rad(ego.Yaw-90); % (rad)
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
    heading = rotm2eul(pose(1:3, 1:3)); % The default order for Euler angle rotations is "ZYX"

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
        'Roll', rad2deg(heading(3)), ...  % deg
        'Pitch', rad2deg(heading(2)), ... % deg
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