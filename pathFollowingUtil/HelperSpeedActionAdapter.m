classdef HelperSpeedActionAdapter < matlab.System
    % HelperSpeedActionAdapter Sets current speed based on speed action 
    % command.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    %
    
    % Copyright 2021 The MathWorks, Inc.

    properties(Access = private)

        % Speed action command
        Command = HelperInitDefaultAgentData.speedAction;

        % Target and current speed
        TargetSpeed = 0;
        CurrentSpeed = 0;

        % Transition properties
        TransitionInitialSpeed = 0;
        TransitionCurrentValue = 0;
        TransitionRange = 0;
        TransitionAcceleration = 0;
        
        % Actor Id
        ActorID = 0;
        
        % Diagnostic message flag for negative velocity
        DiagnosticMsgFlagNegativeVelocity = false;
        
        % Diagnostic message flag for negative acceleration
        DiagnosticMsgFlagForNegativeAccel = false;

        % Diagnostic message flag for not finding reference actor in the
        % list
        DiagnosticMsgFlagForNoRefActor = false;

        % Vehicle Runtime
        AllVehicleStates;  

    end

    methods
        function obj = HelperSpeedActionAdapter(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})

            % Initialize Vehicle states
            SingleVehicleStates = HelperInitDefaultAgentData.actorRuntime;
            obj.AllVehicleStates = SingleVehicleStates; 
        end
    end

    methods(Access = protected)
        
        function [currentSpeed]= stepImpl(obj, timestep, stopVehicle, vehicleRuntime, msgSpeedAction, msgAllVehicleRuntime)
            % STEPIMPL Step function which returns speed for this time step
            
            % Update All Vehicle Runtime
            if numel(msgAllVehicleRuntime) > 0
                obj.AllVehicleStates = msgAllVehicleRuntime;
            end

            obj.ActorID = vehicleRuntime.ActorRuntime.ActorID;
            if stopVehicle
                % Stop vehicle if route is finished
                obj.CurrentSpeed = 0;
            else
                % Process speed commands
                updateCommand(obj, msgSpeedAction)
                % Control speed
                updateCurrentSpeed(obj, timestep);
            end
            
            % Return speed
            if obj.CurrentSpeed < 0
                if ~obj.DiagnosticMsgFlagNegativeVelocity
                    warning(['Invalid speed for the actor ',num2str(obj.ActorID),'. The actor remains stationary during simulation.']);
                    obj.DiagnosticMsgFlagNegativeVelocity = true;
                end
                currentSpeed = 0;
            else
                currentSpeed = obj.CurrentSpeed;
            end
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = [mfilename("class"), newline, newline];
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];
        end

        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
        end
        
        function updateCommand(obj, msgSpeedAction)
            % UPDATECOMMAND Updates command related parameters if new speed
            % action message is received

            numMsgSpeedActions = numel(msgSpeedAction);
            if numMsgSpeedActions > 0
                % Reset diagnostic message flags
                obj.DiagnosticMsgFlagNegativeVelocity = false;
                obj.DiagnosticMsgFlagForNegativeAccel = false;
                obj.DiagnosticMsgFlagForNoRefActor = false;

                % There should only be one message, but if there were more
                % then use the last one.
                cmd = msgSpeedAction(end);   

                obj.Command.ActorID = cmd.ActorAction.ActorID;
                obj.Command.Dynamics = cmd.TransitionDynamics;
                obj.Command.SpeedValue = cmd.SpeedTarget.SpeedValue;
                obj.Command.SpeedComparison = cmd.SpeedTarget.SpeedComparison;
                obj.Command.RefActorID = cmd.SpeedTarget.RefActorID;
                obj.Command.ReferenceSamplingMode = cmd.SpeedTarget.RefSamplingMode;

                % Update target speed property
                updateTargetSpeed(obj);

                % Update transition parameters
                updateTransitionParameters(obj);
            end

            % Check for continuous mode from speed action.
            inContinuousMode = obj.Command.ReferenceSamplingMode == EnumReferenceSamplingMode.Continuous && obj.Command.Dynamics.Dimension == EnumDynamicsDimension.Rate;

            % Set target speed if in continuous mode and speed has not
            % already been updated in this time step
            if inContinuousMode && numMsgSpeedActions == 0
                updateTargetSpeed(obj);
            end
            
        end

        function updateTransitionParameters(obj)
            % UPDATETRANSITIONPARAMETERS Update parameters for transitioning speed 
            
            % Set properties related to transitions
            obj.TransitionInitialSpeed = obj.CurrentSpeed;
            obj.TransitionCurrentValue = 0;

            % Update transition  based on acceleration, time, or distance
            cmd = obj.Command;
            transitionValue = cmd.Dynamics.Value; 

            switch cmd.Dynamics.Dimension
                
                case EnumDynamicsDimension.Rate
                    % With Acceleration
                    obj.TransitionAcceleration = transitionValue;
            
                case EnumDynamicsDimension.Time
                    switch cmd.Dynamics.Shape
                        case {EnumDynamicsShape.Linear,...
                              EnumDynamicsShape.Cubic}
                            obj.TransitionRange = transitionValue;
                            
                        otherwise 
                            % EnumDynamicsShape.Step
                    end
    
                case EnumDynamicsDimension.Distance
                    switch cmd.Dynamics.Shape
                        case {EnumDynamicsShape.Linear,...
                              EnumDynamicsShape.Cubic}
                            % Calculate transition time
                            dist = transitionValue;
                            transitionTime = 2 * dist /...
                                (obj.TransitionInitialSpeed + obj.TargetSpeed);
                            obj.TransitionRange = transitionTime;

                        otherwise 
                            % EnumDynamicsShape.Step
                    end
                otherwise 
                    % Absolute
            end
        end
        

        function updateTargetSpeed(obj)
            % UPDATETARGETSPEED Update target speed if using a speed-based control.
            if obj.Command.SpeedComparison == EnumSpeedComparison.SameAs || ...
               obj.Command.SpeedComparison == EnumSpeedComparison.FasterThan || ...
               obj.Command.SpeedComparison == EnumSpeedComparison.SlowerThan
                    % Continuous tracking mode
                    isVehiclePresent = false;
                    actorState = obj.getActorStates(obj.Command.RefActorID);
                    
					refSpeed = 0;
                    if actorState.ActorID == obj.Command.RefActorID
                        isVehiclePresent = true;
                        refSpeed = norm(actorState.Velocity);
                    end                    
                    % Set target speed to current speed of agent
                    targetSpeed = obj.Command.SpeedValue;
                    if isVehiclePresent
                        switch obj.Command.SpeedComparison
                            case EnumSpeedComparison.SameAs
                                targetSpeed = refSpeed;
                            case EnumSpeedComparison.FasterThan
                                targetSpeed = refSpeed + targetSpeed;
                            case EnumSpeedComparison.SlowerThan
                                targetSpeed = refSpeed - targetSpeed;
                        end
                    else
                        if ~obj.DiagnosticMsgFlagForNoRefActor
                            warning(['Target actor for the actor ',num2str(obj.ActorID),' is not found in the scenario.']);
                            obj.DiagnosticMsgFlagForNoRefActor = true;
                        end
                    end          
            else
                % EnumSpeedComparison.Absolute
                targetSpeed = obj.Command.SpeedValue;
            end
            obj.TargetSpeed = targetSpeed;
        end        

        function updateCurrentSpeed(obj, timestep)
            % UPDATECURRENTSPEED Updates current speed property
            cmd = obj.Command;
            switch cmd.Dynamics.Dimension
                
                case EnumDynamicsDimension.Rate
                    % With Acceleration
                    obj.stepSpeedConstAcceleration(timestep);

                case EnumDynamicsDimension.Time
                    % Step time for transition
                    transitionStepSize = timestep;
                    switch cmd.Dynamics.Shape
                        case EnumDynamicsShape.Linear
                            stepSpeedLinear(obj, transitionStepSize)
                        case EnumDynamicsShape.Cubic
                            stepSpeedCubic(obj, transitionStepSize)
                        otherwise 
                            % EnumDynamicsShape.Step
                            obj.CurrentSpeed = obj.TargetSpeed;
                    end

                case EnumDynamicsDimension.Distance
                    % Step distance for transition
                    transitionStepSize = timestep;
                    switch cmd.Dynamics.Shape
                        case EnumDynamicsShape.Linear
                            stepSpeedLinear(obj, transitionStepSize)
                        case EnumDynamicsShape.Cubic
                            stepSpeedCubic(obj, transitionStepSize)
                        otherwise 
                            % EnumDynamicsShape.Step
                            obj.CurrentSpeed = obj.TargetSpeed;
                    end

                otherwise 
                    % Absolute
                    obj.CurrentSpeed = obj.TargetSpeed;
            end

        end

        function SelectVehStates = getActorStates(obj,SelectActorID)
            SelectVehStates = HelperInitDefaultAgentData.vehicleStates;

            % Index of the actor in AllVehStates.
            selfActorIdx = 0;

            % Find the index of the requested actor.
            for i=1:numel(obj.AllVehicleStates)
                if(obj.AllVehicleStates(i).ActorID == SelectActorID)
                    selfActorIdx = i;
                    break;
                end
            end

            if selfActorIdx > 0
                SelectVehStates = updateVehicleState(obj, selfActorIdx);
            end
        end

        function stepSpeedLinear(obj, stepSize)
            % STEPSPEEDLINEAR Steps current speed with linear transition
            %  stepSize in time or distance since action command was received

            % Increment transition current value
            obj.TransitionCurrentValue = obj.TransitionCurrentValue + stepSize;
            if obj.TransitionCurrentValue >= obj.TransitionRange
                obj.CurrentSpeed = obj.TargetSpeed;
                return;
            end
            
            % Update speed
            obj.CurrentSpeed = obj.TransitionInitialSpeed + ...
                (obj.TargetSpeed - obj.TransitionInitialSpeed) * (obj.TransitionCurrentValue / obj.TransitionRange);
            
        end

        function stepSpeedCubic(obj, stepSize)
            % STEPSPEEDCUBIC Steps current speed with cubic transition
            %  stepSize in time or distance since action command was received
            
            % Increment transition current value
            obj.TransitionCurrentValue = obj.TransitionCurrentValue + stepSize;
            if obj.TransitionCurrentValue >= obj.TransitionRange
                obj.CurrentSpeed = obj.TargetSpeed;
                return;
            end
    
            % Update speed
            obj.CurrentSpeed = evaluateCubic(obj.TransitionInitialSpeed, obj.TargetSpeed, obj.TransitionRange, obj.TransitionCurrentValue);
        end

        function stepSpeedConstAcceleration(obj, timestep)
            % STEPSPEEDCONSTACCELERATION Steps current speed with constant acceleration
            requiredAccelSign = sign(obj.TargetSpeed - obj.CurrentSpeed);
            providedAccelSign = sign(obj.TransitionAcceleration);
            if(providedAccelSign ~= 0 &&  requiredAccelSign ~= 0 && providedAccelSign ~= requiredAccelSign && ~obj.DiagnosticMsgFlagForNegativeAccel)
                % warning(['Ignoring the sign of acceleration as it conflicts with attaining the required speed for the actor ',num2str(obj.ActorID),' in the Change Speed action.']);
                obj.DiagnosticMsgFlagForNegativeAccel = true;
            end
            tempSpeed = obj.CurrentSpeed + timestep * requiredAccelSign * abs(obj.TransitionAcceleration);
            if ((requiredAccelSign == -1 && tempSpeed <  obj.TargetSpeed) ||...
                (requiredAccelSign ==  1 && tempSpeed >= obj.TargetSpeed) ||...
                (requiredAccelSign ==  0))
                
                % Prevent overshoot of target speed
                tempSpeed = obj.TargetSpeed;
            end            
            
            obj.CurrentSpeed = tempSpeed;
        end

        function selectVehStates = updateVehicleState(obj, refActorIdx)            
            % Rotation to euler conversion
            eul = rotm2eul(obj.AllVehicleStates(refActorIdx).Pose(1:3,1:3),'XYZ');
            roll = eul(1)/pi*180;
            pitch = eul(2)/pi*180;
            yaw = eul(3)/pi*180+90;
            
            selectVehStates = HelperInitDefaultAgentData.vehicleStates;
            % Update vehicle states
            selectVehStates.ActorID             = uint32(obj.AllVehicleStates(refActorIdx).ActorID);
            selectVehStates.NumWheels           = 0;
            selectVehStates.Position(1,1)       = obj.AllVehicleStates(refActorIdx).Pose(1,4);
            selectVehStates.Position(1,2)       = obj.AllVehicleStates(refActorIdx).Pose(2,4);
            selectVehStates.Position(1,3)       = obj.AllVehicleStates(refActorIdx).Pose(3,4);
            selectVehStates.Velocity(1,1)       = obj.AllVehicleStates(refActorIdx).Velocity(1);
            selectVehStates.Velocity(1,2)       = obj.AllVehicleStates(refActorIdx).Velocity(2);
            selectVehStates.Velocity(1,3)       = obj.AllVehicleStates(refActorIdx).Velocity(3);
            selectVehStates.Acceleration(1,:)   = zeros(1,3);
            selectVehStates.Roll                = roll;
            selectVehStates.Pitch               = pitch;
            selectVehStates.Yaw                 = yaw;
            selectVehStates.AngularVelocity     = obj.AllVehicleStates(refActorIdx).AngularVelocity;
            selectVehStates.LongitudinalSpeed   = getLongitudinalSpeed(selectVehStates);
            selectVehStates.LongitudinalAccel   = 0;
            selectVehStates.SteeringAngle       = 0;

        end

        function interface = getInterfaceImpl(~)
            import matlab.system.interface.*;
            interface = [...
                Input("in1", Data), ... % Time step
                Input("in2", Data), ... % Route finished
                Input("in3", Data), ... % Agent State
                Input("in4", Message), ... % Actors runtime
                Input("in5", Message), ... % Speed action
                Input("in6", Message), ... % All Actor Attributes
                Output("out1", Data),...
                Output("out2", Data)];
        end
        
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end

function val = evaluateCubic(initialVal, finalVal, range, param)
% range = specified as OverTime or OverDistance in the action dialog
% param = relative value which has already been traveled 
%         0 <= param <= range
%         e.g. Current time - obj.InitialTime
%         e.g. Current distance traveled - obj.InitialDistance
 
% This function satisfies the following constraints
% f'(0) = 0
% f'(range) = 0
% f(0) = initialVal
% f(range) = finalVal
t1 =  2.0*(initialVal - finalVal)/(range*range*range)*param*param*param;
t2 = -3*(initialVal-finalVal)/(range*range)*param*param;
t4 = initialVal;
val = t1 + t2 + t4;
end

function LongitudinalSpeed = getLongitudinalSpeed(VehicleStates)
    % Compute vehicle longitudinal speed with assumptions vehicle
    % motion is nonholonomic and vehicle can travel in both forward
    % and backward directions
    LongitudinalSpeedMag = sqrt(VehicleStates.Velocity(1)^2+VehicleStates.Velocity(2)^2+VehicleStates.Velocity(3)^2);
    LongitudinalSpeedDir = atan2d(VehicleStates.Velocity(2),VehicleStates.Velocity(1));
    AngDiffYawVsVelo = wrapToPi((VehicleStates.Yaw-LongitudinalSpeedDir)/180*pi);
    if abs(AngDiffYawVsVelo)<pi/2
        LongitudinalSpeed = LongitudinalSpeedMag;
    else
        LongitudinalSpeed = -LongitudinalSpeedMag;
    end
end
