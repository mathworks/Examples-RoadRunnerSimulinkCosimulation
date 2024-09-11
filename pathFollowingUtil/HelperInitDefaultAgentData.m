classdef HelperInitDefaultAgentData
    % HelperInitDefaultAgentData Specifies values for common agent data 
    % types.
    %
    % NOTE: The name of this class and it's functionality may change 
    % without notice in a future release, or the class itself may be 
    % removed.
    %
    
    % Copyright 2021 The MathWorks, Inc.
    properties(Constant)
        ID = repmat('0', 1, 36);
    end

    methods(Static)
        % /////////////////////////////////////////////////////////////////
        function out = actorRuntime()
            %ACTORRUNTIME Creates the default actor runtime struct
            out = struct(...
                'ActorID', uint64(0), ...
                'Pose', zeros(4,4), ...
                'Velocity', zeros(1,3), ...
                'AngularVelocity', zeros(1,3));
        end

        % /////////////////////////////////////////////////////////////////
        function out = actorRuntimes()
            %ACTORRUNTIME Creates the default actor runtimes struct
            out = repmat(HelperInitDefaultAgentData.actorRuntime, 0, 1);
            coder.varsize('out', [Inf 1], [1 0]);
        end

        % /////////////////////////////////////////////////////////////////
        function out = alignmentSingle()
            %ALIGNMENTSINGLE Creates the default alignment struct
            out = struct(...
                'ID', HelperInitDefaultAgentData.ID, ...
                'Alignment', ImporterTransferMap.Alignment(0));
        end

        % /////////////////////////////////////////////////////////////////
        function out = alignmentMultiple()
            %ALIGNMENTMULTIPLE Creates the default alignment struct
            out = repmat(HelperInitDefaultAgentData.alignmentSingle, 2, 1);
            coder.varsize('out', [Inf 1], [1 0]);
        end

        % /////////////////////////////////////////////////////////////////
        function out = controlPoints()
            %CONTROLPOINTS Returns the default controlPoints struct
            out = struct(...
                'x', 0, ...
                'y', 0, ...
                'z', 0);
            coder.varsize('out.x', [Inf 1], [1 0]);
            coder.varsize('out.y', [Inf 1], [1 0]);
            coder.varsize('out.z', [Inf 1], [1 0]);
        end

        % /////////////////////////////////////////////////////////////////
        function out = geometry()
            %GEOMETRY Returns the default geometry struct
            out = struct('ControlPoints',HelperInitDefaultAgentData.controlPoints);
        end

        % /////////////////////////////////////////////////////////////////
        function out = lane()
            %LANE Returns the default lane struct
            out = struct(...
                'ID', HelperInitDefaultAgentData.ID, ...
                'TravelDir', ImporterTransferMap.TravelDir(0),...
                'LaneType', ImporterTransferMap.LaneTypes(0),...
                'LeftLaneBoundary', HelperInitDefaultAgentData.alignmentSingle,...
                'RightLaneBoundary', HelperInitDefaultAgentData.alignmentSingle,...
                'SuccessorLanes', HelperInitDefaultAgentData.alignmentMultiple,...
                'PredecessorLanes', HelperInitDefaultAgentData.alignmentMultiple,...
                'Geometry', HelperInitDefaultAgentData.geometry);
            coder.varsize('out.SuccessorLanes', [Inf 1], [1 0]);
            coder.varsize('out.PredecessorLanes', [Inf 1], [1 0]);
        end

        % /////////////////////////////////////////////////////////////////
        function out = lanes()
            %LANES Returns the default lanes struct
            out = repmat(HelperInitDefaultAgentData.lane,0,1);
            coder.varsize('out', [Inf 1], [1 0]);
        end
        
        % /////////////////////////////////////////////////////////////////
        function out = laneBoundary()
            %LANEBOUNDARY Returns the default lane Boundary struct
            out = struct(...
                'ID', HelperInitDefaultAgentData.ID,...
                'Geometry', HelperInitDefaultAgentData.geometry);
        end

        % /////////////////////////////////////////////////////////////////
        function out = laneBoundaries()
            %LANEBOUNDARIES Returns the default lanes struct
            out = repmat(HelperInitDefaultAgentData.laneBoundary,0,1);
            coder.varsize('out', [Inf 1], [1 0]);
        end       

        % /////////////////////////////////////////////////////////////////
        function out = laneChangeAction()
            %LANECHANGEACTION Returns the default geometry struct
            out = struct(...
                "ActorID", uint64(0),...
                "PhaseInterval", EnumPhaseInterval(0),...
                "LaneValue", 0,...
                "LaneComparison", EnumLaneComparison(0),...
                "RefActorID", 0,...
                "Dynamics", HelperInitDefaultAgentData.transitionDynamics);
        end

        % /////////////////////////////////////////////////////////////////
        function out = speedAction()
            %SPEEDACTION Returns the default speed action struct
            out = struct(...
                "ActorID", uint64(0),...
                "PhaseInterval", EnumPhaseInterval(0),...
                "SpeedValue", 0,...
                "SpeedComparison", EnumSpeedComparison(0),...
                "RefActorID", 0,...
                "Dynamics", HelperInitDefaultAgentData.transitionDynamics,...
                "ReferenceSamplingMode", EnumReferenceSamplingMode.Unspecified);
        end

        % /////////////////////////////////////////////////////////////////
        function out = transitionDynamics()
            %LANECHANGEACTION Returns the default geometry struct
            out = struct(...
                "Dimension", EnumDynamicsDimension(0),...
                "Shape", EnumDynamicsShape(0),...
                "Value", 0);
        end
        
        % /////////////////////////////////////////////////////////////////
        function out = vehicleStates()
            % VEHICLESTATES Returns the default vehicle states struct
            out.ActorID  = uint32(0);
            % global states related to refernece point
            out.Position = zeros(1,3);
            out.Velocity = zeros(1,3);
            out.Acceleration = zeros(1,3);
            out.Roll = 0;
            out.Pitch = 0;
            out.Yaw = 0;
            out.AngularVelocity = zeros(1,3);
            out.LongitudinalSpeed = 0;
            out.LongitudinalAccel = 0;
            out.SteeringAngle     = 0;
            out.NumWheels = 0;
            % global states related to individual wheels
            singleWheel.Position = zeros(1,3);
            singleWheel.Roll = 0;
            singleWheel.Pitch = 0;
            singleWheel.Yaw   = 0;
            singleWheel.Rotation = 0;
            out.WheelPoses = repmat(singleWheel,18,1);
        end
        
        % /////////////////////////////////////////////////////////////////
        function out = vehicleSpec()
            out = struct;
            out.ActorSpec = struct;
            out.ActorSpec.ActorID = 0;
            out.ActorSpec.BoundingBox = struct;
            out.ActorSpec.BoundingBox.Min = zeros(1,3);
            out.ActorSpec.BoundingBox.Max = zeros(1,3);
            out.PaintColor = struct;
            out.PaintColor.r = 255;
            out.PaintColor.g = 255;
            out.PaintColor.b = 255;
            out.PaintColor.a = 255;
            out.NumWheels = 0;
            singleWheel.AxleIndex = 0;
            singleWheel.WheelOffset = zeros(1,3);
            singleWheel.WheelRadius = 0;
            out.Wheels = repmat(singleWheel,1,18);
        end
        
        % /////////////////////////////////////////////////////////////////
        function out = lateralAction()
            out = struct("IsNew", false,...
                "IsStep", false,...
                "Dimension", EnumDynamicsDimension.Unspecified,...
                "InitialLateralOffset", 0,...
                "InitialLateralVelocity", 0,...
                "TerminalLateralOffset", 0,...
                "TerminalLateralVelocity", 0,...
                "TerminalDimensionValue", 0);
        end
        
        % /////////////////////////////////////////////////////////////////
        function out = referencePath(numPoints)
            out = struct('s', zeros(numPoints, 1), ...
                            'x', zeros(numPoints, 1), ...
                            'y', zeros(numPoints, 1), ...
                            'theta', zeros(numPoints, 1), ...
                            'kappa', zeros(numPoints, 1), ...
                            'elev', zeros(numPoints, 1), ...
                            'grade', zeros(numPoints, 1), ...
                            'bank', zeros(numPoints, 1)); %%#ok<NASGU> 
            coder.varsize("out.x", [inf 1], [1 0]);
            coder.varsize("out.y", [inf 1], [1 0]);
            coder.varsize("out.s", [inf 1], [1 0]);
            coder.varsize("out.theta", [inf 1], [1 0]);
            coder.varsize("out.kappa", [inf 1], [1 0]);
            coder.varsize("out.elev", [inf 1], [1 0]);
            coder.varsize("out.grade", [inf 1], [1 0]);
            coder.varsize("out.bank", [inf 1], [1 0]);
        end

        % /////////////////////////////////////////////////////////////////
        function out = VehicleStruct()
            %VEHICLESTRUCT Returns the default geometry struct
            out = struct('TrackWidth', 1.2, ...
                         'WheelBase', 0.8, ...
                         'Type', 'CR', ...
                         'Drive', '4WD', ...
                         'FrontAxleWheelCount', 2, ...
                         'AxleCount', 2, ...
                         'EdgeCount', 4, ...
                         'PreviousPose', eye(4, 4), ...
                         'WheelCount', 4, ...
                         'Dimensions', zeros(1, 3), ...
                         'Roll', 0, ...
                         'Pitch', 0, ...
                        'Yaw', 0);
        end

        % /////////////////////////////////////////////////////////////////
        function out = WheelStruct()
            %WHEELSTRUCT Returns the default geometry struct
            out = struct('Pose', eye(4,4), ...
                               'Location', zeros(3,1), ...
                               'AxleIndex', -1, ...    
                               'Radius', 1, ...
                               'Pitch', 0, ... 
                              'Steer', 0, ...
                               'Elevation', 0);
        end
    end

end

