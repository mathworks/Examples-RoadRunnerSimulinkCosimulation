classdef HelperPathActionReader < matlab.System
    % HelperPathActionReader reads path of the ego vehicle from RoadRunner
    % using MATLAB APIs with an upper bound on number of points. It reads
    % the path of the ego vehicle at the initial time step and outputs the
    % same path at each time step during the simulation.
    %
    % NOTE: The name of this System object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    %
    
    % Copyright 2022-2023 The MathWorks, Inc.
    properties (Access = public)
        sampleTime = -1;
    end
    properties (Access = private)
        ActorObj
        PathTarget
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Find the ActorSimulation object of the runtime actor
            obj.ActorObj = Simulink.ScenarioSimulation.find('ActorSimulation', 'SystemObject', obj);
            % Initialize PathTarget
            obj.PathTarget = Simulink.Bus.createMATLABStruct('BusPathTarget');
        end

        function PathTarget = stepImpl(obj)
            % Get path action
            pathAction = obj.ActorObj.getAction('PathAction');

            % Read path from path action
            if ~isempty(pathAction)
                pathTarget = Simulink.Bus.createMATLABStruct('BusPathTarget');

                % Get number of points to output
                if pathAction.PathTarget.NumPoints>length(pathTarget.Path)
                    numPoints = length(pathTarget.Path);
                    warning("Reading first "+num2str(numPoints)+" out of "+num2str(pathAction.PathTarget.NumPoints)+" ego waypoints from the path action." + ...
                        " To read all ego waypoints, set the value of MaxPathPoints argument of helperSLTrajectoryFollowerWithRRScenarioSetup function to "+num2str(pathAction.PathTarget.NumPoints)+" or more.");
                else
                    numPoints = pathAction.PathTarget.NumPoints;
                end

                pathTarget.Path(1:numPoints,:) = pathAction.PathTarget.Path(1:numPoints,:);
                pathTarget.NumPoints = uint64(numPoints);
                pathTarget.HasTimings = pathAction.PathTarget.HasTimings;
    
                if pathAction.PathTarget.HasTimings
                    pathTarget.Timings(1,1:numPoints) = pathAction.PathTarget.Timings(1:numPoints,:);
                end
                obj.PathTarget = pathTarget;
            end

            PathTarget = obj.PathTarget;
        end

        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
        end

        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
        end

        function out = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "Bus: BusPathTarget";
        end

        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
        end
    end

    methods (Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
        function sts = getSampleTimeImpl(obj)
            if obj.sampleTime == -1
                sts = createSampleTime(obj);
            else
                sts = createSampleTime(obj,'Type','Discrete', ...
                    'SampleTime', obj.sampleTime);
            end
        end
    end
end
