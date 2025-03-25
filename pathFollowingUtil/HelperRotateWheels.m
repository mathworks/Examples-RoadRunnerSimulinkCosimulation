classdef HelperRotateWheels < matlab.System
    % HelperRotateWheels Updates the wheel rotation of the vehicle. 
    % HelperRotateWheels uses the actor id, steering angle, wheel spin
    % angle and wheel inclination angle as inputs and updates the wheel
    % poses of the vehicle in RoadRunner using the setAttribute function.
    %
    % NOTE: The name of this System Object and its functionality may
    % change without notice in a future release,
    % or the System Object itself may be removed.

    % Copyright 2023 The MathWorks, Inc.
    
    properties (Access = private)
        % ActorSimulation object of all actors in the scenarios.
        AllActorSimulationObj
    end

    methods (Access = protected)

        function setupImpl(obj)
            % Get the ActorSimulation object of the actors in the scenario
            % from the base workspace. If this variable doesnt exist in the base workspace 
            % then get the ActorSimulation object from Simulink.ScenarioSimulation 
            varList = evalin('base','who');
            for i = 1:numel(varList)
                if varList{i} == "allActorSimulationObj"
                    obj.AllActorSimulationObj = evalin("base","allActorSimulationObj");
                    break;
                end
            end
            if isempty(obj.AllActorSimulationObj)
                ss = Simulink.ScenarioSimulation.find('ScenarioSimulation');
                obj.AllActorSimulationObj = ss.get('ActorSimulation');% This will return all the actors currently exist in the simulation
            end

        end

        function stepImpl(obj,actorID,wheelPitch,wheelSteering,wheelRoll)
            % Find the ActorSimulation object for the vehicle based on
            % actor ID.
            for i = 2:length(obj.AllActorSimulationObj)
                actorSimulationObj = obj.AllActorSimulationObj{i};
                id = actorSimulationObj.getAttribute('ID');
                if id == actorID
                    egoActorSimObj = actorSimulationObj;
                end
            end

            % Get the wheel poses 
            wheelPoses = getAttribute(egoActorSimObj,'WheelPoses');

            
            n = size(wheelPoses,3);
            for i = 1:n
                % Compute the rotation matrix based using the vehicle's
                % steering angle and angular position.
                rotm = eul2rotm([wheelSteering(i) wheelRoll(i) wheelPitch(i)]);
                % Update the rotation matrix
                wheelPoses(1:3,1:3,i) = rotm;
            end

            % Write the updated wheel poses back to roadrunner
            setAttribute(egoActorSimObj,"WheelPoses",wheelPoses);
        end
    end

    methods (Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end
end
