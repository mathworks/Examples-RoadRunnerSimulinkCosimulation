classdef filterActorPose < matlab.System
    %slexMessageArrivalReceiver   Retrieve all available messages from Queue block.

    % Copyright 2020 The MathWorks Inc.

    properties (Nontunable)
        SampleTime = -1;
        ActorID = 1;
        OutputBusName = "BusActorPose";
    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function interface = getInterfaceImpl(obj)
            import matlab.system.interface.*;
            interface = [Input("In1", Message), ...
                Output("Out1", Data)];
        end

        function filteredActor = stepImpl(obj, msg)

            actorIDs = vertcat(msg.ActorID);
            idx_filter = (actorIDs == obj.ActorID);

            if ~any(idx_filter)
                filteredActor = Simulink.ActorSimulation.createMATLABStruct(obj.OutputBusName);
            else
                filteredActor = msg(idx_filter);
            end
            % filteredActor = msg(idx_filter);
        end

        function sts = getSampleTimeImpl(obj)
            if obj.SampleTime == -1
                sts = createSampleTime(obj,'ErrorOnPropagation','Controllable');
            else
                sts = createSampleTime(obj,'Type','Discrete', ...
                    'SampleTime',obj.SampleTime);
            end
        end

        function releaseImpl(obj)
        end

        function varargout = getOutputSizeImpl(obj)
            varargout{1} = 1;
        end

        function varargout = getOutputDataTypeImpl(obj)
            varargout{1} = obj.OutputBusName;
        end

        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
        end

        function varargout = isOutputFixedSizeImpl(obj)
            varargout{1} = true;
        end

    end
    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            % Specify the simulation mode for the MATLAB System block
            simMode = 'Interpreted execution';
        end
    end
    %
    % methods(Access = protected, Static)
    %     function header = getHeaderImpl
    %         % Define header panel for System block dialog
    %         headerText = ['Retrieve all available messages from ', ...
    %             'upstream Queue block'];
    %         header = matlab.system.display.Header(...
    %             'Title', 'Message Receiver', ...
    %             'Text', headerText);
    %     end
    % end
end
