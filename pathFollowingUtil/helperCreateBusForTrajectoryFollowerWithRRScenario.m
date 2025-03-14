% helperCreateBusForTrajectoryFollowerWithRRScenario creates bus objects 
% used by TrajectoryFollowerWithRRScenarioTestBench.slx model
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

% Copyright 2021 The MathWorks, Inc.

% Bus object: Vehicle states
clear elems;
BusVehStates = Simulink.Bus;
BusVehStates.Description = '';
BusVehStates.DataScope = 'Auto';
BusVehStates.HeaderFile = '';
BusVehStates.Alignment = -1;
BusVehStates.PreserveElementDimensions = false;
elems{1} = Simulink.BusElement;
elems{1}.Name = 'ActorID';
elems{1}.Complexity = 'real';
elems{1}.Dimensions = [1 1];
elems{1}.DataType = 'uint32';
elems{1}.Min = [];
elems{1}.Max = [];
elems{1}.DimensionsMode = 'Fixed';
elems{1}.SamplingMode = 'Sample based';
elems{1}.DocUnits = '';
elems{1}.Description = '';
elems{1}(2, 1) = Simulink.BusElement;
elems{1}(2, 1).Name = 'Position';
elems{1}(2, 1).Complexity = 'real';
elems{1}(2, 1).Dimensions = [1 3];
elems{1}(2, 1).DataType = 'double';
elems{1}(2, 1).Min = [];
elems{1}(2, 1).Max = [];
elems{1}(2, 1).DimensionsMode = 'Fixed';
elems{1}(2, 1).SamplingMode = 'Sample based';
elems{1}(2, 1).DocUnits = '';
elems{1}(2, 1).Description = '';
elems{1}(3, 1) = Simulink.BusElement;
elems{1}(3, 1).Name = 'Velocity';
elems{1}(3, 1).Complexity = 'real';
elems{1}(3, 1).Dimensions = [1 3];
elems{1}(3, 1).DataType = 'double';
elems{1}(3, 1).Min = [];
elems{1}(3, 1).Max = [];
elems{1}(3, 1).DimensionsMode = 'Fixed';
elems{1}(3, 1).SamplingMode = 'Sample based';
elems{1}(3, 1).DocUnits = '';
elems{1}(3, 1).Description = '';
elems{1}(4, 1) = Simulink.BusElement;
elems{1}(4, 1).Name = 'Acceleration';
elems{1}(4, 1).Complexity = 'real';
elems{1}(4, 1).Dimensions = [1 3];
elems{1}(4, 1).DataType = 'double';
elems{1}(4, 1).Min = [];
elems{1}(4, 1).Max = [];
elems{1}(4, 1).DimensionsMode = 'Fixed';
elems{1}(4, 1).SamplingMode = 'Sample based';
elems{1}(4, 1).DocUnits = '';
elems{1}(4, 1).Description = '';
elems{1}(5, 1) = Simulink.BusElement;
elems{1}(5, 1).Name = 'Roll';
elems{1}(5, 1).Complexity = 'real';
elems{1}(5, 1).Dimensions = [1 1];
elems{1}(5, 1).DataType = 'double';
elems{1}(5, 1).Min = [];
elems{1}(5, 1).Max = [];
elems{1}(5, 1).DimensionsMode = 'Fixed';
elems{1}(5, 1).SamplingMode = 'Sample based';
elems{1}(5, 1).DocUnits = '';
elems{1}(5, 1).Description = '';
elems{1}(6, 1) = Simulink.BusElement;
elems{1}(6, 1).Name = 'Pitch';
elems{1}(6, 1).Complexity = 'real';
elems{1}(6, 1).Dimensions = [1 1];
elems{1}(6, 1).DataType = 'double';
elems{1}(6, 1).Min = [];
elems{1}(6, 1).Max = [];
elems{1}(6, 1).DimensionsMode = 'Fixed';
elems{1}(6, 1).SamplingMode = 'Sample based';
elems{1}(6, 1).DocUnits = '';
elems{1}(6, 1).Description = '';
elems{1}(7, 1) = Simulink.BusElement;
elems{1}(7, 1).Name = 'Yaw';
elems{1}(7, 1).Complexity = 'real';
elems{1}(7, 1).Dimensions = [1 1];
elems{1}(7, 1).DataType = 'double';
elems{1}(7, 1).Min = [];
elems{1}(7, 1).Max = [];
elems{1}(7, 1).DimensionsMode = 'Fixed';
elems{1}(7, 1).SamplingMode = 'Sample based';
elems{1}(7, 1).DocUnits = '';
elems{1}(7, 1).Description = '';
elems{1}(8, 1) = Simulink.BusElement;
elems{1}(8, 1).Name = 'AngularVelocity';
elems{1}(8, 1).Complexity = 'real';
elems{1}(8, 1).Dimensions = [1 3];
elems{1}(8, 1).DataType = 'double';
elems{1}(8, 1).Min = [];
elems{1}(8, 1).Max = [];
elems{1}(8, 1).DimensionsMode = 'Fixed';
elems{1}(8, 1).SamplingMode = 'Sample based';
elems{1}(8, 1).DocUnits = '';
elems{1}(8, 1).Description = '';
elems{1}(9, 1) = Simulink.BusElement;
elems{1}(9, 1).Name = 'LongitudinalSpeed';
elems{1}(9, 1).Complexity = 'real';
elems{1}(9, 1).Dimensions = [1 1];
elems{1}(9, 1).DataType = 'double';
elems{1}(9, 1).Min = [];
elems{1}(9, 1).Max = [];
elems{1}(9, 1).DimensionsMode = 'Fixed';
elems{1}(9, 1).SamplingMode = 'Sample based';
elems{1}(9, 1).DocUnits = '';
elems{1}(9, 1).Description = '';
elems{1}(10, 1) = Simulink.BusElement;
elems{1}(10, 1).Name = 'LongitudinalAccel';
elems{1}(10, 1).Complexity = 'real';
elems{1}(10, 1).Dimensions = [1 1];
elems{1}(10, 1).DataType = 'double';
elems{1}(10, 1).Min = [];
elems{1}(10, 1).Max = [];
elems{1}(10, 1).DimensionsMode = 'Fixed';
elems{1}(10, 1).SamplingMode = 'Sample based';
elems{1}(10, 1).DocUnits = '';
elems{1}(10, 1).Description = '';
elems{1}(11, 1) = Simulink.BusElement;
elems{1}(11, 1).Name = 'SteeringAngle';
elems{1}(11, 1).Complexity = 'real';
elems{1}(11, 1).Dimensions = [1 1];
elems{1}(11, 1).DataType = 'double';
elems{1}(11, 1).Min = [];
elems{1}(11, 1).Max = [];
elems{1}(11, 1).DimensionsMode = 'Fixed';
elems{1}(11, 1).SamplingMode = 'Sample based';
elems{1}(11, 1).DocUnits = '';
elems{1}(11, 1).Description = '';
elems{1}(12, 1) = Simulink.BusElement;
elems{1}(12, 1).Name = 'NumWheels';
elems{1}(12, 1).Complexity = 'real';
elems{1}(12, 1).Dimensions = [1 1];
elems{1}(12, 1).DataType = 'double';
elems{1}(12, 1).Min = [];
elems{1}(12, 1).Max = [];
elems{1}(12, 1).DimensionsMode = 'Fixed';
elems{1}(12, 1).SamplingMode = 'Sample based';
elems{1}(12, 1).DocUnits = '';
elems{1}(12, 1).Description = '';
elems{1}(13, 1) = Simulink.BusElement;
elems{1}(13, 1).Name = 'WheelPoses';
elems{1}(13, 1).Complexity = 'real';
elems{1}(13, 1).Dimensions = [18 1];
elems{1}(13, 1).DataType = 'BusWheelPoses';
elems{1}(13, 1).Min = [];
elems{1}(13, 1).Max = [];
elems{1}(13, 1).DimensionsMode = 'Fixed';
elems{1}(13, 1).SamplingMode = 'Sample based';
elems{1}(13, 1).DocUnits = '';
elems{1}(13, 1).Description = '';
BusVehStates.Elements = elems{1};
clear elems;
assignin('base','BusVehStates', BusVehStates);

% Vehicle info 
saveVarsTmp(1)                 = Simulink.BusElement;
saveVarsTmp(1).Name            = 'CurrPose';
saveVarsTmp(1).Dimensions      = [1 5];
saveVarsTmp(1).DimensionsMode  = 'Fixed';
saveVarsTmp(1).DataType        = 'double';
saveVarsTmp(1).SampleTime      = -1;
saveVarsTmp(1).Complexity      = 'real';

saveVarsTmp(2)                 = Simulink.BusElement;
saveVarsTmp(2).Name            = 'CurrVelocity';
saveVarsTmp(2).Dimensions      = 1;
saveVarsTmp(2).DimensionsMode  = 'Fixed';
saveVarsTmp(2).DataType        = 'double';
saveVarsTmp(2).SampleTime      = -1;
saveVarsTmp(2).Complexity      = 'real';

saveVarsTmp(3)                 = Simulink.BusElement;
saveVarsTmp(3).Name            = 'CurrYawRate';
saveVarsTmp(3).Dimensions      = 1;
saveVarsTmp(3).DimensionsMode  = 'Fixed';
saveVarsTmp(3).DataType        = 'double';
saveVarsTmp(3).SampleTime      = -1;
saveVarsTmp(3).Complexity      = 'real';

saveVarsTmp(4)                 = Simulink.BusElement;
saveVarsTmp(4).Name            = 'CurrSteer';
saveVarsTmp(4).Dimensions      = 1;
saveVarsTmp(4).DimensionsMode  = 'Fixed';
saveVarsTmp(4).DataType        = 'double';
saveVarsTmp(4).SampleTime      = -1;
saveVarsTmp(4).Complexity      = 'real';

saveVarsTmp(5)                 = Simulink.BusElement;
saveVarsTmp(5).Name            = 'Direction';
saveVarsTmp(5).Dimensions      = 1;
saveVarsTmp(5).DimensionsMode  = 'Fixed';
saveVarsTmp(5).DataType        = 'double';
saveVarsTmp(5).SampleTime      = -1;
saveVarsTmp(5).Complexity      = 'real';

BusVehicleInfo                      = Simulink.Bus;
BusVehicleInfo.Elements             = saveVarsTmp;
clear saveVarsTmp;
assignin('base','BusVehicleInfo',   BusVehicleInfo);

% Bus object: BusVehicleInfo 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'CurrPose';
elems(1).Dimensions = [1 6];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'CurrVelocity';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'CurrYawRate';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'CurrSteer';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Direction';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

BusFullVehicleInfo = Simulink.Bus;
BusFullVehicleInfo.HeaderFile = '';
BusFullVehicleInfo.Description = '';
BusFullVehicleInfo.DataScope = 'Auto';
BusFullVehicleInfo.Alignment = -1;
BusFullVehicleInfo.PreserveElementDimensions = 0;
BusFullVehicleInfo.Elements = elems;
clear elems;
assignin('base','BusFullVehicleInfo', BusFullVehicleInfo);