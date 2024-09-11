function helperCreatePathTargetBus(maxNumPoints)
% helper function to create required buses for HelperPathActionReader
% system object.
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2022-2023 The MathWorks, Inc.

% Bus object: BusPathPointTiming 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Time';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Speed';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'WaitTime';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

BusPathPointTiming = Simulink.Bus;
BusPathPointTiming.HeaderFile = '';
BusPathPointTiming.Description = '';
BusPathPointTiming.DataScope = 'Auto';
BusPathPointTiming.Alignment = -1;
BusPathPointTiming.PreserveElementDimensions = 0;
BusPathPointTiming.Elements = elems;
clear elems;
assignin('base','BusPathPointTiming', BusPathPointTiming);

% Bus object: BusPathTarget 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'Path';
elems(1).Dimensions = [maxNumPoints 3];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'NumPoints';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint64';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'HasTimings';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'boolean';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Timings';
elems(4).Dimensions = [1 maxNumPoints];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'Bus: BusPathPointTiming';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

BusPathTarget = Simulink.Bus;
BusPathTarget.HeaderFile = '';
BusPathTarget.Description = '';
BusPathTarget.DataScope = 'Auto';
BusPathTarget.Alignment = -1;
BusPathTarget.PreserveElementDimensions = 0;
BusPathTarget.Elements = elems;
clear elems;
assignin('base','BusPathTarget', BusPathTarget);

end

