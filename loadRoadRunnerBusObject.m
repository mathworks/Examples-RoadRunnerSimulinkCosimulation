function loadRoadRunnerBusObject()

if isMATLABReleaseOlderThan("R2025a")
    %load(fullfile(matlabroot,'toolbox','driving','drivingdata','rrScenarioSimTypes.mat'));
    evalin('base', sprintf('load(''%s'')', fullfile(matlabroot,'toolbox','driving','drivingdata','rrScenarioSimTypes.mat')));
else % path is changed from R2025a
    % load(fullfile(matlabroot,'toolbox','driving', 'core', 'drivingdata','rrScenarioSimTypes.mat'));
    evalin('base', sprintf('load(''%s'')', fullfile(matlabroot,'toolbox','driving', 'core', 'drivingdata','rrScenarioSimTypes.mat')));
end

end