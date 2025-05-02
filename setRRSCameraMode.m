function setRRSCameraMode(rrApp, mode, opt)
% doc:https://www.mathworks.com/help/releases/R2024b/roadrunner/ref/roadrunner_service_messages.proto.html
arguments
    rrApp roadrunner;
    mode {mustBeMember(mode,["follow","orbit", "front", "default"])};
    opt.actorID {mustBeFinite,mustBePositive, mustBeInteger} = 1;
    opt.height {mustBeFinite,mustBeNonnegative} = 3;
    opt.distance {mustBeFinite,mustBeNonnegative} = 5;
    opt.duration (1,1) {mustBeFinite,mustBeNonnegative} = 0.1;
end

if isMATLABReleaseOlderThan("R2024b")
    warning("camera mode setting is enabled in R2024b or newer.");
    return;
end

%コマンド
rrCmd = strcat('"',fullfile(rrApp.InstallationFolder, "CmdRoadRunnerApi.exe"),'"');
rrCmdBase = rrCmd + " --serverAddress localhost:" + string(rrApp.getAPIPort()) + " ";

% カメラをFollowに変更するコマンド
if mode == "follow"
    rrCmdSetFront = rrCmdBase + """SetCameraMode(follow_mode.focus_actor_id='"+string(opt.actorID)+"'"...
        + " follow_mode.height='"+string(opt.height)+"'"...
        + " follow_mode.distance='"+string(opt.distance)+"'"...
        + ")\""";
elseif mode == "orbit"
    rrCmdSetFront = rrCmdBase + """SetCameraMode(orbit_mode.focus_actor_id='"+string(opt.actorID)+"')\""";
elseif mode == "front"
    rrCmdSetFront = rrCmdBase + """SetCameraMode(front_mode.focus_actor_id='"+string(opt.actorID)+"')\""";
elseif mode == "default"
    rrCmdSetFront = rrCmdBase + """SetCameraMode(default_mode)\""";
end
% カメラを変更
system(rrCmdSetFront);
pause(opt.duration);
end