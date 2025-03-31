classdef HelperReferencePoseOnPath < matlab.System
    % HELPERREFERENCEPOSEONPATH computes reference pose on reference path
    % based of previous pose.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    %
    
    % Copyright 2021 The MathWorks, Inc.
    properties(Access = public)
        timeStep = 0.01; % time step for distance calculation by speed
        
        vehicleOnPath = true; % vehicleOnPath: if vehicle doesn't follow path exactly, set this false
        long_offset = 0;% long_offset: longitudinal offset from input position to front axle. If input position is on front axle, it's 0. 

        debugFig = false; % debugFig: if true, debug figure whch show vehicle target position on path is visualized
        saveFigVideo = false; % only enabled if debugFig == true. "videos/pathFollow{N}.mp4" is saved (N is number). 
        zoomFig = nan; % nan or positive scalar. if positive scalar is set, figure focus on vehicle

        searchIndx = 10; %Range of sections to search for the current nearest neighbor 現在の最近傍を検索するセクションの範囲

        numReferencePose = 1;%numReferencePose: default is 1, if this is increased, future trajectory is outputted
        controlTimestep = 0.1;%controlTimestep: default is 0.1. time step for control loop
    end

    % Pre-computed constants
    properties(Access = private)

        % Starting Index of section containing reference pose
        SectionStartIndex = 1;

        % Prevous reference pose and curvature
        RefPosePrev
        RefCurvaturePrev
        RefTimingPrev;
        RefSpeedOnPathPrev;

        % Reference path
        RefPath
        % Reference curvature of whole path
        RefCurvature
        % Reference timing
        RefTimings
        % Reference speed on each waypoint
        RefSpeedOnWaypoint

        % line segment (only enable if vehicleOnPath == false)
        lineSegment;
        lineSegment_sq;

        % cumsum distance of trajectory
        cumDistanceTraj

        % figure axis for debug draw;
        f_handle;
        ax = [];
        h_repPts;
        h_vehicle;
        h_tire;
        h_futurePts;
        h_quiver;
        videoWriter;
    end

    methods(Access = protected)
        function setupImpl(obj)
            if obj.debugFig && obj.saveFigVideo
                idx = 1;
                while true
                    vname = fullfile(pwd, "videos/", sprintf("pathFollow%02d.mp4", idx));
                    if ~exist(vname, "file")
                        break
                    end
                    idx = idx+1;
                end

                % VideoWriter
                obj.videoWriter = VideoWriter(vname, "MPEG-4");
                if obj.timeStep == -1
                    obj.videoWriter.FrameRate = 30;
                else
                    obj.videoWriter.FrameRate = 1/obj.timeStep;  % 必要に応じて調整
                end
                obj.videoWriter.open();
            end
        end
        function [RefPointOnPath, RefCurvature, IsGoalReached, RefSpeedOnPath, RefTimings] = stepImpl(obj, PrevVehicleInfo, Trajectory, NumTrajPoints, currentSpeed, Traj_timings)
            
            % Getting Pose information from currPosePrev
            % currPose = [PrevVehicleInfo.CurrPose(1), PrevVehicleInfo.CurrPose(2), deg2rad(PrevVehicleInfo.CurrPose(3))];
            currPose = [PrevVehicleInfo(1), PrevVehicleInfo(2), deg2rad(PrevVehicleInfo(3)), PrevVehicleInfo(4), deg2rad(PrevVehicleInfo(5))];

            if getCurrentTime(obj) == 0
            % Interpolate waypoints and caluculate curvatures
                [waypoints, curvatures,timings,refSpeeds] = generateCurvatures(obj,Trajectory,NumTrajPoints,Traj_timings);
                obj.RefPath = waypoints;
                obj.RefCurvature = curvatures;
                obj.RefTimings = timings;
                obj.RefSpeedOnWaypoint = refSpeeds;

                RefPointOnPath = currPose;
                RefCurvature = curvatures(1);
                IsGoalReached = false;
                RefSpeedOnPath = nan;
                RefTimings = nan;
                if ~isempty(obj.RefTimings) && ~isempty(obj.RefSpeedOnWaypoint)
                    RefSpeedOnPath = obj.RefSpeedOnWaypoint(1);
                    RefTimings = obj.RefTimings(1);
                end

                RefPointOnPath([3, 5]) = rad2deg(RefPointOnPath([3, 5]));
                obj.RefPosePrev = RefPointOnPath;
                obj.RefCurvaturePrev = RefCurvature;
                obj.RefSpeedOnPathPrev = RefSpeedOnPath;
                obj.RefTimingPrev = RefTimings;

                % 車両が軌跡に載っていないときのみ使用する
                if obj.vehicleOnPath == false
                    P1 = waypoints(1:end-1, 1:2); %x, yのみ考える
                    P2 = waypoints(2:end, 1:2);

                    obj.lineSegment = P2 - P1;
                    obj.lineSegment_sq = sum(obj.lineSegment.^2, 2);
                end

                % if trajectory needs multiple output, add those output based
                % on speed and timestep  
                if obj.numReferencePose > 1
                    distance_prediction = currentSpeed * obj.controlTimestep * obj.numReferencePose;
                    dists = obj.cumDistanceTraj - obj.cumDistanceTraj(obj.SectionStartIndex) - distance_prediction;
                    index_prediction = find(dists > 0, 1); % firt true means final points of prediction
                    if isempty(index_prediction)
                        index_prediction = numWaypoints-1;
                    end
    
                    cumDist_future = [0, obj.cumDistanceTraj(obj.SectionStartIndex:index_prediction)];
                    waypts_future = [currPose; waypoints(obj.SectionStartIndex:index_prediction, :)];
                    curvature_future = [curvatures(1); curvatures(obj.SectionStartIndex:index_prediction, :)];

                    [cumDist_future, waypts_future, curvature_future] = removeDuplicateRows(cumDist_future, waypts_future, curvature_future);

                    traveled_dists = [0, currentSpeed * (1:obj.numReferencePose-1) * obj.controlTimestep];
                    % Interpolate the postion
                    RefPointOnPath = interp1(cumDist_future,waypts_future,traveled_dists,'linear','extrap');
                    RefCurvature = interp1(cumDist_future,curvature_future,traveled_dists,'linear','extrap')';

                    RefPointOnPath(:,[3, 5]) = rad2deg(RefPointOnPath(:,[3, 5]));

                    if ~isempty(obj.RefTimings) && ~isempty(obj.RefSpeedOnWaypoint)
                        timing_future = obj.RefTimings(obj.SectionStartIndex:index_prediction);
                        speed_future = obj.RefSpeedOnWaypoint(obj.SectionStartIndex:index_prediction);
                        RefTimings = interp1(cumDist_future,timing_future,traveled_dists,'linear','extrap')';
                        RefSpeedOnPath = interp1(cumDist_future,speed_future,traveled_dists,'linear','extrap')';
                    end
                end

                if obj.debugFig && isempty(obj.ax)
                    obj.f_handle = figure; set(obj.f_handle, "Visible", "on");
                    obj.ax = gca;
                    plot(waypoints(:,1), waypoints(:,2), "b-", 'Parent', obj.ax);
                    hold on;
                    % axis equal;
                    obj.h_repPts = plot(waypoints(1,1), waypoints(1,2), 'g*', 'MarkerSize', 12, 'Parent', obj.ax);
                    obj.h_vehicle = plot(waypoints(1:2,1), waypoints(1:2,2), '-r.', 'MarkerSize', 12, 'Parent', obj.ax);
                    
                    obj.h_quiver = quiver(currPose(1), currPose(2), cos(currPose(3)), sin(currPose(3)), Parent=obj.ax);
                    obj.h_quiver.MaxHeadSize = 10;
                    legend(obj.ax, ["trajectory", "ref pts", "vehicle pos", "vehicle orientation"], Location="southeast");
                    if obj.numReferencePose > 1
                        obj.h_futurePts = plot(RefPointOnPath(:,1), RefPointOnPath(:,2), 'm*', 'MarkerSize', 6, 'Parent', obj.ax);
                        legend(obj.ax, ["trajectory", "ref pts", "vehicle pos", "vehicle orientation", "future ref pts"]);
                    end
                    if ~isnan(obj.zoomFig) && obj.zoomFig > 0
                        axis(obj.ax, 'manual');
                        xlim(obj.ax, [currPose(1) - obj.zoomFig, currPose(1) + obj.zoomFig]);
                        ylim(obj.ax, [currPose(2) - obj.zoomFig, currPose(2) + obj.zoomFig]);
                    end
                    if obj.saveFigVideo
                        frame = getframe(obj.f_handle).cdata;
                        obj.videoWriter.writeVideo(frame);
                    end
                end

                return
            end

            % Obtain reference pose on the path based on the previous pose
            [IsGoalReached,RefPointOnPath,RefCurvature,RefTimings,RefSpeedOnPath] = computeReferenceStateBasedOnPose(...
                obj,currPose,obj.RefPath,obj.RefCurvature,currentSpeed,obj.RefTimings, obj.RefSpeedOnWaypoint);

            % Update the pose and curvature for next time step
            obj.RefPosePrev = RefPointOnPath;
            obj.RefCurvaturePrev = RefCurvature;
            obj.RefTimingPrev = RefTimings;
            obj.RefSpeedOnPathPrev = RefSpeedOnPath;

            if obj.debugFig
                obj.h_repPts.XData = RefPointOnPath(1, 1);
                obj.h_repPts.YData = RefPointOnPath(1, 2);

                obj.h_vehicle.XData = [currPose(1), currPose(1)+obj.long_offset*cos(currPose(3))];
                obj.h_vehicle.YData = [currPose(2), currPose(2)+obj.long_offset*sin(currPose(3))];

                obj.h_quiver.XData = currPose(1);
                obj.h_quiver.YData = currPose(2);
                obj.h_quiver.UData = cos(currPose(3));
                obj.h_quiver.VData = sin(currPose(3));

                if obj.numReferencePose > 1
                    obj.h_futurePts.XData = RefPointOnPath(:,1);
                    obj.h_futurePts.YData = RefPointOnPath(:,2);
                end
                if ~isnan(obj.zoomFig) && obj.zoomFig > 0
                    xlim(obj.ax, [currPose(1) - obj.zoomFig, currPose(1) + obj.zoomFig]);
                    ylim(obj.ax, [currPose(2) - obj.zoomFig, currPose(2) + obj.zoomFig]);
                end
                if obj.saveFigVideo
                    frame = getframe(obj.f_handle).cdata;
                    obj.videoWriter.writeVideo(frame);
                end
            end
        end

        % Generate curvatures from the trajectory
        function [refPose, refCurvature, refTimings, refSpeed] = ...
                generateCurvatures(obj, trajectory, numTrajPoints, timings)
            % Remove repetetive waypoints
            [~,uniqueId] = unique(trajectory(1:numTrajPoints,1),'stable');
            trajectory = trajectory(uniqueId,:);

            % Interpolate based on distance
            interpDistance = 1;
            cumDistance = [0, cumsum(vecnorm(diff(trajectory),2,2))'];
            interpNumTrajPoints = round(cumDistance(end)/interpDistance);
            cumDistanceResample = linspace(0, cumDistance(end), interpNumTrajPoints);
            % Interpolate the postion
            refPose = zeros(interpNumTrajPoints,3);
            refPose(:,1) = interp1(cumDistance,trajectory(:,1),cumDistanceResample);
            refPose(:,2) = interp1(cumDistance,trajectory(:,2),cumDistanceResample);

            % Get reference yaw using interpolated position
            refYaw = getYaw(obj,refPose(:,1:2),interpNumTrajPoints);
            refPose = [refPose(:,1:2), refYaw];

            % add z value and pitch information
            refPose(:,4) = interp1(cumDistance,trajectory(:,3),cumDistanceResample);
            refPitch = getPitch(obj,refPose,interpNumTrajPoints);
            refPose(:,5) = refPitch * -1;

            % Calculating curvature
            [~,~,~,refCurvature] = smoothPathSpline(refPose(:,1:3), ones(interpNumTrajPoints,1),interpNumTrajPoints);
            refPose(:,3) = deg2rad(refPose(:, 3));
            refPose(:,5) = deg2rad(refPose(:, 5));

            obj.cumDistanceTraj = cumDistanceResample;

            refTimings = [];
            refSpeed = [];
            % Calculating timings and speed
            if ~isempty(timings)
                refTimings = interp1(cumDistance,vertcat(timings(uniqueId).Time),cumDistanceResample);
                refSpeed = interp1(cumDistance,vertcat(timings(uniqueId).Speed),cumDistanceResample);
            end
        end

        % Caluculate yaw from the positions
        function refYaw = getYaw(~, trajectory, numTrajPoints)
            refYaw = zeros(numTrajPoints,1);
            
            for idx = 2:numTrajPoints
                % Get the positions
                currPoint = trajectory(idx-1, :);
                nextPoint = trajectory(idx, :);
                % Caluculate yaw
                if ~all(nextPoint == currPoint)
                    tangent = (nextPoint - currPoint)/ norm(nextPoint - currPoint);
                    tangent = tangent / norm(tangent);
                    yaw = atan2d(tangent(2), tangent(1));
                    % Store the yaw
                    refYaw(idx-1,1) = yaw;
                else
                    refYaw(idx-1,1) = refYaw(idx-2,1);
                end
            end
            refYaw(end,1) = refYaw(end-1,1);
        end

        % Caluculate pitch from the positions
        function refPitch = getPitch(~, trajectory, numTrajPoints)
            refPitch = zeros(numTrajPoints,1);
            
            for idx = 2:numTrajPoints
                % Get the positions
                currPoint = trajectory(idx-1, [1 2 4]);
                nextPoint = trajectory(idx, [1 2 4]);
                % Caluculate pitch
                if ~all(nextPoint == currPoint)
                    diff = (nextPoint - currPoint);
                    pitch = atan2d(diff(3), sqrt(diff(1)^2 + diff(2)^2));
                    % Store the yaw
                    refPitch(idx-1,1) = pitch;
                else
                    refPitch(idx-1,1) = refPitch(idx-2,1);
                end
            end
            refPitch(end,1) = refPitch(end-1,1);
        end

        % Caluclate reference pose on the path based on the previous pose
        function [isGoalReached,refPoseCurr,refCurvature,refTiming,refSpeed] = computeReferenceStateBasedOnPose(...
                obj, currPosePrev, waypoints, curvatures, speed, timings, traj_speeds)
            
            if obj.SectionStartIndex < 1
                obj.SectionStartIndex = 1;
            end
            
            refPoseCurr  = zeros(1, 5);
            refCurvature = 0;
            isGoalReached = true;
            
            numWaypoints = size(waypoints,1);
            if (numWaypoints == 0)
                return;
            end

            % 車両が必ず軌跡上に乗る場合、車両の移動距離に応じてセクションを切り替える
            % （＝どのwaypoitsを参照姿勢とするかは車両の移動距離のみで決まる）
            if obj.vehicleOnPath
                % Forward distance
                dist_forward = speed * obj.timeStep;
                
                % Distance between section start and end points
                DeltaXY = [waypoints(obj.SectionStartIndex+1,1)-waypoints(obj.SectionStartIndex,1),...
                           waypoints(obj.SectionStartIndex+1,2)-waypoints(obj.SectionStartIndex,2),...
                           waypoints(obj.SectionStartIndex+1,4)-waypoints(obj.SectionStartIndex,4)];
                
                % Distance between current position and section starting point
                RXY = [currPosePrev(1)-waypoints(obj.SectionStartIndex,1),...
                       currPosePrev(2)-waypoints(obj.SectionStartIndex,2),...
                       currPosePrev(4)-waypoints(obj.SectionStartIndex,4)];
    
                %
                RXY_next = [waypoints(obj.SectionStartIndex+1,1) - currPosePrev(1),...
                       waypoints(obj.SectionStartIndex+1,2) - currPosePrev(2),...
                       waypoints(obj.SectionStartIndex+1,4) - currPosePrev(4)];
                dist_nextSection = norm(RXY_next);
    
                % 車両が次のセクションよりも進む場合、現在セクションを次のセクションに更新
                while dist_nextSection < dist_forward
                    dist_forward = dist_forward - dist_nextSection;
                    obj.SectionStartIndex = obj.SectionStartIndex + 1;
    
                    RXY_next = [waypoints(obj.SectionStartIndex+1,1) - waypoints(obj.SectionStartIndex,1),...
                       waypoints(obj.SectionStartIndex+1,2) - waypoints(obj.SectionStartIndex,2),...
                       waypoints(obj.SectionStartIndex+1,4) - waypoints(obj.SectionStartIndex,4)];
                    dist_nextSection = norm(RXY_next);
                    currPosePrev =  waypoints(obj.SectionStartIndex,:); %次のセクションで現在位置を更新　セクションとセクションの間の位置
                end
                weight1 = dist_forward / dist_nextSection; %現在値から次のセクションまで距離の割合　次のセクションの重み
                weight2 = 1 - weight1;%現在セクションから現在地までの距離の割合　現在セクションの重み
                sectionStartPose = currPosePrev;
            else
                %車両が軌跡から外れる場合、最短のセクション（waypontsとwaypointsの間の線分）を探索する
                PointXY_raw = currPosePrev(1:2);
                offset = eul2rotm([currPosePrev(3), 0, currPosePrev(5)], "ZYX") * [obj.long_offset; 0; 0];
                offset = [obj.long_offset*cos(currPosePrev(3)), obj.long_offset*sin(currPosePrev(3))];

                % offset = eul2rotm([currPosePrev(3), 0, currPosePrev(5)], "ZYX") * [speed * obj.timeStep; 0; 0];
                PointXY = PointXY_raw + offset(1:2);

                %現在のセクションより近傍のセクションのみ探索
                lowerIdx = max(obj.SectionStartIndex - obj.searchIndx, 1);
                upperIdx = min(obj.SectionStartIndex + obj.searchIndx, size(obj.lineSegment, 1));

                pts2SegmentStartPts = PointXY - waypoints(lowerIdx:upperIdx,1:2);

                % 係数 t の計算
                % t < 0のとき、最短距離はセクションの開始点
                % 0 <= t <= 1のとき、最短距離はセクション線分への垂直距離
                % t > 1とのき、最短距離はセクションの終了点
                t_raw = sum(obj.lineSegment(lowerIdx:upperIdx,:) .* pts2SegmentStartPts, 2) ./ obj.lineSegment_sq(lowerIdx:upperIdx);
                t = max(0, min(1, t_raw));% t の範囲を調整

                % 各セクション上の点を最短点を計算
                R = waypoints(lowerIdx:upperIdx,1:2) + t .* obj.lineSegment(lowerIdx:upperIdx,:);

                % 車両位置との距離を計算
                distances = sqrt(sum((PointXY - R).^2, 2));

                % 最小距離の選択
                [minDist, minIdx] = min(distances);
                obj.SectionStartIndex = lowerIdx - 1 + minIdx;

                weight2 = 1-t(minIdx); %referencePoseから次のセクションまで距離の割合　次のセクションの重み
                weight1 = t(minIdx); %現在セクションからreferencePoseまでの距離の割合　現在セクションの重み

                sectionStartPose = waypoints(obj.SectionStartIndex,:);
                % currPosePrev = waypoints(obj.SectionStartIndex,:);
            end

            if obj.SectionStartIndex < (numWaypoints-1)
                currentSectionEndIndex = obj.SectionStartIndex+1;
                nextSectionEndIndex    = currentSectionEndIndex+1;
                % Target States at end point of current and next sections
                XYTarget0 = [sectionStartPose(1), sectionStartPose(2)];
                XYTarget1 = [waypoints(currentSectionEndIndex,1),...
                             waypoints(currentSectionEndIndex,2)];
                YawTarget0 = sectionStartPose(3);
                YawTarget1 = waypoints(currentSectionEndIndex,3);
                YawTarget1_set = [YawTarget1, YawTarget1+2*pi, YawTarget1-2*pi];
                [~, index_min] = min(abs(YawTarget1_set - YawTarget0));

                YawTarget1 = YawTarget1_set(index_min);

                ZTarget0 = sectionStartPose(4);
                ZTarget1 = waypoints(currentSectionEndIndex,4);
                PitchTarget0 = sectionStartPose(5);
                PitchTarget1 = waypoints(currentSectionEndIndex,5);
                
                CurvatureTarget0 = curvatures(obj.SectionStartIndex);
                CurvatureTarget1 = curvatures(currentSectionEndIndex);
                
                % Target position
                XYTargetCurr = weight2*XYTarget0+weight1*XYTarget1;
                YawTargetCurr = weight2*YawTarget0+weight1*YawTarget1;
                ZTargetCurr = weight2*ZTarget0+weight1*ZTarget1;
                PitchTargetCurr = weight2*PitchTarget0+weight1*PitchTarget1;

                % Desired RefPose
                refPoseCurr = [XYTargetCurr rad2deg(YawTargetCurr) ZTargetCurr rad2deg(PitchTargetCurr)];
                refCurvature = weight2*CurvatureTarget0+weight1*CurvatureTarget1;
                isGoalReached = false;

                % Desired RefTiming and speed
                if ~isempty(timings) && ~isempty(traj_speeds)
                    refTiming = weight2*timings(obj.SectionStartIndex) + weight1*timings(currentSectionEndIndex);
                    refSpeed = weight2*traj_speeds(obj.SectionStartIndex) + weight1*traj_speeds(currentSectionEndIndex);
                end
                
                % figure;
                % plot(waypoints(:,1), waypoints(:,2)); hold on;
                % axis equal
                % plot(currPosePrev(1), currPosePrev(2), "^");
                % plot(refPoseCurr(minIdx, 1), refPoseCurr(minIdx, 2), "x", "Color", "r");
                % plot(waypoints(obj.SectionStartIndex, 1), waypoints(obj.SectionStartIndex, 2), "x", "Color", "g");
                % plot(waypoints(obj.SectionStartIndex+1, 1), waypoints(obj.SectionStartIndex+1, 2), "x", "Color", "b");
            else
                % Maintain last heading between points
                refPoseCurr = obj.RefPosePrev;   
                refCurvature = obj.RefCurvaturePrev;
                refTiming = obj.RefTimingPrev;
                refSpeed = obj.RefSpeedOnPathPrev;
                obj.SectionStartIndex = numWaypoints-1;
                isGoalReached = true;

                if obj.numReferencePose > 1
                    refPoseCurr = repmat(refPoseCurr, obj.numReferencePose, 1);
                    refCurvature = repmat(refCurvature, obj.numReferencePose, 1);

                    refTiming = repmat(refTiming, obj.numReferencePose, 1);
                    refSpeed = repmat(refSpeed, obj.numReferencePose, 1);
                    return;
                end
            end

            % if trajectory needs multiple output, add those output based
            % on speed and timestep  
            if obj.numReferencePose > 1
                currentSectionEndIndex = obj.SectionStartIndex+1;
                distance_prediction = speed * obj.controlTimestep * obj.numReferencePose;
                dists = obj.cumDistanceTraj - obj.cumDistanceTraj(currentSectionEndIndex) - distance_prediction;
                index_prediction = find(dists > 0, 1); % firt true means final points of prediction
                if isempty(index_prediction)
                    index_prediction = numWaypoints;
                end

                refPoseCurr_temp = refPoseCurr;
                refPoseCurr_temp([3, 5]) = deg2rad(refPoseCurr_temp([3, 5]));
                cumDist_future = [0, obj.cumDistanceTraj(currentSectionEndIndex:index_prediction) - obj.cumDistanceTraj(currentSectionEndIndex)];

                waypts_future = [refPoseCurr_temp; waypoints(currentSectionEndIndex:index_prediction, :)];
                curvature_future = [refCurvature; curvatures(currentSectionEndIndex:index_prediction, :)];

                [cumDist_future, waypts_future, curvature_future,validFlags] = removeDuplicateRows(cumDist_future, waypts_future, curvature_future);

                traveled_dists = [0, speed * (1:obj.numReferencePose-1) * obj.controlTimestep];
                % Interpolate the postion
                refPoseCurr = interp1(cumDist_future,waypts_future,traveled_dists,'linear','extrap');
                refCurvature = interp1(cumDist_future,curvature_future,traveled_dists,'linear','extrap')';

                refPoseCurr(:,[3, 5]) = rad2deg(refPoseCurr(:,[3, 5]));

                % calculate timing future
                if ~isempty(timings) && ~isempty(traj_speeds)
                    timing_future = [refTiming; timings(currentSectionEndIndex:index_prediction)'];
                    speed_future = [refSpeed; traj_speeds(currentSectionEndIndex:index_prediction)'];

                    timing_future = timing_future(validFlags);
                    speed_future = speed_future(validFlags);

                    refTiming = interp1(cumDist_future,timing_future,traveled_dists,'linear','extrap')';
                    refSpeed = interp1(cumDist_future,speed_future,traveled_dists,'linear','extrap')';
                end
            end
        end


        function releaseImpl(obj)
            close(obj.videoWriter);
        end

        function icon = getIconImpl(~)
            % Use class name slightly raised above center
            icon = [mfilename("class"), newline, newline, newline, newline];
        end

        function [out,out2,out3,out4,out5] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [obj.numReferencePose 5];
            out2 = [obj.numReferencePose 1];
            out3 = [1 1];
            out4 = [obj.numReferencePose 1];
            out5 = [obj.numReferencePose 1];
        end

        function [out,out2,out3,out4,out5] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "boolean";
            out4 = "double";
            out5 = "double";
        end

        function [out,out2,out3,out4,out5] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
        end

        function [out,out2,out3,out4,out5] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
        end
        
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
        function sts = getSampleTimeImpl(obj)
            if obj.timeStep == -1
                sts = createSampleTime(obj,'ErrorOnPropagation','Controllable');
            else
                sts = createSampleTime(obj,'Type','Discrete',...
                 'SampleTime', obj.timeStep);
            end
        end
    end
end
function [cumDist_future, waypts_future, curvature_future,logicalArrayAnd] = removeDuplicateRows(cumDist_future, waypts_future, curvature_future)
    [~, unique_indices1] = unique(cumDist_future, "stable");
    [~, unique_indices2] = unique(waypts_future, "rows", "stable");
    [~, unique_indices3] = unique(curvature_future, "stable");
    
    logicalArray1 = false(1, numel(cumDist_future));
    logicalArray1(unique_indices1) = true;
    logicalArray2 = false(1, numel(cumDist_future));
    logicalArray2(unique_indices2) = true;
    logicalArray3 = false(1, numel(cumDist_future));
    logicalArray3(unique_indices3) = true;
    
    logicalArrayAnd = logicalArray1 & logicalArray2 & logicalArray3;
    
    cumDist_future = cumDist_future(logicalArrayAnd);
    waypts_future = waypts_future(logicalArrayAnd, :);
    curvature_future = curvature_future(logicalArrayAnd);
end