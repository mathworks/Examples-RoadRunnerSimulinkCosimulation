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
        long_offset = 0;
    end

    % Pre-computed constants
    properties(Access = private)

        % Starting Index of section containing reference pose
        SectionStartIndex = 1;

        % Prevous reference pose and curvature
        RefPosePrev
        RefCurvaturePrev

        % Reference path
        RefPath
        % Reference curvature of whole path
        RefCurvature

        % line segment (only enable if vehicleOnPath == false)
        lineSegment;
        lineSegment_sq;
    end

    methods(Access = protected)
        function [RefPointOnPath, RefCurvature, IsGoalReached] = stepImpl(obj, PrevVehicleInfo, Trajectory, NumTrajPoints, currentSpeed)
            
            % Getting Pose information from currPosePrev
            % currPose = [PrevVehicleInfo.CurrPose(1), PrevVehicleInfo.CurrPose(2), deg2rad(PrevVehicleInfo.CurrPose(3))];
            currPose = [PrevVehicleInfo(1), PrevVehicleInfo(2), deg2rad(PrevVehicleInfo(3)), PrevVehicleInfo(4), deg2rad(PrevVehicleInfo(5))];

            if getCurrentTime(obj) == 0
            % Interpolate waypoints and caluculate curvatures
                [waypoints, curvatures] = generateCurvatures(obj,Trajectory,NumTrajPoints);
                obj.RefPath = waypoints;
                obj.RefCurvature = curvatures;

                RefPointOnPath = currPose;
                RefCurvature = curvatures(1);
                IsGoalReached = false;

                RefPointOnPath([3, 5]) = rad2deg(RefPointOnPath([3, 5]));
                obj.RefPosePrev = RefPointOnPath;
                obj.RefCurvaturePrev = RefCurvature;

                % 車両が軌跡に載っていないときのみ使用する
                if obj.vehicleOnPath == false
                    P1 = waypoints(1:end-1, 1:2); %x, yのみ考える
                    P2 = waypoints(2:end, 1:2);

                    obj.lineSegment = P2 - P1;
                    obj.lineSegment_sq = sum(obj.lineSegment.^2, 2);
                end

                return
            end

            % Obtain reference pose on the path based on the previous pose
            [IsGoalReached,RefPointOnPath,RefCurvature] = computeReferenceStateBasedOnPose(...
                obj,currPose,obj.RefPath,obj.RefCurvature, currentSpeed);

            % Update the pose and curvature for next time step
            obj.RefPosePrev = RefPointOnPath;
            obj.RefCurvaturePrev = RefCurvature;
        end

        % Generate curvatures from the trajectory
        function [refPose, refCurvature] = generateCurvatures(obj, trajectory, numTrajPoints)
            % Remove repetetive waypoints
            [~,uniqueId] = unique(trajectory(1:numTrajPoints,1),'stable');
            trajectory = trajectory(uniqueId,:);

            % Interpolate based on distance
            interpDistance = 3;
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
            refPose(:,5) = refPitch;

            % Calculating curvature
            [~,~,~,refCurvature] = smoothPathSpline(refPose(:,1:3), ones(interpNumTrajPoints,1),interpNumTrajPoints);
            refPose(:,3) = deg2rad(refPose(:, 3));
            refPose(:,5) = deg2rad(refPose(:, 5));
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
        function [isGoalReached,refPoseCurr,refCurvature] = computeReferenceStateBasedOnPose(...
                obj, currPosePrev, waypoints, curvatures, speed)
            
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
            else
                %車両が軌跡から外れる場合、最短のセクション（waypontsとwaypointsの間の線分）を探索する
                PointXY_raw = currPosePrev(1:2);
                offset = eul2rotm([currPosePrev(3), 0, currPosePrev(5)], "ZYX") * [obj.long_offset; 0; 0];

                % offset = eul2rotm([currPosePrev(3), 0, currPosePrev(5)], "ZYX") * [speed * obj.timeStep; 0; 0];
                PointXY = PointXY_raw + offset(1:2)';
                % degbug for offset
                % figure;
                % plot(waypoints(:,1), waypoints(:,2)); hold on;
                % axis equal
                % plot(PointXY_raw(1), PointXY_raw(2), "o");
                % plot(PointXY(1), PointXY(2), "^");

                %現在のセクションより近傍のセクションのみ探索
                lowerIdx = max(obj.SectionStartIndex - 3, 1);
                upperIdx = min(obj.SectionStartIndex + 3, size(obj.lineSegment, 1));

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

                weight1 = 1-t(minIdx); %現在値から次のセクションまで距離の割合　次のセクションの重み
                weight2 = t(minIdx);%現在セクションから現在地までの距離の割合　現在セクションの重み

                currPosePrev = waypoints(obj.SectionStartIndex,:);

                %debug for section
                % figure;
                % plot(waypoints(:,1), waypoints(:,2)); hold on;
                % axis equal
                % plot(PointXY_raw(1), PointXY_raw(2), "o");
                % plot(PointXY(1), PointXY(2), "^");
                % plot(R(minIdx, 1), R(minIdx, 2), "x", "Color", "r");
                % plot(waypoints(obj.SectionStartIndex, 1), waypoints(obj.SectionStartIndex, 2), "x", "Color", "g");
                % plot(waypoints(obj.SectionStartIndex+1, 1), waypoints(obj.SectionStartIndex+1, 2), "x", "Color", "b");
                % plot([R(minIdx, 1);PointXY(1)], [R(minIdx, 2); PointXY(2)], "Color", "r");
            end
            
            % % Normalized distance between current position and section starting point
            % u = (RXY.*DeltaXY)/(DeltaXY.*DeltaXY);
            % 
            % % Find section ending point
            % indexIncrement = ceil(u-1);
            % 
            % if indexIncrement<0
            %     % In current section
            %     indexIncrement = 0;
            % end
            % if u >=1
            %     % Increment to appropriate section 
            %     % with the assumption that the distance between waypoints is 
            %     % approximately equal for near sections
            %     obj.SectionStartIndex = obj.SectionStartIndex+indexIncrement;
            % 
            %     % Adjust u to account for new section starting point
            %     u = u - indexIncrement;
            % end
            
            if obj.SectionStartIndex < (numWaypoints-1)
                currentSectionEndIndex = obj.SectionStartIndex+1;
                nextSectionEndIndex    = currentSectionEndIndex+1;
                % Target States at end point of current and next sections
                XYTarget0 = [currPosePrev(1), currPosePrev(2)];
                XYTarget1 = [waypoints(currentSectionEndIndex,1),...
                             waypoints(currentSectionEndIndex,2)];
                YawTarget0 = currPosePrev(3);
                YawTarget1 = waypoints(currentSectionEndIndex,3);
                YawTarget1_set = [YawTarget1, YawTarget1+2*pi, YawTarget1-2*pi];
                [~, index_min] = min(abs(YawTarget1_set - YawTarget0));

                YawTarget1 = YawTarget1_set(index_min);

                ZTarget0 = currPosePrev(4);
                ZTarget1 = waypoints(currentSectionEndIndex,4);
                PitchTarget0 = currPosePrev(5);
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
                obj.SectionStartIndex = numWaypoints-1;
                isGoalReached = true;
            end
        end


        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function icon = getIconImpl(~)
            % Use class name slightly raised above center
            icon = [mfilename("class"), newline, newline, newline, newline];
        end

        function [out,out2,out3] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 5];
            out2 = [1 1];
            out3 = [1 1];
        end

        function [out,out2,out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "boolean";
        end

        function [out,out2,out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
        end

        function [out,out2,out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
        end
        
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end