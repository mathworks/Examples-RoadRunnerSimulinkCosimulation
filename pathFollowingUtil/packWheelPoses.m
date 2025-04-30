classdef packWheelPoses < matlab.System
    % WheelPosesSystem updates wheel poses by applying a rotation and a vertical translation.
    %
    % This System object receives the previous wheel poses (wheelPosesPrev),
    % Euler angles roll, pitch, yaw, and a vertical offset dz. It computes an
    % initial tire z-coordinate from wheelPosesPrev during the first execution
    % of stepImpl and uses it for subsequent updates.

    properties (Access = private)
        tireZ_initial = zeros(4,1);
        % 初回実行判定用フラグ
        isInitialized = false;
    end

    methods (Access = protected)
        function wheelPoses = stepImpl(obj, wheelPosesPrev, roll, pitch, yaw, dz)
            % 初回実行時にtireZ_initialが未設定なら初期化する
            if ~obj.isInitialized
                obj.tireZ_initial = squeeze(wheelPosesPrev(3, 4, 1:4));
                obj.isInitialized = true;
            end

            % 前回のポーズをコピー
            wheelPoses = wheelPosesPrev;

            % 回転行列の計算
            rotmat = eul2rotm([yaw, roll, pitch], "ZYX");  % RRSではy軸が前方->yaw,roll,pitch

            % 各車輪の回転成分を更新
            wheelPoses(1:3, 1:3, 1:4) = rotmat;

            % tireZ_initialとdzを用いてz座標を更新
            wheelPoses(3, 4, 1:4) = obj.tireZ_initial + dz;
        end

        function resetImpl(obj)
            % オブジェクトの状態をリセット
            obj.tireZ_initial = zeros(4,1);
            obj.isInitialized = false;
        end
        function sz = getOutputSizeImpl(obj)
            % wheelPoses のサイズは入力 wheelPosesPrev のサイズに伝播
            sz = propagatedInputSize(obj, 1);
        end

        function flag = isOutputFixedSizeImpl(obj)
            % wheelPoses は入力と同じ固定サイズかどうか
            flag = true;
        end

        function dt = getOutputDataTypeImpl(obj)
            % wheelPoses のデータ型は入力と同じ
            dt = propagatedInputDataType(obj, 1);
        end

        function flag = isOutputComplexImpl(obj)
            % wheelPoses は入力と同じ複素数フラグを伝播
            flag = false;
        end
    end
end
