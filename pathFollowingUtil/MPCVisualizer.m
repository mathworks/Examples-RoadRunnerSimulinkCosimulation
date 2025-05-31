classdef MPCVisualizer < matlab.System
    %MPCVisualizer MATLAB System for visualizing MPC predictions
    %   This System object visualizes measured outputs, manipulated variables,
    %   predicted states, and references over the prediction horizon. It
    %   supports saving the animation as a video.
    %
    %   Usage:
    %     sys = MPCVisualizer;
    %     setup(sys);
    %     while simulation_running
    %         measured = <4x1 vector>;
    %         manipulated = <(N+1)x2 matrix>;
    %         state = <(N+1)x7 predicted states>;
    %         reference = <N x 4 reference>;
    %         step(sys, measured, manipulated, state, reference);
    %     end
    %     release(sys);

    % 公開プロパティ — ユーザが Simulink ブロック上で設定可能
    properties (Access = public, Nontunable)
        % PredictionHorizon Length of prediction horizon (N), 予測ホライゾンの長さ (N)
        PredictionHorizon (1,1) {mustBePositive, mustBeInteger} = 10;
        % SampleTime SampleTime[s] サンプリングタイム [秒]
        SampleTime (1,1) {mustBePositive} = 0.1;
        % VideoFlag if true, video is saved. true にすると動画保存を行う
        VideoFlag (1,1) logical = false;
        % VideoFileName video file name with extention. 保存する動画ファイル名 (拡張子付き)
        VideoFileName (1,:) char = 'mpc_visualization.mp4';

        % Y axis limit of each graph. 各グラフの縦軸リミット ([nan, nan] の場合は auto でスケーリング)
        YLimVx        (1,2) double = [0, 25];
        YLimY         (1,2) double = [-10, 10];
        YLimYaw       (1,2) double = [-pi/6, pi/6];
        YLimYawRate   (1,2) double = [-pi/6, pi/6];
        YLimSteer     (1,2) double = [-pi/6, pi/6];
        YLimAx        (1,2) double = [-10, 2];
        YLimVy        (1,2) double = [-2, 2];
    end

    % 内部で使うハンドルなど
    properties (Access = private)
        FigHandle       % Figure handle
        AxesHandles     % 各サブプロットの Axes handle (7 個)
        LineHandles     % 各ラインのハンドルを構造体で保持
        VideoWriterObj  % VideoWriter オブジェクト
        TimeVector      % 時間軸ベクトル [0:N]
    end

    methods
        % コンストラクタ
        function obj = MPCVisualizer(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            % 初期設定：Figure／Axes／Line オブジェクトの生成、動画ファイル準備
            N = obj.PredictionHorizon;
            % 時間軸：0 から N
            obj.TimeVector = 0 : 1 : N;

            % 図の作成
            obj.FigHandle = figure('Name','MPC Visualization','NumberTitle','off');
            % 4 行 × 2 列 ＝ 8 プロットのうち、7 プロット（縦加速度を統合）
            obj.AxesHandles = gobjects(7,1);
            obj.LineHandles = struct();

            %% 1) Longitudinal Acceleration a_x (Manipulated & Predicted)
            obj.AxesHandles(1) = subplot(4,2,1, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(1), 'on'); grid(obj.AxesHandles(1), 'on');
            title(obj.AxesHandles(1), 'Longitudinal Acceleration a_x (m/s^2)');
            xlabel(obj.AxesHandles(1), 'Time Step');
            ylabel(obj.AxesHandles(1), 'a_x');
            if all(isfinite(obj.YLimAx))
                ylim(obj.AxesHandles(1), obj.YLimAx);
            end
            % Manipulated: 階段状に表示
            obj.LineHandles.MVLongAccel = stairs(obj.AxesHandles(1), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'r-', 'LineWidth', 1.5);
            % Predicted: 連続線
            obj.LineHandles.PredAx       = plot(obj.AxesHandles(1), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b--', 'LineWidth', 1.5);
            legend(obj.AxesHandles(1), {'MV a_x','Pred a_x'});

            %% 2) Steering Angle δ (Manipulated Variable)
            obj.AxesHandles(2) = subplot(4,2,2, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(2), 'on'); grid(obj.AxesHandles(2), 'on');
            title(obj.AxesHandles(2), 'Steering Angle \delta (rad)');
            xlabel(obj.AxesHandles(2), 'Time Step');
            ylabel(obj.AxesHandles(2), '\delta');
            if all(isfinite(obj.YLimSteer))
                ylim(obj.AxesHandles(2), obj.YLimSteer);
            end
            % Manipulated: 階段状表示
            obj.LineHandles.MVSteer = stairs(obj.AxesHandles(2), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'r-', 'LineWidth', 1.5);
            % Predicted: 連続線
            obj.LineHandles.PredSteer       = plot(obj.AxesHandles(2), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b--', 'LineWidth', 1.5);
            legend(obj.AxesHandles(2), {'MV steer','Pred steer'});

            %% 3) Longitudinal Velocity Vx (state/参照/測定)
            obj.AxesHandles(3) = subplot(4,2,3, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(3), 'on'); grid(obj.AxesHandles(3), 'on');
            title(obj.AxesHandles(3), 'Longitudinal Velocity V_x (m/s)');
            xlabel(obj.AxesHandles(3), 'Time Step');
            ylabel(obj.AxesHandles(3), 'V_x');
            if all(isfinite(obj.YLimVx))
                ylim(obj.AxesHandles(3), obj.YLimVx);
            end
            obj.LineHandles.PredVx = plot(obj.AxesHandles(3), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b-', 'LineWidth', 1.5);
            obj.LineHandles.RefVx  = plot(obj.AxesHandles(3), ...
                1:N, nan(1,N), 'r--', 'LineWidth', 1.5);
            obj.LineHandles.MeaVx  = plot(obj.AxesHandles(3), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'k:', 'LineWidth', 1.5);
            legend(obj.AxesHandles(3), {'Predicted','Reference','Measured'});

            %% 4) Lateral Position Y (state/参照/測定)
            obj.AxesHandles(4) = subplot(4,2,4, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(4), 'on'); grid(obj.AxesHandles(4), 'on');
            title(obj.AxesHandles(4), 'Lateral Position Y (m)');
            xlabel(obj.AxesHandles(4), 'Time Step');
            ylabel(obj.AxesHandles(4), 'Y');
            if all(isfinite(obj.YLimY))
                ylim(obj.AxesHandles(4), obj.YLimY);
            end
            obj.LineHandles.PredY = plot(obj.AxesHandles(4), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b-', 'LineWidth', 1.5);
            obj.LineHandles.RefY  = plot(obj.AxesHandles(4), ...
                1:N, nan(1,N), 'r--', 'LineWidth', 1.5);
            obj.LineHandles.MeaY  = plot(obj.AxesHandles(4), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'k:', 'LineWidth', 1.5);
            legend(obj.AxesHandles(4), {'Predicted','Reference','Measured'});

            %% 5) Yaw Angle ψ (state/参照/測定)
            obj.AxesHandles(5) = subplot(4,2,5, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(5), 'on'); grid(obj.AxesHandles(5), 'on');
            title(obj.AxesHandles(5), 'Yaw Angle \psi (rad)');
            xlabel(obj.AxesHandles(5), 'Time Step');
            ylabel(obj.AxesHandles(5), '\psi');
            if all(isfinite(obj.YLimYaw))
                ylim(obj.AxesHandles(5), obj.YLimYaw);
            end
            obj.LineHandles.PredYaw      = plot(obj.AxesHandles(5), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b-', 'LineWidth', 1.5);
            obj.LineHandles.RefYaw       = plot(obj.AxesHandles(5), ...
                1:N, nan(1,N), 'r--', 'LineWidth', 1.5);
            obj.LineHandles.MeaYaw       = plot(obj.AxesHandles(5), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'k:', 'LineWidth', 1.5);
            legend(obj.AxesHandles(5), {'Predicted','Reference','Measured'});

            %% 6) Yaw Rate \dot{ψ} (state/参照/測定)
            obj.AxesHandles(6) = subplot(4,2,6, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(6), 'on'); grid(obj.AxesHandles(6), 'on');
            title(obj.AxesHandles(6), 'Yaw Rate \dot{\psi} (rad/s)');
            xlabel(obj.AxesHandles(6), 'Time Step');
            ylabel(obj.AxesHandles(6), '\dot{\psi}');
            if all(isfinite(obj.YLimYawRate))
                ylim(obj.AxesHandles(6), obj.YLimYawRate);
            end
            obj.LineHandles.PredYawRate  = plot(obj.AxesHandles(6), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b-', 'LineWidth', 1.5);
            obj.LineHandles.RefYawRate   = plot(obj.AxesHandles(6), ...
                1:N, nan(1,N), 'r--', 'LineWidth', 1.5);
            obj.LineHandles.MeaYawRate   = plot(obj.AxesHandles(6), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'k:', 'LineWidth', 1.5);
            legend(obj.AxesHandles(6), {'Predicted','Reference','Measured'});

            %% 7) Lateral Velocity V_y (state)
            obj.AxesHandles(7) = subplot(4,2,8, 'Parent', obj.FigHandle);
            hold(obj.AxesHandles(7), 'on'); grid(obj.AxesHandles(7), 'on');
            title(obj.AxesHandles(7), 'Predicted Lateral Velocity V_y (m/s)');
            xlabel(obj.AxesHandles(7), 'Time Step');
            ylabel(obj.AxesHandles(7), 'V_y');
            if all(isfinite(obj.YLimVy))
                ylim(obj.AxesHandles(7), obj.YLimVy);
            end
            obj.LineHandles.PredVy       = plot(obj.AxesHandles(7), ...
                obj.TimeVector, nan(size(obj.TimeVector)), 'b-', 'LineWidth', 1.5);

            % 動画保存フラグが true の場合は VideoWriter を開く
            if obj.VideoFlag
                obj.VideoWriterObj = VideoWriter(obj.VideoFileName, 'MPEG-4');
                % フレームレートを 1/SampleTime に設定
                obj.VideoWriterObj.FrameRate = 1 / obj.SampleTime;
                open(obj.VideoWriterObj);
            end

            drawnow;
        end

        function stepImpl(obj, measured, manipulated, state, reference)
            %stepImpl 毎ステップで呼び出され、グラフを更新・描画
            %   measured     : [4x1] => [Vx; Y; psi; psi_dot]
            %   manipulated  : [(N+1)x2] => [a_x_horizon, delta_horizon]
            %   state        : [(N+1)x7] => [delta; vx; ax; vy; psi; psi_dot; y]_pred
            %   reference    : [N x 4]  => [vx_ref, y_ref, psi_ref, psi_dot_ref]

            N    = obj.PredictionHorizon;
            tVec = obj.TimeVector;

            %% 1) Longitudinal Accel (Manipulated & Predicted)
            ax_horizon = manipulated(:,1);
            set(obj.LineHandles.MVLongAccel, ...
                'XData', tVec, ...
                'YData', ax_horizon(:)');
            ax_pred = state(:,3);
            set(obj.LineHandles.PredAx, ...
                'XData', tVec, ...
                'YData', ax_pred(:)');

            %% 2) Steering Angle (Manipulated Variable)
            steer_horizon = manipulated(:,2);
            steer_pred = state(:,1);
            set(obj.LineHandles.MVSteer, ...
                'XData', tVec, ...
                'YData', steer_horizon(:)');
            set(obj.LineHandles.PredSteer, ...
                'XData', tVec, ...
                'YData', steer_pred(:)');

            %% 3) Vx（予測 / 参照 / 測定）
            vx_pred = state(:,2);
            vx_ref  = reference(:,1);
            vx_mea  = measured(1);
            set(obj.LineHandles.PredVx, ...
                'XData', tVec, ...
                'YData', vx_pred(:)');
            set(obj.LineHandles.RefVx, ...
                'XData', 1:N, ...
                'YData', vx_ref(:)');
            set(obj.LineHandles.MeaVx, ...
                'XData', tVec, ...
                'YData', repmat(vx_mea, size(tVec)));

            %% 4) Y（予測 / 参照 / 測定）
            y_pred = state(:,7);
            y_ref  = reference(:,2);
            y_mea  = measured(2);
            set(obj.LineHandles.PredY, ...
                'XData', tVec, ...
                'YData', y_pred(:)');
            set(obj.LineHandles.RefY, ...
                'XData', 1:N, ...
                'YData', y_ref(:)');
            set(obj.LineHandles.MeaY, ...
                'XData', tVec, ...
                'YData', repmat(y_mea, size(tVec)));

            %% 5) Yaw (ψ)（予測 / 参照 / 測定）
            psi_pred = state(:,5);
            psi_ref  = reference(:,3);
            psi_mea  = measured(3);
            set(obj.LineHandles.PredYaw, ...
                'XData', tVec, ...
                'YData', psi_pred(:)');
            set(obj.LineHandles.RefYaw, ...
                'XData', 1:N, ...
                'YData', psi_ref(:)');
            set(obj.LineHandles.MeaYaw, ...
                'XData', tVec, ...
                'YData', repmat(psi_mea, size(tVec)));

            %% 6) Yaw Rate (\dot{ψ})（予測 / 参照 / 測定）
            psi_dot_pred = state(:,6);
            psi_dot_ref  = reference(:,4);
            psi_dot_mea  = measured(4);
            set(obj.LineHandles.PredYawRate, ...
                'XData', tVec, ...
                'YData', psi_dot_pred(:)');
            set(obj.LineHandles.RefYawRate, ...
                'XData', 1:N, ...
                'YData', psi_dot_ref(:)');
            set(obj.LineHandles.MeaYawRate, ...
                'XData', tVec, ...
                'YData', repmat(psi_dot_mea, size(tVec)));

            %% 7) Lateral Velocity V_y (状態)
            vy_pred = state(:,4);
            set(obj.LineHandles.PredVy, ...
                'XData', tVec, ...
                'YData', vy_pred(:)');

            drawnow limitrate;

            % 動画フラグが立っていればフレームをキャプチャして書き込む
            if obj.VideoFlag
                frame = getframe(obj.FigHandle);
                writeVideo(obj.VideoWriterObj, frame);
            end
        end

        function resetImpl(~)
            % resetImpl：特に状態を持たないので何もしない
        end

        function releaseImpl(obj)
            % releaseImpl：リソース解放（動画保存終了、図を閉じる）
            if obj.VideoFlag && ~isempty(obj.VideoWriterObj)
                close(obj.VideoWriterObj);
            end
            if ishandle(obj.FigHandle)
                close(obj.FigHandle);
            end
        end
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end