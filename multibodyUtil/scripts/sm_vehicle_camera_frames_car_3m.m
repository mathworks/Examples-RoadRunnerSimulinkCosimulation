function Camera =  sm_vehicle_camera_frames_car_3m
% Function to specify parameters for animation cameras
%
% Adjust frame locations below
%
% Copyright 2021-2024 The MathWorks, Inc.

% Offset from vehicle reference to camera reference
%   Vehicle Reference: Frame where camera frame subsystem is attached
%   Camera Reference:  Frame camera will point at

camera_param.veh_to_cam = [   -1.5      0        0.7];

% Camera positions relative to camera reference
% Circle of cameras
camera_param.xyz_f      = [   5      0        1.1];   % Front
camera_param.xyz_l      = [   0      5        0.65];  % Left  (right)
camera_param.xyz_r      = [  -5      0        1.1];   % Rear
camera_param.xyz_d      = [   3.84   3.2      1.1];   % Front Left (diagonal)
camera_param.xyz_t      = [-eps      0       10];     % Top

% Viewing Suspension and Seats
camera_param.whl_fl     = [   1.41    3      -0.35];  % Wheel Front Left (right)
camera_param.whl_rl     = [  -1.41    3      -0.35];  % Wheel Rear Left  (right)
camera_param.susp_f     = [   3.00    0      -0.35];  % Suspension Front
camera_param.susp_r     = [  -3.00    0      -0.35];  % Suspension Rear
camera_param.susp_fl    = [   0.80    0.45   -0.35];  % Suspension Front Left (right) 
camera_param.susp_rl    = [  -0.70    0.45   -0.35];  % Suspension Rear Left  (right) 
camera_param.seat_fl    = [   0.10    0.38    0.55];  % Seat Front Left       (right) 

% Dolly Camera
camera_param.dolly.s    = [   7.00   -5.00    1.30];  % Camera offset
camera_param.dolly.a    = [   0      15.00  155];     % View angle orientation
camera_param.dolly.tvec = [0   1   2    3    4    5    7   10    20]; % Time
camera_param.dolly.xvec = [5   6.1 8.6 12.5 17.7 24.5 44.1 79.9 199]; % Trajectory x
camera_param.dolly.yvec = [0   0   0    0    0    0    0    0     0]; % Trajectory y
camera_param.dolly.zvec = [0   0   0    0    0    0    0    0     0]; % Trajectory z

% Obtain Camera structure
Camera = sm_vehicle_camera_generate_structure(camera_param);
