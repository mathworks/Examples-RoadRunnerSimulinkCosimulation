function x = StateFcn_bicycle(x, A, B, u, timeStep)
% A = vinput.A;
% B = vinput.B;
% u = vinput.U;
% timeStep = vinput.timeStep;
% x = x + timeStep * A * x + B * u;

x = A * x + B * u; % A and B is already discretized

% % Continuous-time model
% Ac = [0 1 0 0 0 0;
%      0 -1/tau 0 0 0 0;
%      0 0 -(2*Cf+2*Cr)/(m*Vx) 0 -Vx-(2*Cf*lf - 2*Cr*lr)/(m*Vx) 0;
%      0 0 0 0 1 0;
%      0 0 -(2*Cf*lf-2*Cr*lr)/(Iz*Vx) 0 -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*Vx) 0;
%      0 0 1 0 0 0];
% Bc = [0 0; 1/tau 0; 0 2*Cf/m; 0 0; 0 2*Cf*lf/Iz; 0 0];
% Cc = [1 0 0 0 0 0; 0 0 0 0 0 1; 0 0 0 1 0 0];
% Dc = zeros(size(Cc,1),size(Bc,2));
% x = x + vdpStateFcnContinuous(x)*timeStep;
end
% 
% function dxdt = vdpStateFcnContinuous(x)
% %vdpStateFcnContinuous Evaluate the van der Pol ODEs for mu = 1
% dxdt = [x(2); (1-x(1)^2)*x(2)-x(1)];
% end