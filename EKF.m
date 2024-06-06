clear all; close all; clc;
setupProj;

% Constants
g = -9.81;          % Acceleration Due to Gravity [m/s/s]

% System Parameters
Jp = 2.5;           % Mass Moment of Inertia @ Pin [Nm rad/s/s]
m = 1.6;            % Mass [kg]
l = 1;              % Length [m]
J = Jp + m*l^2;     % Mass Moment of Inertia @ CG [Nm rad/s/s]
b = 1.25;           % Rotational Damping Constant [Nm/rad/s]

% Noise Parameters
sigmaE = sqrt(deg2rad(1));
sigmaT = sqrt(deg2rad(0.5));
sigmaF = sqrt(2);

% Time Parameters
Fs = 20;            % Sampling Frequency [Hz]
dt = 1/Fs;          % Sampling Period [s]
time = 0:dt:10;     % Time Vector [s]
N = length(time);   % Number of Samples

%% Simulation
initialAngle = deg2rad(30);
y = zeros(2,N);     % Measurement Vector [deg : deg/s]
truth = zeros(2,N); % Truth Vector [deg]
[y(1,1), truth(1,1)] = deal(initialAngle);   % Initial Angle [deg]
[y(1,1), y(2,1), truth(1,1), truth(2,1)] = ...
    simPendulum(0, y(1,1), ...
        PendulumType="normal", ...
        SystemSigma=sigmaF, ...
        EncoderSigma=sigmaE, ...
        EncoderBias=0, ...
        TachometerSigma=sigmaT, ...
        TachometerBias=0);
for i = 2:N
  [y(1,i), y(2,i), truth(1,i), truth(2,i)] = simPendulum(0);
end

%% Extend Kalman Filter
C = [1 0];                      % Observation Matrix
Bw = [0;
      1];                       % Input Noise Matrix
Qc = sigmaF^2;                  % Continuous Process Covariance
R = diag([sigmaE^2 sigmaT^2]);  % Measurement Covariance
P = Bw*Qc*Bw'.*dt;              % State Covariance
x = zeros(2,N);                 % State
x(1,1) = initialAngle;          % Initial State
for k = 2:N
    % Relinearization
    A = [                 0             1;
         ((m*g*l)/J)*x(1,k-1)  -(b/J)*x(2,k-1)];    % Dynamic Matrix
    Phi = expm(A*dt);
    % Time Update
    xp = [x(1,k-1) + x(2,k-1)*dt;
          x(2,k-1) + ((m*g*l)*sin(x(1,k-1))/J - b*x(2,k-1)/J)*dt];
    Pp = Phi*P*Phi' + Bw*Qc*Bw'.*dt;
    % Kalman Gain
    % Using C to transform R so that user can test with one measurement 
    % or both
    L = Pp*C'/(C*Pp*C' + C*R*C');
    % Measurement Update
    % Using C to transform y so that the user can test with one 
    % measurement or both
    x(:,k) = xp + L*(C*y(:,k-1) - C*xp);
    P = (eye(2) - L*C)*Pp;
end

figure();
tiledlayout(2,1);
nexttile();
hold('on');
plot(time, rad2deg(x(1,:)), 'LineWidth', 2);
plot(time, rad2deg(truth(1,:)), 'LineWidth', 2);
title('Pendulum Angle');
subtitle('\theta vs. Time');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('State Estimate', 'True State');
ax = gca;
ax.FontSize = 16;
nexttile();
hold('on');
plot(time, rad2deg(x(2,:)), 'LineWidth', 2);
plot(time, rad2deg(truth(2,:)), 'LineWidth', 2);
title('Pendulum Angular Velocity');
subtitle('\omega vs. Time');
ylabel('Angular Velocity (deg/s)');
xlabel('Time (s)');
legend('State Estimate', 'True State');
ax = gca;
ax.FontSize = 16;