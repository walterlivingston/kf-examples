function [theta_meas, thetad_meas, theta_new, thetad_new] = ...
    simPendulum(input, initialAngle, options)
%SIMPENDULUM A pendulum simulation
%   This function simulates a pendulum.  An input can be passed in to
%   control the pendulum.  It provides a true (affected by
%   process noise) and measured values for angle and angular velocity of
%   the pendulum.
% Inputs:
%   input           :   A control input to the pendulum. Torque or Voltage
%                       depending on the input options. [Nm; V]
%   initialAngle    :   The initial angle of the pendulum. Only passed once
%                       for initialization. [rad]
%   options:
%       PendulumType    :   Type of pendulum; normal or inverted.
%                           (default = normal)
%       InputType       :   Units of input; torq or volt (default = torq)
%       SystemSigma     :   Standard deviation on process noise. 
%                           (default = 1N) [N]
%       EncoderSigma    :   Standard deviation on angle measurement noise.
%                           (default = 1rad) [rad]
%       EncoderBias     :   Bias on angle measurement. 
%                           (default = 0 rad) [rad]
%       TachometerSigma :   Standard deviation on angular rate measurement.
%                           noise (default = 1rad/s) [rad/s]
%       TachometerBias  :   Bias on angular rate measurement.
%                           (default = 0rad) [rad]
% Outputs:
%   theta_meas      :   Measured pendulum angle. [rad]
%   thetad_meas     :   Measured pendulum angular rate. [rad/s]
%   theta_new       :   True pendulum angle. [rad]
%   thetad_new      :   True pendulum angular rate. [rad]
% Example:
%      clear all; close all; clc;
%        
%      Fs = 20;                % Sampling Frequency [Hz]
%      dt = 1/Fs;              % Sampling Period [s]
%      time = 0:dt:10;         % Time Vector [s]
%      N = length(time);       % Number of Samples
%
%      y = zeros(2,N);         % Measurement Vector [deg : deg/s]
%      truth = zeros(2,N);     % Truth Vector [deg]
%      y(1,1) = deg2rad(10);   % Initial Angle [deg]
%      [y(1,1), y(2,1), truth(1,1), truth(2,1)] = ...
%         simPendulum(0, y(1,1), ...
%            PendulumType="inverted");
%      for i = 2:N
%          [y(1,i), y(2,i), truth(1,i), truth(2,i)] = simPendulum(0);
%      end

arguments
    input double
    initialAngle double = NaN
    options.PendulumType (1,1) string = "normal"
    options.InputType (1,1) string = "torq"
    options.SystemSigma double = 1
    options.EncoderSigma double = 1
    options.EncoderBias double = 0
    options.TachometerSigma double = 1
    options.TachometerBias double = 0
end

    persistent theta_old thetad_old
    persistent PendulumType InputType
    persistent sigmaF sigmaE biasE sigmaT biasT

    if isempty(PendulumType); PendulumType = options.PendulumType; end
    if isempty(InputType); InputType = options.InputType; end
    if isempty(sigmaF); sigmaF = options.SystemSigma; end
    if isempty(sigmaE); sigmaE = options.EncoderSigma; end
    if isempty(biasE); biasE = options.EncoderBias; end
    if isempty(sigmaT); sigmaT = options.TachometerSigma; end
    if isempty(biasT); biasT = options.TachometerBias; end

    % Constants
    g = -9.81;          % Acceleration Due to Gravity [m/s/s]

    % System Parameters
    Jp = 2.5;           % Mass Moment of Inertia @ Pin [Nm rad/s/s]
    m = 1.6;            % Mass [kg]
    l = 1;              % Length [m]
    J = Jp + m*l^2; % Mass Moment of Inertia @ CG [Nm rad/s/s]
    b = 1.25;           % Rotational Damping Constant [Nm/rad/s]
    
    % Simulation Parameters
    Fs = 20;            % Simulation Frequency [Hz]
    dt = 1/Fs;          % Simulation Time Step [s]
    
    if ~isnan(initialAngle)
        theta_old = initialAngle;
        thetad_old = 0;
    end

    if PendulumType == "inverted"
        inverted = -1;
    else
        inverted = 1;
    end

    % (TODO) Implement Voltage Control
    if InputType ~= "torq"
        
    end

    F = 5 + sigmaF*randn;
    thetadd_new = inverted.*((m*g*l)*sin(theta_old) - ...
        (F*l)*cos(theta_old) - b*thetad_old + input)/J;
    thetad_new = thetad_old + thetadd_new*dt;
    theta_new = theta_old + thetad_new*dt;

    theta_old = theta_new;
    thetad_old = thetad_new;

    encoder_noise = sigmaE*randn + biasE;
    tachometer_noise = sigmaT*randn + biasT;

    theta_meas = theta_new + encoder_noise;
    thetad_meas = thetad_new + tachometer_noise;
end

