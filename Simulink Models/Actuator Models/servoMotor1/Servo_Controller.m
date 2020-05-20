%% Parameters for Servo Control

%% Control Parameters
Ts  = 5e-6;  % Fundamental sample time            [s]
Tsc = 1e-4;  % Sample time for inner control loop [s]
Vdc = 48;    % Maximum DC link voltage            [V]
Kpw = 0.2;   % Proportional gain speed controller
Kiw = 18;    % Integrator gain speed controller
Kpv = 0.2;   % Proportional gain voltage controller
Kiv = 0.4;   % Integrator gain voltage controller