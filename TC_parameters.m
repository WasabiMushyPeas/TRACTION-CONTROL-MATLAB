%% CP27E parameters for the simple 0-75 m traction-control simulation
% Source: D:\CLUBS\FSAE\CP27E_Vehicle_Parameters.md (2026-09-24)
% Current CP27 PDR values are used when the source documents conflict.

%% Units
lb_to_kg = 0.45359237;          % [kg/lbm]
in_to_m = 0.0254;               % [m/in]

%% Vehicle mass and geometry (current CP27 working values)
vehicleMassNoDriver = 473.92 * lb_to_kg;  % [kg]
driverMass = 150.00 * lb_to_kg;           % [kg]
Mv = vehicleMassNoDriver + driverMass;    % [kg], 283.00 kg rounded in PDR

W = 60.25 * in_to_m;                     % Wheelbase [m]
trackFront = 49.0 * in_to_m;              % Front track [m]
trackRear = 49.0 * in_to_m;               % Rear track [m]
h_cg = 10.4 * in_to_m;                    % CG height [m] at 1.5 in ride height
r = 0.5 * 16.0 * in_to_m;                 % Nominal tire radius [m]
gravity = 9.80665;                         % Standard gravity [m/s^2]

frontWeightFraction = 0.50;                % Static weight distribution [-]
rearWeightFraction = 1 - frontWeightFraction;
cg_f = W * rearWeightFraction;             % Front axle to CG [m]
cg_r = W * frontWeightFraction;            % CG to rear axle [m]
halfcar_a = cg_f;                           % Compatibility alias [m]
halfcar_b = cg_r;                           % Compatibility alias [m]

unsprungMassPerCorner = 26.0 * lb_to_kg;   % Current projection [kg/corner]

%% Static loads
Fz_front_static = Mv * gravity * cg_r / W; % Front axle normal load [N]
Fz_rear_static = Mv * gravity * cg_f / W;  % Rear axle normal load [N]
Fz_static_rear = Fz_rear_static;           % Compatibility alias [N]

%% Four-motor AMK drivetrain (current CP27 working values)
numDrivenWheels = 4;
fd = 12.5;                                 % Nominal motor-to-wheel ratio [-]
gearboxEfficiency = 0.85;                  % Minimum drivetrain efficiency [-]
motorInverterEfficiency = 1.00;             % Simple-model assumption [-]
Drive_Train = gearboxEfficiency;            % Compatibility alias [-]
Max_Motor_Torque = 21.5;                   % Per-motor stall torque [N*m]
Max_Motor_RPM = 20000;                     % Motor speed limit [rpm]
Max_Wheel_Omega = Max_Motor_RPM * 2*pi/60 / fd; % Wheel speed limit [rad/s]
maxTractivePower = 80e3;                   % E-meter power limit [W]

T_request_per_motor = Max_Motor_Torque;    % Full-pedal request [N*m/motor]
T_request = numDrivenWheels * T_request_per_motor; % Aggregate request [N*m]

%% Aerodynamics (use area coefficients directly)
rho = 1.225;                                % Air density assumption [kg/m^3]
CDA = 1.20;                                 % Current drag-area target [m^2]
CLA = 4.40;                                 % Current downforce-area target [m^2]
aeroFrontFraction = 0.50;                   % Current front COP fraction [-]
Cp = 1 - aeroFrontFraction;                 % Rear aero-load fraction [-]

% Compatibility aliases for models that calculate 0.5*rho*A*C*v^2.
A = 1.0;                                    % Reference area [m^2]
Cd = CDA / A;                               % Equivalent drag coefficient [-]
Cl = CLA / A;                               % Equivalent downforce coefficient [-]

%% Four independent traction controllers
Ts = 0.002;                                 % 500 Hz TC execution rate [s]
Ts_plant = 0.0001;                          % Stiff wheel/tire plant integration step [s]
slipTolerance = 0.02;                       % Settling band [-]

% Each corner remains independently calibratable. The straight-line tune is
% left/right symmetric and stays below the 0.1476 Pacejka peak.
Slip_Target_FL = 0.145;
Slip_Target_FR = 0.145;
Slip_Target_RL = 0.145;
Slip_Target_RR = 0.145;

% Realistic driver-request feedforward.  It is sampled at 500 Hz, delayed
% by one controller tick, and slew limited before the residual PI cut.
Kff_FL = 1.0;
Kff_FR = 1.0;
Kff_RL = 1.0;
Kff_RR = 1.0;
Feedforward_RiseRate_FL = 800;              % [N*m/s]
Feedforward_RiseRate_FR = 800;              % [N*m/s]
Feedforward_RiseRate_RL = 800;              % [N*m/s]
Feedforward_RiseRate_RR = 800;              % [N*m/s]
Feedforward_FallRate_FL = 2000;             % [N*m/s]
Feedforward_FallRate_FR = 2000;             % [N*m/s]
Feedforward_FallRate_RL = 2000;             % [N*m/s]
Feedforward_FallRate_RR = 2000;             % [N*m/s]
Slip_Filter_Hz = 25;                        % Wheel-slip measurement filter [Hz]
Slip_Filter_Alpha = 1 - exp(-2*pi*Slip_Filter_Hz*Ts);

% Legacy launch-ramp values retained for comparison sweeps.
Launch_Ramp_Time_FL = 0.005;                % [s]
Launch_Ramp_Time_FR = 0.005;                % [s]
Launch_Ramp_Time_RL = 0.005;                % [s]
Launch_Ramp_Time_RR = 0.005;                % [s]
Feedforward_Filter_Hz = 20;                 % Load-estimate low-pass cutoff [Hz]
Feedforward_Filter_Alpha = 1 - exp(-2*pi*Feedforward_Filter_Hz*Ts);

Kp_FL = 20; Ki_FL = 240; Kd_FL = 0;
Kp_FR = 20; Ki_FR = 240; Kd_FR = 0;
Kp_RL = 20; Ki_RL = 240; Kd_RL = 0;
Kp_RR = 20; Ki_RR = 240; Kd_RR = 0;

Kaw_FL = Ki_FL / Kp_FL;
Kaw_FR = Ki_FR / Kp_FR;
Kaw_RL = Ki_RL / Kp_RL;
Kaw_RR = Ki_RR / Kp_RR;
Ktrack_FL = 50;                              % Final-saturation tracking [1/s]
Ktrack_FR = 50;
Ktrack_RL = 50;
Ktrack_RR = 50;

Cmin = -Max_Motor_Torque;                   % Residual PID lower limit [N*m]
Cmax = 0;                                   % TC residual can cut but not add torque [N*m]

%% Current CP27 longitudinal Pacejka coefficients
% Preserve these signs. The tire block converts normal load to kN and uses
% mu = -(D1 + D2*Fz_kN), with the force sign corrected for negative C.
% The source file does not state D2 units or the complete fitted equation,
% so the 1/kN load-sensitivity interpretation remains provisional.
Pacejka_B = 10.400;
Pacejka_C = -1.580;
Pacejka_D1 = -3.020;
Pacejka_D2 = 0.800;
Grip_Fact = 1.0;                            % Nominal dry grip scale [-]

% Start the controller-side load filters at the known static tire capacity.
% This avoids an artificial zero-load transient at the beginning of launch.
mu_front_initial = max(-(Pacejka_D1 + ...
    Pacejka_D2*(Fz_front_static/2)/1000)*Grip_Fact, 0);
mu_rear_initial = max(-(Pacejka_D1 + ...
    Pacejka_D2*(Fz_rear_static/2)/1000)*Grip_Fact, 0);
Feedforward_InitialState_FL = (1-Feedforward_Filter_Alpha) * ...
    mu_front_initial*(Fz_front_static/2);
Feedforward_InitialState_FR = Feedforward_InitialState_FL;
Feedforward_InitialState_RL = (1-Feedforward_Filter_Alpha) * ...
    mu_rear_initial*(Fz_rear_static/2);
Feedforward_InitialState_RR = Feedforward_InitialState_RL;

%% Simple-model assumptions (not released CP27 vehicle parameters)
% Replace these when measured CP27 tire and inertia data become available.
J_wheel_side = 0.45;                        % Effective inertia per corner [kg*m^2]
J = numDrivenWheels * J_wheel_side;         % Aggregate compatibility alias [kg*m^2]
J_Motor = 0;                                % Included in J_wheel_side [kg*m^2]
J_Wheel = J;                                % Compatibility alias [kg*m^2]
tau_motor = 0.03;                           % Torque-response time constant [s]
% The lag allows a small delivered-power overshoot above the command limit.
slipSpeedFloor = 0.50;                      % Low-speed slip denominator [m/s]

%% 0-75 m simulation and acceptance targets
distanceTarget = 75;                        % MIS acceleration distance [m]
timeMax = 10;                               % Simulation timeout [s]
timedomain = timeMax;                       % Compatibility alias [s]
accelTimeTarget = 3.87;                     % Maximum target time [s]
peakAccelFirstSecondTarget = 1.45;          % Minimum [g]
averageAccelTarget = 1.02;                  % Equivalent 75 m acceleration [g]
