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
Drive_Train = 0.85;                        % Minimum drivetrain efficiency [-]
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
slipTolerance = 0.02;                       % Settling band [-]

% Values start equal, but each corner can now be calibrated independently.
Slip_Target_FL = 0.12;
Slip_Target_FR = 0.12;
Slip_Target_RL = 0.12;
Slip_Target_RR = 0.12;

Kp_FL = 32;  Ki_FL = 180;  Kd_FL = 0;
Kp_FR = 32;  Ki_FR = 180;  Kd_FR = 0;
Kp_RL = 32;  Ki_RL = 180;  Kd_RL = 0;
Kp_RR = 32;  Ki_RR = 180;  Kd_RR = 0;

Cmin = -Max_Motor_Torque;                   % Maximum torque correction [N*m]
Cmax = 0;                                   % TC applies cuts only [N*m]

%% Current CP27 longitudinal Pacejka coefficients
% Preserve these signs. The tire block converts normal load to kN and uses
% mu = -(D1 + D2*Fz_kN), with the force sign corrected for negative C.
Pacejka_B = 10.400;
Pacejka_C = -1.580;
Pacejka_D1 = -3.020;
Pacejka_D2 = 0.800;

%% Simple-model assumptions (not released CP27 vehicle parameters)
% Replace these when measured CP27 tire and inertia data become available.
J_wheel_side = 0.45;                        % Effective inertia per corner [kg*m^2]
J = numDrivenWheels * J_wheel_side;         % Aggregate compatibility alias [kg*m^2]
J_Motor = 0;                                % Included in J_wheel_side [kg*m^2]
J_Wheel = J;                                % Compatibility alias [kg*m^2]
tau_motor = 0.03;                           % Torque-response time constant [s]
Grip_Fact = 1.0;                            % Nominal dry grip scale [-]
slipSpeedFloor = 0.50;                      % Low-speed slip denominator [m/s]

%% 0-75 m simulation and acceptance targets
distanceTarget = 75;                        % MIS acceleration distance [m]
timeMax = 10;                               % Simulation timeout [s]
timedomain = timeMax;                       % Compatibility alias [s]
accelTimeTarget = 3.87;                     % Maximum target time [s]
peakAccelFirstSecondTarget = 1.45;          % Minimum [g]
averageAccelTarget = 1.02;                  % Equivalent 75 m acceleration [g]
