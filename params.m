clear P

% --- Vehicle mass and geometry ---
P.vehicleMass = 278.10;                  % car plus driver [kg]
P.gravity = 9.80665;                     % gravitational acceleration [m/s^2]
P.wheelbase = 1.53035;                   % front axle to rear axle [m]
P.trackWidth = 1.22;                     % left tire center to right tire center [m]
P.centerOfGravityHeight = 0.254;         % CG height above the ground [m]
P.frontAxleToCg = 29.402 * 0.0254;       % front axle to CG [m]
P.rearAxleToCg = 30.848 * 0.0254;        % CG to rear axle [m]
P.staticRearWeightFraction = ...
    P.frontAxleToCg / P.wheelbase;

% --- Tire and wheel ---
P.wheelRadius = 0.203;                   % loaded tire radius [m]
P.tireMagicFormulaB = 10.4;              % Magic Formula stiffness factor
P.tireMagicFormulaC = 1.58;              % Magic Formula shape factor
P.tirePeakMuBase = 3.02;                 % peak mu before load sensitivity
P.tirePeakMuLoadSensitivity = 0.0008 / 4.448221615;  % mu loss per newton, converted from per lbf
P.globalGripScale = 1.0;                 % multiplies all tire grip
P.tireRelaxationLength = 0.25;           % tire force lag length [m]
P.useTireRelaxation = true;
P.slipSpeedFloor = 1.0;                  % minimum speed in slip denominator [m/s]


% --- Powertrain ---
P.gearRatio = 10;                        % motor speed / wheel speed
P.drivetrainEfficiency = 0.89;           % torque delivered after drivetrain losses
P.peakMotorTorque = 21.0;                % peak torque per motor [Nm]
P.peakMotorPowerPerMotor = 20.0e3;       % peak power per motor [W]
P.vehiclePowerLimit = 80.0e3;            % FSAE EV total power limit [W]
P.maxMotorSpeed = 12000 * 2*pi/60;       % rated motor speed [rad/s]
P.maxWheelSpeed = P.maxMotorSpeed / P.gearRatio;
P.motorTorqueTimeConstant = 0.010;       % inverter plus motor torque lag [s]

% --- Rotational inertia ---
P.rotorInertia = 2.74e-4;                % motor rotor inertia [kg*m^2]
P.wheelInertia = 0.16490 / 2;            % one wheel/tire/hub equivalent [kg*m^2]
P.gearboxInertia = 8.60e-4;              % motor-side, disc approx [kg*m^2]
P.combinedWheelInertia = ...
    (P.rotorInertia + P.gearboxInertia) * P.gearRatio^2 + P.wheelInertia;

% --- Aero ---
P.airDensity = 1.225;                    % air density [kg/m^3]
P.frontalArea = 1.0;                     % reference area [m^2]
P.dragCoefficient = 1.0;
P.downforceCoefficient = 3.8;
P.rearDownforceFraction = 0.53;          % aero load fraction on rear axle
P.rollingResistanceCoefficient = 0.015;

% --- Traction controller ---
P.targetSlipRatio = 0.148;
P.slipProportionalGain = 30;
P.slipIntegralGain = 300;
P.antiWindupGain = 15;                   % back-calculation gain [1/s]

P.lowSpeedFloor = 0.1;                   % yaw-rate speed floor [m/s]
P.useGripFeedforward = true;
P.slipFilterCutoffHz = 0;                % 0 means off; slip filter handled elsewhere
P.useFrictionEllipse = true;             % derate longitudinal grip when cornering
P.useCornerSpeedEstimate = true;         % estimate each tire's ground speed from yaw

% --- Maneuver ---
P.initialSpeed = 0.3;                    % initial vehicle speed [m/s]
P.simulationEndTime = 10;                % Simulink stop time [s]
P.solverName = 'ode23t';                 % stiff solver for fast wheel/tire modes
P.maxTimeStep = 1e-3;

% --- Initial stuff ---
initialWheelSpeed = P.initialSpeed / P.wheelRadius;
initialState = [
    P.initialSpeed
    initialWheelSpeed * ones(4,1)
    zeros(4,1)
    zeros(4,1)
];
controllerInitialIntegralState = zeros(1,4);

assignin('base','P',P);
assignin('base','X0',initialState);
assignin('base','I0',controllerInitialIntegralState);

fprintf('params.m: grip=%.2f  gear=%.2f  peak torque=%.1f Nm  inertia=%.4f kg*m^2\n', ...
    P.globalGripScale, P.gearRatio, P.peakMotorTorque, P.combinedWheelInertia);
