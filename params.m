%% CP27E 0-75 m traction-control simulation: parameters, run, and plots
% Edit vehicle and controller values in the parameter sections, then run
% this file to simulate TC_organized and plot the results.
% TC_organized's InitFcn sets TC_paramsOnly and runs this file, so pressing
% Run in Simulink loads only the parameters (the guard below stops there).

%% Parameters
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

% Grip-based feedforward: filtered mu*Fz at the slip target converted to
% motor torque, plus wheel-inertia torque, capped by the driver request.
% It is sampled at 500 Hz and slew limited before the residual PI.
% Kff < 1 keeps the estimate conservative so the PI closes the last gap.
Kff_FL = 0.9;
Kff_FR = 0.9;
Kff_RL = 0.9;
Kff_RR = 0.9;
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
Cmax = 0.25 * Max_Motor_Torque;             % Residual may add torque up to the driver request [N*m]

%% Current CP27 longitudinal Pacejka coefficients
% Preserve these signs. The tire block converts normal load to kN and uses
% mu = -(D1 + D2*Fz_kN), with the force sign corrected for negative C.
% The source file does not state D2 units or the complete fitted equation,
% so the 1/kN load-sensitivity interpretation remains provisional.
Pacejka_B = 10.400;
Pacejka_C = -1.580;
Pacejka_D1 = -3.020;
Pacejka_D2 = 0.800;
% The raw fit gives mu = 2.46 at static corner load, typical of unscaled
% tire-rig belt data. 0.60 scales it to mu = 1.48 for FSAE slicks on
% asphalt, which makes the fronts traction-limited at launch.
Grip_Fact = 0.60;                           % Road-surface grip scale [-]

% Static tire friction, shown in the figure titles.
tireMuAtLoad = @(Fz) max(-(Pacejka_D1 + Pacejka_D2*Fz/1000)*Grip_Fact, 0);
mu_front_initial = tireMuAtLoad(Fz_front_static/2);
mu_rear_initial = tireMuAtLoad(Fz_rear_static/2);

% Predict the launch operating point: each corner delivers the lesser of its
% motor force and tire capacity, and the resulting acceleration transfers
% load rearward. Iterate the coupled loads to a fixed point.
motorForceMax = Max_Motor_Torque*fd*Drive_Train/r; % Per corner [N]
Launch_Accel_Predicted = 0;                         % [m/s^2]
for launchIteration = 1:50
    launchTransfer = Mv*Launch_Accel_Predicted*h_cg/(2*W); % Per corner [N]
    Fz_front_launch = Fz_front_static/2 - launchTransfer;  % [N]
    Fz_rear_launch = Fz_rear_static/2 + launchTransfer;    % [N]
    launchForce = 2*min(motorForceMax, tireMuAtLoad(Fz_front_launch)*Fz_front_launch) + ...
        2*min(motorForceMax, tireMuAtLoad(Fz_rear_launch)*Fz_rear_launch);
    Launch_Accel_Predicted = 0.5*Launch_Accel_Predicted + 0.5*launchForce/Mv;
end
clear launchIteration launchTransfer launchForce

% Start the controller-side load filters at the predicted launch capacity
% so the front feedforward does not begin at the static front load.
% The Direct form II state is w = y/alpha, so a first output of mu*Fz
% needs an initial state of mu*Fz/alpha.
muFz_front_launch = tireMuAtLoad(Fz_front_launch)*Fz_front_launch; % [N]
muFz_rear_launch = tireMuAtLoad(Fz_rear_launch)*Fz_rear_launch;    % [N]
Feedforward_InitialState_FL = muFz_front_launch / Feedforward_Filter_Alpha;
Feedforward_InitialState_FR = Feedforward_InitialState_FL;
Feedforward_InitialState_RL = muFz_rear_launch / Feedforward_Filter_Alpha;
Feedforward_InitialState_RR = Feedforward_InitialState_RL;

% Launch with the pedal already held at full, so the feedforward starts at
% its grip-limited launch value instead of slew limiting up from zero.
Launch_Pedal_Initial = T_request_per_motor; % [N*m]
targetForceShape = @(slipTarget) -sin(Pacejka_C*atan(Pacejka_B*slipTarget));
Feedforward_Launch_FL = min(Launch_Pedal_Initial, Kff_FL*r/(fd*Drive_Train) * ...
    muFz_front_launch*targetForceShape(Slip_Target_FL));
Feedforward_Launch_FR = min(Launch_Pedal_Initial, Kff_FR*r/(fd*Drive_Train) * ...
    muFz_front_launch*targetForceShape(Slip_Target_FR));
Feedforward_Launch_RL = min(Launch_Pedal_Initial, Kff_RL*r/(fd*Drive_Train) * ...
    muFz_rear_launch*targetForceShape(Slip_Target_RL));
Feedforward_Launch_RR = min(Launch_Pedal_Initial, Kff_RR*r/(fd*Drive_Train) * ...
    muFz_rear_launch*targetForceShape(Slip_Target_RR));

%% Simple-model assumptions (not released CP27 vehicle parameters)
% Replace these when measured CP27 tire and inertia data become available.
J_wheel_side = 0.45;                        % Effective inertia per corner [kg*m^2]
J = numDrivenWheels * J_wheel_side;         % Aggregate compatibility alias [kg*m^2]
J_Motor = 0;                                % Included in J_wheel_side [kg*m^2]
J_Wheel = J;                                % Compatibility alias [kg*m^2]
tau_motor = 0.005;                          % Torque-response time constant [s], AMK-like
% The lag allows a small delivered-power overshoot above the command limit.
slipSpeedFloor = 0.50;                      % Low-speed slip denominator [m/s]

%% 0-75 m simulation and acceptance targets
distanceTarget = 75;                        % MIS acceleration distance [m]
timeMax = 10;                               % Simulation timeout [s]
timedomain = timeMax;                       % Compatibility alias [s]
accelTimeTarget = 3.87;                     % Maximum target time [s]
peakAccelFirstSecondTarget = 1.45;          % Minimum [g]
averageAccelTarget = 1.02;                  % Equivalent 75 m acceleration [g]

% Stop here when the model's InitFcn only needs the parameters.
if exist('TC_paramsOnly', 'var') && TC_paramsOnly
    return
end

%% Run TC_organized with the 500 Hz grip-based feedforward and residual PID
modelName = 'TC_organized';
in = Simulink.SimulationInput(modelName);
in = in.setModelParameter('StopTime', num2str(timeMax));
out = sim(in);
simout = out;  % Compatibility alias for interactive workspace use.

%% Completion and headline metrics
distanceData = out.distance.Data(:);
distanceTime = out.distance.Time(:);
distanceTolerance = 1e-3;  % [m], only avoids rejecting roundoff at 75 m.

if isempty(distanceData) || any(~isfinite(distanceData))
    error('CP27E:InvalidDistance', ...
        'The simulation did not return a finite distance history.');
end

finishIndex = find(distanceData >= distanceTarget, 1, 'first');
if isempty(finishIndex)
    finishIndex = find(distanceData >= distanceTarget - distanceTolerance, ...
        1, 'first');
end
if isempty(finishIndex)
    error('CP27E:DistanceNotReached', ...
        ['The simulation stopped at %.2f m after %.2f s without reaching ' ...
         'the %.2f m target. Increase timeMax or inspect the controller.'], ...
        distanceData(end), distanceTime(end), distanceTarget);
end

if finishIndex == 1
    finishTime = distanceTime(1);
else
    finishTime = interp1(distanceData(finishIndex-1:finishIndex), ...
        distanceTime(finishIndex-1:finishIndex), distanceTarget, ...
        'linear', 'extrap');
end
finishSpeed = interp1(out.vehicleSpeed.Time(:), ...
    out.vehicleSpeed.Data(:), finishTime, 'linear', 'extrap');
firstSecondMask = out.acceleration.Time(:) <= min(1, finishTime);
peakAccelFirstSecondG = max( ...
    out.acceleration.Data(firstSecondMask)) / gravity;
equivalentAverageAccelG = 2 * distanceTarget / finishTime^2 / gravity;

%% Electrical power and motor-speed diagnostics
% wheelSpeeds is tire peripheral speed, so divide by tire radius before
% applying the final-drive ratio. The simple model assumes the electrical
% power crossing the E-meter is motor mechanical power divided by the
% motor/inverter efficiency.
wheelOmega = out.wheelSpeeds.Data / r;
motorRPM = abs(wheelOmega * fd * 60 / (2*pi));

requestWheelSpeed = interp1(out.wheelSpeeds.Time(:), ...
    out.wheelSpeeds.Data, out.torqueRequests.Time(:), 'linear', 'extrap');
commandWheelSpeed = interp1(out.wheelSpeeds.Time(:), ...
    out.wheelSpeeds.Data, out.torqueCommands.Time(:), 'linear', 'extrap');
actualWheelSpeed = interp1(out.wheelSpeeds.Time(:), ...
    out.wheelSpeeds.Data, out.motorTorques.Time(:), 'linear', 'extrap');

requestedElectricalPower = sum(out.torqueRequests.Data .* ...
    (requestWheelSpeed / r) * fd / motorInverterEfficiency, 2);
commandedElectricalPower = sum(out.torqueCommands.Data .* ...
    (commandWheelSpeed / r) * fd / motorInverterEfficiency, 2);
actualElectricalPower = sum(out.motorTorques.Data .* ...
    (actualWheelSpeed / r) * fd / motorInverterEfficiency, 2);

commandedExcessEnergy = trapz(out.torqueCommands.Time(:), ...
    max(commandedElectricalPower - maxTractivePower, 0));
commandedEnergy = trapz(out.torqueCommands.Time(:), ...
    max(commandedElectricalPower, 0));
commandedExcessEnergyFraction = commandedExcessEnergy / ...
    max(commandedEnergy, eps);
commandedOvershootDuration = trapz(out.torqueCommands.Time(:), ...
    double(commandedElectricalPower > maxTractivePower));

deliveredExcessEnergy = trapz(out.motorTorques.Time(:), ...
    max(actualElectricalPower - maxTractivePower, 0));
deliveredEnergy = trapz(out.motorTorques.Time(:), ...
    max(actualElectricalPower, 0));
deliveredExcessEnergyFraction = deliveredExcessEnergy / ...
    max(deliveredEnergy, eps);

%% Launch-controller diagnostics
wheelLabels = {'FL', 'FR', 'RL', 'RR'};
slipTargets = [Slip_Target_FL, Slip_Target_FR, ...
    Slip_Target_RL, Slip_Target_RR];
slipVehicleSpeed = interp1(out.vehicleSpeed.Time(:), ...
    out.vehicleSpeed.Data(:), out.wheelSlips.Time(:), 'linear', 'extrap');
slipWheelSpeed = interp1(out.wheelSpeeds.Time(:), ...
    out.wheelSpeeds.Data, out.wheelSlips.Time(:), 'linear', 'extrap');
wheelSurfaceExcess = slipWheelSpeed - slipVehicleSpeed;
launchMask = out.wheelSlips.Time(:) >= 0.05 & ...
    out.wheelSlips.Time(:) <= min(0.5, finishTime);
validSlipMask = slipVehicleSpeed >= 2;
launchRequestMask = out.torqueRequests.Time(:) >= 0.05 & ...
    out.torqueRequests.Time(:) <= min(0.5, finishTime);

launchExcessRMS = sqrt(mean( ...
    wheelSurfaceExcess(launchMask, :).^2, 'all'));
launchExcessPeak = max(abs(wheelSurfaceExcess(launchMask, :)), [], 'all');
peakPositiveSlipError = max(out.wheelSlips.Data(validSlipMask, :) - ...
    slipTargets, [], 'all');
requestDeltaRMS = sqrt(mean(diff( ...
    out.torqueRequests.Data(launchRequestMask, :), 1, 1).^2, 'all'));

%% Numerical consistency checks
normalLoadResidual = max(abs(sum(out.normalLoads.Data, 2) - ...
    (Mv * gravity + out.downforce.Data)));
tireForceResidual = max(abs(out.totalTireForce.Data - ...
    sum(out.tireForces.Data, 2)));
vehicleForceResidual = max(abs(Mv * out.acceleration.Data - ...
    (out.totalTireForce.Data - out.aeroDrag.Data)));
requestAtCommandTime = interp1(out.torqueRequests.Time(:), ...
    out.torqueRequests.Data, out.torqueCommands.Time(:), ...
    'previous', 'extrap');
availableAtCommandTime = interp1(out.torqueAvailable.Time(:), ...
    out.torqueAvailable.Data, out.torqueCommands.Time(:), ...
    'previous', 'extrap');
requestTorqueViolation = max(out.torqueCommands.Data - ...
    requestAtCommandTime, [], 'all');
availableTorqueViolation = max(out.torqueCommands.Data - ...
    availableAtCommandTime, [], 'all');
commandPowerViolation = max(commandedElectricalPower - maxTractivePower);
motorSpeedViolation = max(motorRPM(:)) - Max_Motor_RPM;

forceBalanceTolerance = 1e-6;  % [N], far above floating-point residuals.
constraintTolerance = 1e-8;
commandPowerOvershootAllowance = 0.01 * maxTractivePower;
commandExcessEnergyAllowance = 0.01;
assert(normalLoadResidual <= forceBalanceTolerance, ...
    'CP27E:NormalLoadBalance', 'Normal-load balance check failed.');
assert(tireForceResidual <= forceBalanceTolerance, ...
    'CP27E:TireForceBalance', 'Tire-force summation check failed.');
assert(vehicleForceResidual <= forceBalanceTolerance, ...
    'CP27E:VehicleForceBalance', 'Vehicle F=ma check failed.');
assert(requestTorqueViolation <= constraintTolerance, ...
    'CP27E:TorqueRequestLimit', ...
    'A commanded torque exceeded its pre-limit TC request.');
assert(availableTorqueViolation <= constraintTolerance, ...
    'CP27E:TorqueAvailableLimit', ...
    'A commanded torque exceeded the simplified available ceiling.');
assert(commandPowerViolation <= commandPowerOvershootAllowance && ...
        commandedExcessEnergyFraction <= commandExcessEnergyAllowance, ...
    'CP27E:CommandPowerLimit', ...
    ['The commanded electrical-power overshoot exceeded the 1%% peak ' ...
     'or 1%% excess-energy allowance.']);
assert(motorSpeedViolation <= constraintTolerance, ...
    'CP27E:MotorSpeedLimit', 'A motor exceeded the speed limit.');

fprintf('CP27E organized four-wheel TC simulation (grip-based 500 Hz feedforward)\n');
fprintf('  Time to 75 m:             %.3f s (target <= %.2f s)\n', ...
    finishTime, accelTimeTarget);
fprintf('  Speed at 75 m:            %.2f m/s (%.1f km/h)\n', ...
    finishSpeed, 3.6 * finishSpeed);
fprintf('  Peak accel in first 1 s:  %.2f g (target >= %.2f g)\n', ...
    peakAccelFirstSecondG, peakAccelFirstSecondTarget);
fprintf('  Equivalent average accel: %.2f g (target >= %.2f g)\n', ...
    equivalentAverageAccelG, averageAccelTarget);
fprintf('  Minimum power scale:      %.3f\n', min(out.powerScale.Data));
fprintf('  Peak requested power:     %.1f kW\n', ...
    max(requestedElectricalPower) / 1e3);
fprintf('  Peak commanded power:     %.1f kW (limit %.1f kW)\n', ...
    max(commandedElectricalPower) / 1e3, maxTractivePower / 1e3);
fprintf('  Command excess energy:    %.3f%% over %.3f s\n', ...
    100 * commandedExcessEnergyFraction, commandedOvershootDuration);
fprintf('  Peak delivered power:     %.1f kW (%.1f%% peak overshoot)\n', ...
    max(actualElectricalPower) / 1e3, ...
    100 * max(0, max(actualElectricalPower) / maxTractivePower - 1));
fprintf('  Excess delivered energy:  %.3f%% of delivered energy\n', ...
    100 * deliveredExcessEnergyFraction);
fprintf('  Peak motor speed:         %.0f rpm (limit %.0f rpm)\n', ...
    max(motorRPM(:)), Max_Motor_RPM);
fprintf('  Launch wheel excess RMS:  %.3f m/s (peak %.3f m/s)\n', ...
    launchExcessRMS, launchExcessPeak);
fprintf('  Launch request delta RMS: %.4f N m/sample\n', ...
    requestDeltaRMS);
fprintf('  Peak positive slip error: %.4f above target (v >= 2 m/s)\n', ...
    peakPositiveSlipError);
fprintf(['  Math checks:              PASS (load %.1e N, tire sum %.1e N, ' ...
    'F=ma %.1e N)\n'], normalLoadResidual, tireForceResidual, ...
    vehicleForceResidual);

%% Grip label shared by every figure
gripLabel = sprintf(['Grip factor %.2f (\\mu = %.2f front, %.2f rear ' ...
    'at static load)'], Grip_Fact, mu_front_initial, mu_rear_initial);
gripName = sprintf(' - grip %.2f', Grip_Fact);

%% Performance overview
performanceFigure = figure('Name', ['CP27E Organized Performance Overview' ...
    gripName], 'Color', 'w');
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(out.distance.Time, out.distance.Data, 'LineWidth', 1.5);
hold on;
yline(distanceTarget, '--k', '75 m target');
grid on;
xlabel('Time [s]');
ylabel('Distance [m]');
title(sprintf('Distance -- finish %.3f s', finishTime));

nexttile;
plot(out.vehicleSpeed.Time, out.vehicleSpeed.Data, 'k', 'LineWidth', 1.7);
hold on;
plot(out.wheelSpeeds.Time, out.wheelSpeeds.Data, 'LineWidth', 1.0);
yline(Max_Wheel_Omega * r, '--k', 'Wheel-speed limit');
grid on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
title('Vehicle and wheel peripheral speeds');
legend([{'Vehicle'}, wheelLabels, {'Wheel-speed limit'}], ...
    'Location', 'best');

nexttile;
plot(out.wheelSlips.Time, out.wheelSlips.Data, 'LineWidth', 1.2);
hold on;
targetTime = [out.wheelSlips.Time(1); out.wheelSlips.Time(end)];
plot(targetTime, repmat(slipTargets, 2, 1), '--', 'LineWidth', 0.9);
grid on;
ylim([-0.02, 0.17]);
xlabel('Time [s]');
ylabel('Slip ratio [-]');
title('Independent wheel-slip control');
legend([wheelLabels, strcat(wheelLabels, ' target')], 'Location', 'best');

nexttile;
plot(out.acceleration.Time, out.acceleration.Data / gravity, ...
    'LineWidth', 1.4);
hold on;
yline(peakAccelFirstSecondTarget, '--k', 'First-second target');
grid on;
xlabel('Time [s]');
ylabel('Acceleration [g]');
title('Longitudinal acceleration');

nexttile;
plot(out.tireForces.Time, out.tireForces.Data, 'LineWidth', 1.1);
grid on;
xlabel('Time [s]');
ylabel('Longitudinal force [N]');
title('Per-wheel tire forces');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(out.tireMu.Time, out.tireMu.Data, 'LineWidth', 1.1);
grid on;
xlabel('Time [s]');
ylabel('\mu [-]');
title('Load-sensitive tire friction');
legend(wheelLabels, 'Location', 'best');
performanceTitle = sgtitle({'Performance overview', gripLabel});
performanceTitle.Color = 'k';
styleSimulationFigure(performanceFigure);

%% Aerodynamic loads, normal loads, and electrical constraints
loadsFigure = figure('Name', ['CP27E Organized Loads and Power' gripName], ...
    'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(out.aeroDrag.Time, out.aeroDrag.Data, 'LineWidth', 1.4);
hold on;
plot(out.downforce.Time, out.downforce.Data, 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Force [N]');
title('Aerodynamic forces');
legend({'Drag', 'Downforce'}, 'Location', 'best');

nexttile;
plot(out.normalLoads.Time, out.normalLoads.Data, 'LineWidth', 1.1);
grid on;
xlabel('Time [s]');
ylabel('Normal load [N]');
title('Per-wheel normal loads (left/right overlap)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(out.torqueRequests.Time, requestedElectricalPower / 1e3, ...
    'LineWidth', 1.2);
hold on;
plot(out.torqueCommands.Time, commandedElectricalPower / 1e3, ...
    'LineWidth', 1.4);
plot(out.motorTorques.Time, actualElectricalPower / 1e3, ...
    'LineWidth', 1.2);
yline(maxTractivePower / 1e3, '--k', '80 kW command limit');
grid on;
xlabel('Time [s]');
ylabel('Total electrical power [kW]');
title('Requested, commanded, and delivered power');
legend({'Requested', 'Commanded', 'Delivered', 'Command limit'}, ...
    'Location', 'best');

nexttile;
yyaxis left;
plot(out.powerScale.Time, out.powerScale.Data, 'LineWidth', 1.4);
ylabel('Power scale [-]');
ylim([0, 1.05]);
yyaxis right;
plot(out.wheelSpeeds.Time, max(motorRPM, [], 2), 'LineWidth', 1.4);
hold on;
yline(Max_Motor_RPM, '--k', 'Motor-speed limit');
ylabel('Maximum motor speed [rpm]');
grid on;
xlabel('Time [s]');
title('Power and motor-speed limiting');
legend({'Power scale', 'Maximum motor speed', 'Motor-speed limit'}, ...
    'Location', 'best');
loadsTitle = sgtitle({'Loads and power', gripLabel});
loadsTitle.Color = 'k';
styleSimulationFigure(loadsFigure);

%% Per-wheel torque requests, limits, commands, and actual response
torqueFigure = figure('Name', ['CP27E Organized Per-Wheel Torque Envelope' ...
    gripName], ...
    'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for wheelIndex = 1:numel(wheelLabels)
    nexttile;
    plot(out.torqueRequests.Time, ...
        out.torqueRequests.Data(:, wheelIndex), 'LineWidth', 1.2);
    hold on;
    plot(out.torqueAvailable.Time, ...
        out.torqueAvailable.Data(:, wheelIndex), '--', 'LineWidth', 1.5);
    plot(out.torqueCommands.Time, ...
        out.torqueCommands.Data(:, wheelIndex), '-.', 'LineWidth', 1.3);
    plot(out.motorTorques.Time, ...
        out.motorTorques.Data(:, wheelIndex), ':', 'LineWidth', 1.5);
    yline(Max_Motor_Torque, ':k', 'Stall limit', ...
        'HandleVisibility', 'off');
    grid on;
    xlabel('Time [s]');
    ylabel('Motor torque [N m]');
    title([wheelLabels{wheelIndex}, ' motor']);
    legend({'TC request', 'Available ceiling', 'Command', ...
        'Delivered/used'}, ...
        'Location', 'best');
end

torqueTitle = sgtitle({ ...
    'Simplified power- and speed-limited per-wheel torque envelope', ...
    gripLabel});
torqueTitle.Color = 'k';
styleSimulationFigure(torqueFigure);

%% Feedforward and residual PID contributions during launch
controllerFigure = figure('Name', ...
    ['CP27E Organized Grip-Feedforward Controller Contributions' gripName], ...
    'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for wheelIndex = 1:numel(wheelLabels)
    nexttile;
    plot(out.feedforwardTorques.Time, ...
        out.feedforwardTorques.Data(:, wheelIndex), 'LineWidth', 1.4);
    hold on;
    plot(out.pidCorrections.Time, ...
        out.pidCorrections.Data(:, wheelIndex), '--', 'LineWidth', 1.2);
    plot(out.torqueRequests.Time, ...
        out.torqueRequests.Data(:, wheelIndex), 'k', 'LineWidth', 1.3);
    yline(Max_Motor_Torque, ':k', 'Stall limit', ...
        'HandleVisibility', 'off');
    xlim([0, min(0.5, finishTime)]);
    grid on;
    xlabel('Time [s]');
    ylabel('Motor torque [N m]');
    title([wheelLabels{wheelIndex}, ' launch controller']);
    legend({'Feedforward', 'Residual PID', 'Total request'}, ...
        'Location', 'best');
end

controllerTitle = sgtitle({ ...
    'Grip-based 500 Hz feedforward plus residual PID -- first 0.5 s', ...
    gripLabel});
controllerTitle.Color = 'k';
styleSimulationFigure(controllerFigure);

function styleSimulationFigure(figureHandle)
% Keep saved figures readable regardless of the MATLAB desktop theme.
axesHandles = findall(figureHandle, 'Type', 'axes');
set(axesHandles, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.25 0.25 0.25], 'GridAlpha', 0.22);
set(findall(figureHandle, 'Type', 'text'), 'Color', 'k');
legendHandles = findall(figureHandle, 'Type', 'legend');
set(legendHandles, 'Color', 'w', 'TextColor', 'k', ...
    'EdgeColor', [0.35 0.35 0.35]);
end
