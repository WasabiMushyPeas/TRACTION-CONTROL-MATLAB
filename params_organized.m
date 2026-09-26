%% Run and plot the organized CP27E 0-75 m traction-control simulation
% Runs TC_organized with the realistic 500 Hz feedforward and residual PID.
% Edit controller and vehicle values in TC_parameters.m, then run this file.

run('TC_parameters.m');

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

fprintf('CP27E organized four-wheel TC simulation (realistic 500 Hz feedforward)\n');
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

%% Performance overview
performanceFigure = figure('Name', 'CP27E Organized Performance Overview', ...
    'Color', 'w');
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
styleSimulationFigure(performanceFigure);

%% Aerodynamic loads, normal loads, and electrical constraints
loadsFigure = figure('Name', 'CP27E Organized Loads and Power', 'Color', 'w');
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
styleSimulationFigure(loadsFigure);

%% Per-wheel torque requests, limits, commands, and actual response
torqueFigure = figure('Name', 'CP27E Organized Per-Wheel Torque Envelope', ...
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

torqueTitle = sgtitle( ...
    'Simplified power- and speed-limited per-wheel torque envelope');
torqueTitle.Color = 'k';
styleSimulationFigure(torqueFigure);

%% Feedforward and residual PID contributions during launch
controllerFigure = figure('Name', ...
    'CP27E Organized Realistic-Feedforward Controller Contributions', ...
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

controllerTitle = sgtitle( ...
    'Realistic 500 Hz feedforward plus residual PID -- first 0.5 s');
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
