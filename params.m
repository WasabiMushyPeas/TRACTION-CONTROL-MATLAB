%% Run and plot the CP27E 0-75 m traction-control simulation
% Edit controller and vehicle values in TC_parameters.m, then run this file.

run('TC_parameters.m');

in = Simulink.SimulationInput('TC');
in = in.setModelParameter('StopTime', num2str(timeMax));
out = sim(in);
simout = out;  % Compatibility alias for interactive workspace use.

finishTime = out.tout(end);
finishSpeed = out.vehicleSpeed.Data(end);
peakAccelFirstSecondG = max( ...
    out.acceleration.Data(out.acceleration.Time <= 1)) / gravity;
equivalentAverageAccelG = 2 * distanceTarget / finishTime^2 / gravity;

fprintf('CP27E four-wheel TC simulation\n');
fprintf('  Time to 75 m:             %.3f s (target <= %.2f s)\n', ...
    finishTime, accelTimeTarget);
fprintf('  Speed at 75 m:            %.2f m/s (%.1f km/h)\n', ...
    finishSpeed, 3.6 * finishSpeed);
fprintf('  Peak accel in first 1 s:  %.2f g (target >= %.2f g)\n', ...
    peakAccelFirstSecondG, peakAccelFirstSecondTarget);
fprintf('  Equivalent average accel: %.2f g (target >= %.2f g)\n', ...
    equivalentAverageAccelG, averageAccelTarget);
fprintf('  Minimum power scale:      %.3f\n', min(out.powerScale.Data));

wheelLabels = {'FL', 'FR', 'RL', 'RR'};
slipTargets = [Slip_Target_FL, Slip_Target_FR, ...
    Slip_Target_RL, Slip_Target_RR];

figure('Name', 'CP27E Four-Wheel Traction Control', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(out.distance.Time, out.distance.Data, 'LineWidth', 1.5);
hold on;
yline(distanceTarget, '--k', '75 m target');
grid on;
xlabel('Time [s]');
ylabel('Distance [m]');
title(sprintf('Distance — finish %.3f s', finishTime));

nexttile;
plot(out.vehicleSpeed.Time, out.vehicleSpeed.Data, 'k', 'LineWidth', 1.7);
hold on;
plot(out.wheelSpeeds.Time, out.wheelSpeeds.Data, 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Speed [m/s]');
title('Vehicle and wheel peripheral speeds');
legend([{'Vehicle'}, wheelLabels], 'Location', 'best');

nexttile;
plot(out.wheelSlips.Time, out.wheelSlips.Data, 'LineWidth', 1.2);
hold on;
targetTime = [out.wheelSlips.Time(1); out.wheelSlips.Time(end)];
plot(targetTime, repmat(slipTargets, 2, 1), '--', 'LineWidth', 0.9);
grid on;
xlabel('Time [s]');
ylabel('Slip ratio [-]');
title('Independent wheel-slip control');
legend([wheelLabels, strcat(wheelLabels, ' target')], 'Location', 'best');

nexttile;
yyaxis left;
plot(out.motorTorques.Time, out.motorTorques.Data, 'LineWidth', 1.2);
ylabel('Motor torque [N m]');
yyaxis right;
plot(out.powerScale.Time, out.powerScale.Data, 'k--', 'LineWidth', 1.3);
ylabel('80 kW power scale [-]');
grid on;
xlabel('Time [s]');
title('Per-motor torque and power limiting');
legend([wheelLabels, {'Power scale'}], 'Location', 'best');
