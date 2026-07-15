clear
clc
params

modelName = "CP27E_TC";

if ~bdIsLoaded(modelName)
    open_system(modelName)
end

set_param(modelName, ...
    "SolverType", "Variable-step", ...
    "SolverName", P.solver, ...
    "StopTime", string(P.tEnd), ...
    "MaxStep", string(P.maxStep), ...
    "ReturnWorkspaceOutputs", "on");

simulationOutput = sim(modelName);

loggedSignals = simulationOutput.logsout;

getLoggedSignalValues = @(signalName) ...
    loggedSignals.get(signalName).Values;

% --- Sim Vars ---
time = getLoggedSignalValues("vx").Time;
schedule = getLoggedSignalValues("sched").Data(:);
vehicleSpeed = getLoggedSignalValues("vx").Data(:);
longitudinalAccel = getLoggedSignalValues("ax").Data(:);
lateralAccel = getLoggedSignalValues("ay").Data(:);
requestedTorque = getLoggedSignalValues("Treq").Data(:);
wheelSpeed = asFourColumnMatrix(getLoggedSignalValues("w").Data);
slipRatio = asFourColumnMatrix(getLoggedSignalValues("slip").Data);
commandedTorque = asFourColumnMatrix(getLoggedSignalValues("Tcmd").Data);
normalLoad = asFourColumnMatrix(getLoggedSignalValues("Fz").Data);
muUtilization = asFourColumnMatrix(getLoggedSignalValues("mu_util").Data);
longitudinalForceCapacity = asFourColumnMatrix(getLoggedSignalValues("Fx_cap").Data);
maxDriveTorque = asFourColumnMatrix(getLoggedSignalValues("Tmax_drv").Data);

distance = cumtrapz(time, vehicleSpeed);
wheelLabels = {'FL','FR','RL','RR'};
eventTimes = P.t1;

% --- Accel nums ---
targetDistance = 75;
firstIndexPastTarget = find(distance >= targetDistance, 1);

if isempty(firstIndexPastTarget)
    timeTo75m = NaN;
    fprintf('run_sim: reached only %.1f m by t=%.2f s (never hit 75 m)\n', distance(end), time(end));
elseif firstIndexPastTarget == 1
    timeTo75m = time(1);
    speedAt75m = vehicleSpeed(1);
    fprintf('run_sim: 0-75 m in %.3f s (v=%.1f m/s)\n', timeTo75m, speedAt75m);
else
    indexBeforeTarget = firstIndexPastTarget - 1;
    indexAfterTarget = firstIndexPastTarget;

    distanceBefore = distance(indexBeforeTarget);
    distanceAfter = distance(indexAfterTarget);
    interpolationFraction = (targetDistance - distanceBefore) / (distanceAfter - distanceBefore);

    timeTo75m = time(indexBeforeTarget) + ...
        interpolationFraction * (time(indexAfterTarget) - time(indexBeforeTarget));
    speedAt75m = vehicleSpeed(indexBeforeTarget) + ...
        interpolationFraction * (vehicleSpeed(indexAfterTarget) - vehicleSpeed(indexBeforeTarget));

    fprintf('run_sim: 0-75 m in %.3f s (v=%.1f m/s)\n', timeTo75m, speedAt75m);
end

% --- front/rear avg ---
frontSlipRatio = mean(slipRatio(:,1:2),2);
rearSlipRatio = mean(slipRatio(:,3:4),2);
frontCommandedTorque = mean(commandedTorque(:,1:2),2);
rearCommandedTorque = mean(commandedTorque(:,3:4),2);
frontNormalLoad = mean(normalLoad(:,1:2),2);
rearNormalLoad = mean(normalLoad(:,3:4),2);

% --- Motor max torque-speed ---
motorSpeed = max(abs(vehicleSpeed)/P.Rw*P.gear, 1);
torqueEnvelope = min(P.Tmot_pk, min(P.Pmot_pk, P.Pcap_veh/4)./motorSpeed);

plotEndTime = min(time(end), timeTo75m*1.05);
if isnan(plotEndTime), plotEndTime = time(end); end

% --- plots ---
fig = figure('Name','CP27E accel 0-75m','Color','w','Position',[60 60 1280 760]);
tl  = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

nexttile
plot(time, frontSlipRatio, 'b', time, rearSlipRatio, 'r', 'LineWidth', 1.2)
hold on
yline(P.slip_tgt, 'k--', 'target')
grid on
xlim([0 plotEndTime])
title('slip ratio (axle avg)'); xlabel('t [s]'); ylabel('\sigma')
legend({'front','rear'},'Location','best'); ylim([-0.05 0.5]);

nexttile
plot(time, frontCommandedTorque, 'b', time, rearCommandedTorque, 'r', 'LineWidth', 1.2)
hold on
plot(time, torqueEnvelope, 'k--', 'LineWidth', 1)
grid on
xlim([0 plotEndTime])
title('motor torque (axle avg)'); xlabel('t [s]'); ylabel('N\cdotm')
legend({'front','rear','T_{max} envelope'},'Location','best');

nexttile
plot(time, frontNormalLoad, 'b', time, rearNormalLoad, 'r', 'LineWidth', 1.2)
grid on
xlim([0 plotEndTime])
title('normal load (axle avg)'); xlabel('t [s]'); ylabel('F_z [N]')
legend({'front','rear'},'Location','best');

nexttile
plot(time, longitudinalAccel, 'LineWidth', 1.2)
hold on
grid on
xlim([0 plotEndTime])
if ~isnan(timeTo75m)
    xline(timeTo75m, 'k:', '75 m')
end
title('longitudinal accel'); xlabel('t [s]'); ylabel('a_x [m/s^2]');

nexttile
plot(time, distance, 'LineWidth', 1.2)
hold on
grid on
xlim([0 plotEndTime])
yline(targetDistance, 'k:')
if ~isnan(timeTo75m)
    xline(timeTo75m, 'k:', '75 m')
end
title('position'); xlabel('t [s]'); ylabel('x [m]');

title(tl, sprintf('CP27E accel  |  0-75 m = %.3f s  |  gear=%.1f  T_{pk}=%.0f Nm  P=%.0f kW/motor', ...
    timeTo75m, P.gear, P.Tmot_pk, min(P.Pmot_pk,P.Pcap_veh/4)/1e3));

outdir = 'plots';
if ~exist(outdir,'dir'); 
    mkdir(outdir); 
end

fname = fullfile(outdir, sprintf('accel_%s.png', datestr(now,'yyyy-mm-dd_HHMMSS')));
exportgraphics(fig, fname, 'Resolution', 200);
fprintf('saved %s\n', fname);

function matrix = asFourColumnMatrix(data)
    matrix = squeeze(data);
    if size(matrix,1)==4 && size(matrix,2)~=4
        matrix = matrix.';
    end
end
