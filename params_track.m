%% CP27E track traction-control simulation: corner + low-grip patch at full throttle
% Same car and the same traction controller as params.m / TC_organized.
% TC_track adds a track to drive on, with the pedal held flat the whole way:
%   - a launch straight with a low-grip patch (water or ice) on it,
%   - a corner, so lateral load transfer unloads the inside wheels and the
%     tires share their grip between cornering and traction,
%   - an exit straight to the end of the course.
% The controller blocks are unchanged, so this tests the controller that
% the C++ port implements. The controller still assumes the dry grip
% factor and a single vehicle speed for all four wheels; the plant now
% has per-wheel road grip and per-wheel ground speeds.
% TC_track's InitFcn sets TC_paramsOnly and runs this file, so pressing
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
gearboxEfficiency = 0.92;                  % Drivetrain efficiency [-] (requirement minimum is 0.85)
motorInverterEfficiency = 1.00;             % Simple-model assumption [-]
Drive_Train = gearboxEfficiency;            % Compatibility alias [-]
Max_Motor_Torque = 21.5;                   % Per-motor stall torque [N*m]
Max_Motor_RPM = 20000;                     % Motor speed limit [rpm]
Max_Wheel_Omega = Max_Motor_RPM * 2*pi/60 / fd; % Wheel speed limit [rad/s]
% Torque tapers linearly to zero over the last band below the speed limit,
% so the limiter does not switch torque on and off at 20,000 rpm.
Speed_Limit_Taper_RPM = 500;               % Taper band below the limit [rpm]
Speed_Limit_Taper_Omega = Speed_Limit_Taper_RPM * 2*pi/60 / fd; % [rad/s]
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
Slip_Target_FL = 0.12;
Slip_Target_FR = 0.12;
Slip_Target_RL = 0.145;
Slip_Target_RR = 0.145;

% Grip-based feedforward: filtered mu*Fz at the slip target converted to
% motor torque, plus wheel-inertia torque, capped at the motor limit.
% Fz and mu come from Load_Transfer_Predictor, which predicts load transfer
% from the previous tick's torque commands, so the front feedforward drops
% before the measured front load does.
% It is sampled at 500 Hz and slew limited. The driver throttle only caps
% the final command in the Torque_Limiter, after FF + PID.
% Kff < 1 keeps the estimate conservative so the PID closes the last gap.
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

% Front gains tuned for a small launch overshoot (about 10%) and fastest
% settling (about 80 ms to within 5% of target) with the load predictor.
Kp_FL = 15; Ki_FL = 480; Kd_FL = 0;
Kp_FR = 15; Ki_FR = 480; Kd_FR = 0;
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
Cmax = 0.25 * Max_Motor_Torque;             % PID may add torque; Torque_Limiter caps at throttle [N*m]

%% Current CP27 longitudinal Pacejka coefficients
% Preserve these signs. The tire block reads these values, converts normal
% load to kN, and uses mu = -(D1 + D2*Fz_kN), sign corrected for negative C.
% The source file does not state D2 units or the complete fitted equation,
% so the 1/kN load-sensitivity interpretation remains provisional.
Pacejka_B = 10.400;
Pacejka_C = -1.580;
Pacejka_D1 = -3.020;
Pacejka_D2 = 0.800;
% The raw fit gives mu = 2.46 at static corner load, typical of unscaled
% tire-rig belt data. 0.60 scales it to mu = 1.48 for FSAE slicks on
% asphalt, which makes the fronts traction-limited at launch.
% In TC_track this is the dry road: the plant uses it off the low-grip
% patch, and the controller's feedforward always assumes it.
Grip_Fact = 0.60;                           % Dry road-surface grip scale [-]

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

% Launch with the pedal already held at full (throttle into Torque_Limiter),
% and start the feedforward at its launch value instead of zero.
Launch_Pedal_Initial = T_request_per_motor; % [N*m]
targetForceShape = @(slipTarget) -sin(Pacejka_C*atan(Pacejka_B*slipTarget));
Feedforward_Launch_FL = min(Max_Motor_Torque, Kff_FL*r/(fd*Drive_Train) * ...
    muFz_front_launch*targetForceShape(Slip_Target_FL));
Feedforward_Launch_FR = min(Max_Motor_Torque, Kff_FR*r/(fd*Drive_Train) * ...
    muFz_front_launch*targetForceShape(Slip_Target_FR));
Feedforward_Launch_RL = min(Max_Motor_Torque, Kff_RL*r/(fd*Drive_Train) * ...
    muFz_rear_launch*targetForceShape(Slip_Target_RL));
Feedforward_Launch_RR = min(Max_Motor_Torque, Kff_RR*r/(fd*Drive_Train) * ...
    muFz_rear_launch*targetForceShape(Slip_Target_RR));

%% Simple-model assumptions (not released CP27 vehicle parameters)
% Replace these when measured CP27 tire and inertia data become available.
J_wheel_side = 0.35;                        % Effective inertia per corner [kg*m^2]
J = numDrivenWheels * J_wheel_side;         % Aggregate compatibility alias [kg*m^2]
J_Motor = 0;                                % Included in J_wheel_side [kg*m^2]
J_Wheel = J;                                % Compatibility alias [kg*m^2]
tau_motor = 0.005;                          % Torque-response time constant [s], AMK-like
% The lag allows a small delivered-power overshoot above the command limit.
slipSpeedFloor = 0.50;                      % Low-speed slip denominator [m/s]

%% Track layout
% Distance s is measured along the driven line from the CG start position.
% The corner is an entry ramp, a constant-radius arc, and an exit ramp.
% Curvature ramps linearly on the ramps (clothoids), so the lateral
% acceleration demand v^2*kappa builds smoothly.
%
% Flat out from a standstill, the car can only be traction limited in a
% corner on a corner exit: any corner reached after a straight launch is
% taken above about 20 m/s, where the 80 kW limit holds torque below the
% tire limit, and a tight one cannot be held at that speed. So the default
% is a standing start at the end of a hairpin apex, flat out on the exit,
% with the patch on the exit straight. For a corner after the launch,
% set e.g. Corner_Start = 25 and Corner_Radius = 60.
Corner_Direction = 1;                       % +1 left turn, -1 right turn
Corner_Entry_Length = 10;                   % Curvature ramp-in length [m]
Corner_Radius = 15;                         % Constant-radius (apex) section [m]
Corner_Arc_Length = 10;                     % Constant-radius length [m]
Corner_Exit_Length = 30;                    % Curvature ramp-out length [m]
Corner_Start = -(Corner_Entry_Length + Corner_Arc_Length); % Entry ramp start [m]; < 0 starts in the corner
Course_Length = 100;                        % The simulation stops here [m]

assert(Corner_Entry_Length > 0 && Corner_Arc_Length > 0 && ...
    Corner_Exit_Length > 0, 'CP27E:CornerGeometry', ...
    'Corner entry, arc, and exit lengths must be positive.');
Corner_End = Corner_Start + Corner_Entry_Length + Corner_Arc_Length + ...
    Corner_Exit_Length;                                     % [m]
Corner_Angle_deg = rad2deg((Corner_Arc_Length + 0.5*(Corner_Entry_Length + ...
    Corner_Exit_Length))/Corner_Radius);                   % Whole corner [deg]
Track_Breakpoints = [min(Corner_Start, 0) - 10, Corner_Start, ...
    Corner_Start + Corner_Entry_Length, Corner_End - Corner_Exit_Length, ...
    Corner_End, max(Course_Length, Corner_End) + 50];       % [m]
Track_Curvature = Corner_Direction/Corner_Radius*[0, 0, 1, 1, 0, 0]; % [1/m]

%% Low-grip patch (water or ice)
% The patch scales tire friction the way Grip_Fact does. Each wheel reads
% the surface at its own position, so the fronts reach the patch a
% wheelbase before the rears, and the patch can cover one side (split mu).
% The controller is not told about the patch; it keeps assuming Grip_Fact.
% On the exit straight the car is power limited (about 25 m/s), so water
% there barely makes the wheels slip; ice does. Water hurts on the corner
% exit while the car is still traction limited: try Low_Grip_Start = 12.
Low_Grip_Surface = "ice";                   % "water", "ice", or "dry" (no patch)
switch Low_Grip_Surface
    case "water"
        Low_Grip_Fact = 0.30;               % mu about 0.74 at static load (half of dry)
    case "ice"
        Low_Grip_Fact = 0.06;               % mu about 0.15 at static load
    case "dry"
        Low_Grip_Fact = Grip_Fact;          % Baseline run without a patch
    otherwise
        error('CP27E:LowGripSurface', ...
            'Low_Grip_Surface must be "water", "ice", or "dry".');
end
Low_Grip_Start = 40;                        % Patch start [m]
Low_Grip_Length = 6;                        % Patch length [m]
Low_Grip_Sides = "both";                    % "both", "left", or "right"
Surface_Transition_Length = 0.2;            % Grip changes over about one contact patch [m]

Low_Grip_End = Low_Grip_Start + Low_Grip_Length;            % [m]
Surface_Breakpoints = [min(Low_Grip_Start, 0) - 10, ...
    Low_Grip_Start - Surface_Transition_Length, ...
    Low_Grip_Start, Low_Grip_End, Low_Grip_End + Surface_Transition_Length, ...
    max(Course_Length, Low_Grip_End) + 50];                 % [m]
patchProfile = [0, 0, 1, 1, 0, 0];
Low_Grip_Wheels = [any(Low_Grip_Sides == ["both", "left"]), ...
    any(Low_Grip_Sides == ["both", "right"])];              % [left, right]
% Rows: left side, right side. Columns: Surface_Breakpoints.
Surface_Grip_Table = Grip_Fact + (Low_Grip_Fact - Grip_Fact) * ...
    [Low_Grip_Wheels(1)*patchProfile; Low_Grip_Wheels(2)*patchProfile];
Low_Grip_Wheels = Low_Grip_Wheels([1, 2, 1, 2]);           % FL FR RL RR
clear patchProfile

%% Wheel positions relative to the CG (order FL, FR, RL, RR)
Wheel_Longitudinal_Position = [cg_f; cg_f; -cg_r; -cg_r];  % Forward of CG [m]
Wheel_Lateral_Position = [trackFront/2; -trackFront/2; ...
    trackRear/2; -trackRear/2];                             % Left of CG [m]
Wheel_Side = [1; 2; 1; 2];                                  % Surface_Grip_Table row

%% Cornering: lateral load transfer and combined slip
% Lateral load transfer on each axle is Mv*ay*h_cg/track times that axle's
% share of the roll stiffness. 0.50 matches the 50/50 weight distribution.
LLTD_Front = 0.50;                          % Front share of lateral load transfer [-]
% The tire fit is longitudinal only, so lateral peak friction is assumed
% equal to longitudinal (isotropic friction ellipse).
Lateral_Mu_Ratio = 1.00;                    % Lateral / longitudinal peak mu [-]
Pacejka_Slip_Peak = tan(pi/(2*abs(Pacejka_C)))/Pacejka_B;  % 0.148 [-]

%% Simulation
timeMax = 15;                               % Simulation timeout [s]
timedomain = timeMax;                       % Compatibility alias [s]

% Stop here when the model's InitFcn only needs the parameters.
if exist('TC_paramsOnly', 'var') && TC_paramsOnly
    return
end

%% Run TC_track at full throttle
modelName = 'TC_track';
in = Simulink.SimulationInput(modelName);
in = in.setModelParameter('StopTime', num2str(timeMax));
out = sim(in);
simout = out;  % Compatibility alias for interactive workspace use.

%% Distance axis and course time
distanceTime = out.distance.Time(:);
distanceData = out.distance.Data(:);
if isempty(distanceData) || any(~isfinite(distanceData))
    error('CP27E:InvalidDistance', ...
        'The simulation did not return a finite distance history.');
end
if distanceData(end) < Course_Length - 1e-3
    error('CP27E:CourseNotFinished', ...
        ['The simulation stopped at %.2f m after %.2f s without reaching ' ...
         'the %.0f m course end. Increase timeMax or inspect the model.'], ...
        distanceData(end), distanceTime(end), Course_Length);
end
finishIndex = find(distanceData >= Course_Length, 1, 'first');
finishTime = interp1(distanceData(finishIndex-1:finishIndex), ...
    distanceTime(finishIndex-1:finishIndex), Course_Length);
finishSpeed = interp1(out.vehicleSpeed.Time(:), out.vehicleSpeed.Data(:), ...
    finishTime);

% Distance travelled at each logged sample (logs run at 10 kHz or 500 Hz).
atDistance = @(ts) interp1(distanceTime, distanceData, ts.Time(:), ...
    'linear', 'extrap');

wheelLabels = {'FL', 'FR', 'RL', 'RR'};
slipTargets = [Slip_Target_FL, Slip_Target_FR, ...
    Slip_Target_RL, Slip_Target_RR];
isInside = Corner_Direction*Wheel_Lateral_Position.' > 0;   % Inside wheels

trueSlip = logData(out.trueSlips);           % Tire slip (own ground speed)
trueSlipS = atDistance(out.trueSlips);
measuredSlip = logData(out.wheelSlips);      % Controller slip (CG speed)
measuredSlipS = atDistance(out.wheelSlips);
plantSpeedAtSlip = interp1(out.vehicleSpeed.Time(:), ...
    out.vehicleSpeed.Data(:), out.trueSlips.Time(:));
wheelPosition = trueSlipS + Wheel_Longitudinal_Position.'; % Each wheel's s [m]

ayDemand = logData(out.lateralAccelDemand);
ayActual = logData(out.lateralAccel);
lateralS = atDistance(out.lateralAccel);
lateralOffset = logData(out.lateralOffset);
lateralOffsetS = atDistance(out.lateralOffset);
gripUse = logData(out.gripUse);
gripUseS = atDistance(out.gripUse);
lateralForce = logData(out.lateralForces);
normalLoad = logData(out.normalLoads);
normalLoadS = atDistance(out.normalLoads);

%% Launch metrics (first 8 m or up to the patch, v >= 2 m/s)
launchEnd = min(8, Low_Grip_Start - cg_f);
launchMask = trueSlipS <= launchEnd & plantSpeedAtSlip >= 2;
launchPeakSlip = max(trueSlip(launchMask, :), [], 1);

%% Low-grip patch metrics, per wheel
% On the patch: the wheel's own position is on the low-grip section.
% Recovery: time from leaving the patch until the PID has wound back to
% within 1 N*m of its value before the patch, i.e. the controller has
% stopped holding torque back for the low grip it no longer has.
pidAtSlip = interp1(out.pidCorrections.Time(:), out.pidCorrections.Data, ...
    out.trueSlips.Time(:), 'previous', 'extrap');
patchPeakSlip = nan(1, 4);
patchRecoveryTime = nan(1, 4);
patchEntryTime = nan(1, 4);
patchMinPid = nan(1, 4);
for wheelIndex = find(Low_Grip_Wheels & Low_Grip_Fact ~= Grip_Fact)
    onPatch = wheelPosition(:, wheelIndex) >= Low_Grip_Start & ...
        wheelPosition(:, wheelIndex) <= Low_Grip_End;
    if ~any(onPatch)
        continue
    end
    beforePatch = find(wheelPosition(:, wheelIndex) < ...
        Low_Grip_Start - Surface_Transition_Length, 1, 'last');
    afterPatch = find(wheelPosition(:, wheelIndex) >= ...
        Low_Grip_End + Surface_Transition_Length, 1, 'first');
    patchEntryTime(wheelIndex) = out.trueSlips.Time(find(onPatch, 1, 'first'));
    patchPeakSlip(wheelIndex) = max(trueSlip(onPatch, wheelIndex));
    patchMinPid(wheelIndex) = min(pidAtSlip(beforePatch:afterPatch, wheelIndex));
    recovered = find(pidAtSlip(afterPatch:end, wheelIndex) >= ...
        pidAtSlip(beforePatch, wheelIndex) - 1, 1, 'first');
    if ~isempty(recovered)
        patchRecoveryTime(wheelIndex) = out.trueSlips.Time(afterPatch + recovered - 1) - ...
            out.trueSlips.Time(afterPatch);
    end
end

%% Corner metrics
% The driven part of the corner (it may start behind the start line).
cornerFrom = max(Corner_Start, 0);
cornerMask = trueSlipS >= cornerFrom & trueSlipS <= Corner_End;
cornerPeakSlip = max(trueSlip(cornerMask, :), [], 1);
cornerMeanSlip = mean(trueSlip(cornerMask, :), 1);
measuredSlipAtTrue = interp1(out.wheelSlips.Time(:), measuredSlip, ...
    out.trueSlips.Time(:), 'previous', 'extrap');
cornerSlipBias = mean(measuredSlipAtTrue(cornerMask, :) - ...
    trueSlip(cornerMask, :), 1);           % Controller minus true slip
cornerLateralMask = lateralS >= cornerFrom & lateralS <= Corner_End;
peakLateralG = max(abs(ayActual(cornerLateralMask))) / gravity;
demandMask = cornerLateralMask & abs(ayDemand) > 1;
lateralShortfall = abs(ayDemand(demandMask)) - abs(ayActual(demandMask));
maxLateralShortfall = max([lateralShortfall; 0]);
ranWide = maxLateralShortfall > 0.01;
if ranWide
    firstWideIndex = find(demandMask & abs(ayDemand) - abs(ayActual) > 0.01, ...
        1, 'first');
    firstWideDistance = lateralS(firstWideIndex);
end
maxLateralOffset = max(lateralOffset);
cornerUseMask = gripUseS >= cornerFrom & gripUseS <= Corner_End;
cornerPeakGripUse = max(gripUse(cornerUseMask, :), [], 1);
cornerLoadMask = normalLoadS >= cornerFrom & normalLoadS <= Corner_End;
cornerMinLoad = min(normalLoad(cornerLoadMask, :), [], 1);

%% Electrical power and motor speed
wheelOmega = out.wheelSpeeds.Data / r;
motorRPM = abs(wheelOmega * fd * 60 / (2*pi));
commandWheelSpeed = interp1(out.wheelSpeeds.Time(:), ...
    out.wheelSpeeds.Data, out.torqueCommands.Time(:), 'linear', 'extrap');
actualWheelSpeed = interp1(out.wheelSpeeds.Time(:), ...
    out.wheelSpeeds.Data, out.motorTorques.Time(:), 'linear', 'extrap');
commandedElectricalPower = sum(out.torqueCommands.Data .* ...
    (commandWheelSpeed / r) * fd / motorInverterEfficiency, 2);
actualElectricalPower = sum(out.motorTorques.Data .* ...
    (actualWheelSpeed / r) * fd / motorInverterEfficiency, 2);
commandedExcessEnergyFraction = trapz(out.torqueCommands.Time(:), ...
    max(commandedElectricalPower - maxTractivePower, 0)) / ...
    max(trapz(out.torqueCommands.Time(:), max(commandedElectricalPower, 0)), eps);

%% Numerical consistency checks
normalLoadResidual = max(abs(sum(out.normalLoads.Data, 2) - ...
    (Mv * gravity + out.downforce.Data)));
tireForceResidual = max(abs(out.totalTireForce.Data - ...
    sum(out.tireForces.Data, 2)));
vehicleForceResidual = max(abs(Mv * out.acceleration.Data - ...
    (out.totalTireForce.Data - out.aeroDrag.Data)));
lateralForceResidual = max(abs(sum(lateralForce, 2) - Mv * ayActual));
requestAtCommandTime = interp1(out.torqueRequests.Time(:), ...
    out.torqueRequests.Data, out.torqueCommands.Time(:), 'previous', 'extrap');
requestTorqueViolation = max(out.torqueCommands.Data - ...
    requestAtCommandTime, [], 'all');
commandPowerViolation = max(commandedElectricalPower - maxTractivePower);
motorSpeedViolation = max(motorRPM(:)) - Max_Motor_RPM;

forceBalanceTolerance = 1e-6;  % [N], far above floating-point residuals.
constraintTolerance = 1e-8;
assert(normalLoadResidual <= forceBalanceTolerance, ...
    'CP27E:NormalLoadBalance', ...
    'Normal-load balance check failed (a wheel may have lifted).');
assert(tireForceResidual <= forceBalanceTolerance, ...
    'CP27E:TireForceBalance', 'Tire-force summation check failed.');
assert(vehicleForceResidual <= forceBalanceTolerance, ...
    'CP27E:VehicleForceBalance', 'Vehicle F=ma check failed.');
assert(lateralForceResidual <= forceBalanceTolerance, ...
    'CP27E:LateralForceBalance', 'Lateral F=ma check failed.');
assert(requestTorqueViolation <= constraintTolerance, ...
    'CP27E:TorqueRequestLimit', ...
    'A commanded torque exceeded its pre-limit TC request.');
% The power limit uses the previous 2 ms tick, so a wheel spinning up on the
% patch or in the corner can push the command briefly over the limit. That
% is reported as a warning instead of stopping the run.
powerOvershootPercent = 100 * max(commandPowerViolation, 0) / maxTractivePower;
if powerOvershootPercent > 1 || commandedExcessEnergyFraction > 0.01
    warning('CP27E:CommandPowerLimit', ...
        'Commanded power peaked %.1f%% over the limit (excess energy %.3f%%).', ...
        powerOvershootPercent, 100 * commandedExcessEnergyFraction);
end
% A wheel spinning up on the patch near top speed can overrun the speed
% taper during the motor torque lag, so this is reported, not an error.
if motorSpeedViolation > constraintTolerance
    warning('CP27E:MotorSpeedLimit', ...
        'A motor reached %.0f rpm, %.0f rpm over the speed limit.', ...
        max(motorRPM(:)), motorSpeedViolation);
end

%% Report
turnName = 'left';
if Corner_Direction < 0
    turnName = 'right';
end
sideName = Low_Grip_Sides;
if sideName == "both"
    sideName = "full width";
else
    sideName = sideName + " side only";
end
fprintf('CP27E track TC simulation (full throttle, unchanged TC_organized controller)\n');
fprintf('  Course time (%.0f m):        %.3f s, exit speed %.1f m/s (%.0f km/h)\n', ...
    Course_Length, finishTime, finishSpeed, 3.6*finishSpeed);
fprintf('  Launch peak tire slip:      %s (targets %s, v >= 2 m/s)\n', ...
    sprintf('%.3f ', launchPeakSlip), sprintf('%.3f ', slipTargets));
if Low_Grip_Fact ~= Grip_Fact
    fprintf('  %s patch, %s, %.0f-%.0f m (grip %.2f vs %.2f dry):\n', ...
        Low_Grip_Surface, sideName, Low_Grip_Start, Low_Grip_End, ...
        Low_Grip_Fact, Grip_Fact);
    for wheelIndex = find(~isnan(patchPeakSlip))
        fprintf(['    %s: peak slip %.3f on the patch, PID down to %.1f N m, ' ...
            'wound back %.3f s after leaving it\n'], wheelLabels{wheelIndex}, ...
            patchPeakSlip(wheelIndex), patchMinPid(wheelIndex), ...
            patchRecoveryTime(wheelIndex));
    end
end
cornerGrid = linspace(cornerFrom, Corner_End, 2001);
drivenCornerAngle = rad2deg(abs(trapz(cornerGrid, ...
    interp1(Track_Breakpoints, Track_Curvature, cornerGrid))));
fprintf('  Corner: R %.0f m apex, %.0f deg %s driven, %.0f-%.0f m\n', ...
    Corner_Radius, drivenCornerAngle, turnName, cornerFrom, Corner_End);
fprintf('    Peak lateral accel:       %.2f g\n', peakLateralG);
if ranWide
    fprintf(['    Grip ran out at %.1f m: lateral accel up to %.2f m/s^2 ' ...
        'short, ran %.2f m wide\n'], firstWideDistance, ...
        maxLateralShortfall, maxLateralOffset);
else
    fprintf('    Held the line (no lateral grip shortfall)\n');
end
fprintf('    Peak tire slip:           %s\n', sprintf('%.3f ', cornerPeakSlip));
fprintf('    Mean tire slip:           %s\n', sprintf('%.3f ', cornerMeanSlip));
fprintf('    Controller - tire slip:   %s (mean; single-speed slip bias)\n', ...
    sprintf('%+.3f ', cornerSlipBias));
fprintf('    Peak friction use:        %s\n', sprintf('%.2f ', cornerPeakGripUse));
fprintf('    Lowest normal load:       %s N\n', sprintf('%.0f ', cornerMinLoad));
fprintf(['  Peak commanded power:       %.1f kW (limit %.1f kW, ' ...
    'excess energy %.3f%%)\n'], max(commandedElectricalPower) / 1e3, ...
    maxTractivePower / 1e3, 100 * commandedExcessEnergyFraction);
fprintf('  Peak motor speed:           %.0f rpm (limit %.0f rpm)\n', ...
    max(motorRPM(:)), Max_Motor_RPM);
fprintf(['  Math checks:                PASS (load %.1e N, tire sum %.1e N, ' ...
    'F=ma %.1e N, lateral %.1e N)\n'], normalLoadResidual, ...
    tireForceResidual, vehicleForceResidual, lateralForceResidual);

%% Figure labels shared by every figure
scenarioLabel = sprintf(['%s patch (grip %.2f, %s) at %.0f-%.0f m, ' ...
    'R %.0f m %s corner at %.0f-%.0f m, dry grip %.2f'], ...
    Low_Grip_Surface, Low_Grip_Fact, sideName, Low_Grip_Start, ...
    Low_Grip_End, Corner_Radius, turnName, cornerFrom, Corner_End, Grip_Fact);
bandLabel = 'Blue band: low-grip patch under any wheel. Orange band: corner.';
scenarioName = sprintf(' - %s, grip %.2f', Low_Grip_Surface, Low_Grip_Fact);
patchBand = [Low_Grip_Start - cg_f, Low_Grip_End + cg_r];  % CG distance [m]
if Low_Grip_Fact == Grip_Fact
    patchBand = [];
end
cornerBand = [cornerFrom, Corner_End];
xMax = Course_Length;

%% Track overview
trackFigure = figure('Name', ['CP27E Track Overview' scenarioName], 'Color', 'w');
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Track map from the curvature profile; the car's path adds the lateral offset.
sGrid = (0:0.1:Course_Length)';
heading = cumtrapz(sGrid, interp1(Track_Breakpoints, Track_Curvature, sGrid));
trackX = cumtrapz(sGrid, cos(heading));
trackY = cumtrapz(sGrid, sin(heading));
[offsetDistance, movingIndex] = unique(lateralOffsetS, 'last');  % Skip standstill samples
offsetGrid = interp1(offsetDistance, lateralOffset(movingIndex), sGrid, ...
    'linear', 'extrap');
outwardX = Corner_Direction*sin(heading);   % Outward normal of the turn
outwardY = -Corner_Direction*cos(heading);
nexttile;
plot(trackX, trackY, 'k', 'LineWidth', 1.0);
hold on;
plot(trackX + offsetGrid.*outwardX, trackY + offsetGrid.*outwardY, ...
    'r--', 'LineWidth', 1.4);
if ~isempty(patchBand)
    onPatchGrid = sGrid >= Low_Grip_Start & sGrid <= Low_Grip_End;
    plot(trackX(onPatchGrid), trackY(onPatchGrid), 'b', 'LineWidth', 6);
end
onCornerGrid = sGrid >= Corner_Start & sGrid <= Corner_End;
plot(trackX(onCornerGrid), trackY(onCornerGrid), 'Color', [0.95 0.55 0.1], ...
    'LineWidth', 2.5);
plot(trackX(1), trackY(1), 'ko', 'MarkerFaceColor', 'g');
axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Track (car path dashed red)');
legendEntries = {'Racing line', 'Car path'};
if ~isempty(patchBand)
    legendEntries{end+1} = 'Low-grip patch';
end
legend([legendEntries, {'Corner', 'Start'}], 'Location', 'best');

nexttile;
plot(atDistance(out.vehicleSpeed), out.vehicleSpeed.Data, 'k', 'LineWidth', 1.7);
hold on;
plot(atDistance(out.wheelSpeeds), out.wheelSpeeds.Data, 'LineWidth', 1.0);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Speed [m/s]');
title(sprintf('Speeds -- course time %.3f s', finishTime));
legend([{'Vehicle'}, strcat(wheelLabels, ' wheel surface')], 'Location', 'best');

nexttile;
plot(lateralS, ayDemand / gravity, 'k--', 'LineWidth', 1.4);
hold on;
plot(lateralS, ayActual / gravity, 'r', 'LineWidth', 1.4);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Lateral acceleration [g]');
title('Lateral acceleration: line needs vs tires give');
legend({'Demanded by the line', 'Achieved'}, 'Location', 'best');

nexttile;
plot(lateralOffsetS, lateralOffset, 'r', 'LineWidth', 1.4);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
ylim([-0.1, max(0.5, 1.1 * max(lateralOffset))]);  % Hide solver-tolerance noise
xlabel('Distance [m]');
ylabel('Offset [m]');
title('Running wide (outward offset from the line)');

nexttile;
plot(atDistance(out.acceleration), out.acceleration.Data / gravity, ...
    'LineWidth', 1.4);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Acceleration [g]');
title('Longitudinal acceleration');

nexttile;
plot(atDistance(out.torqueCommands), commandedElectricalPower / 1e3, 'LineWidth', 1.4);
hold on;
plot(atDistance(out.motorTorques), actualElectricalPower / 1e3, 'LineWidth', 1.2);
yline(maxTractivePower / 1e3, '--k', '80 kW limit');
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Electrical power [kW]');
title('Commanded and delivered power');
legend({'Commanded', 'Delivered'}, 'Location', 'best');
trackTitle = sgtitle({'Track overview, full throttle', scenarioLabel, bandLabel});
trackTitle.Color = 'k';
styleSimulationFigure(trackFigure);

%% Wheel slip: what the tire sees vs what the controller sees
slipFigure = figure('Name', ['CP27E Track Wheel Slip' scenarioName], 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for wheelIndex = 1:4
    nexttile;
    plot(trueSlipS, trueSlip(:, wheelIndex), 'LineWidth', 1.4);
    hold on;
    plot(measuredSlipS, measuredSlip(:, wheelIndex), '--', 'LineWidth', 1.1);
    yline(slipTargets(wheelIndex), ':k', 'Target', 'LineWidth', 1.2);
    yline(Pacejka_Slip_Peak, ':', 'Peak \mu', 'Color', [0.5 0.5 0.5]);
    shadeEvents(gca, patchBand, cornerBand);
    grid on;
    xlim([0, xMax]);
    ylim([-0.05, max(0.3, min(1.0, 1.1*max(trueSlip(:, wheelIndex))))]);
    xlabel('Distance [m]');
    ylabel('Slip ratio [-]');
    position = 'outside';
    if isInside(wheelIndex)
        position = 'inside';
    end
    title(sprintf('%s (%s wheel in the corner)', wheelLabels{wheelIndex}, position));
    legend({'Tire slip (own ground speed)', 'Controller slip (CG speed)'}, ...
        'Location', 'best');
end
slipTitle = sgtitle({'Wheel slip', scenarioLabel, bandLabel});
slipTitle.Color = 'k';
styleSimulationFigure(slipFigure);

%% Loads, surface grip, and friction use
loadsFigure = figure('Name', ['CP27E Track Loads and Grip' scenarioName], 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(normalLoadS, normalLoad, 'LineWidth', 1.2);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Normal load [N]');
title('Normal loads (longitudinal + lateral transfer)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(atDistance(out.tireMu), logData(out.tireMu), 'LineWidth', 1.2);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('\mu [-]');
title('Tire friction (surface grip and load sensitivity)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(gripUseS, gripUse, 'LineWidth', 1.2);
yline(1, '--k', 'Friction limit');
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
ylim([0, 1.1]);
xlabel('Distance [m]');
ylabel('Friction used [-]');
title('Friction-ellipse use (1 = at the limit)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(atDistance(out.tireForces), logData(out.tireForces), 'LineWidth', 1.1);
hold on;
set(gca, 'ColorOrderIndex', 1);
plot(atDistance(out.lateralForces), lateralForce, '--', 'LineWidth', 1.1);
shadeEvents(gca, patchBand, cornerBand);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Force [N]');
title('Tire forces: longitudinal (solid), lateral (dashed)');
legend(wheelLabels, 'Location', 'best');
loadsTitle = sgtitle({'Loads and grip', scenarioLabel, bandLabel});
loadsTitle.Color = 'k';
styleSimulationFigure(loadsFigure);

%% Controller contributions
controllerFigure = figure('Name', ['CP27E Track Controller' scenarioName], 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for wheelIndex = 1:4
    nexttile;
    plot(atDistance(out.feedforwardTorques), ...
        out.feedforwardTorques.Data(:, wheelIndex), 'LineWidth', 1.4);
    hold on;
    plot(atDistance(out.pidCorrections), ...
        out.pidCorrections.Data(:, wheelIndex), '--', 'LineWidth', 1.2);
    plot(atDistance(out.torqueRequests), ...
        out.torqueRequests.Data(:, wheelIndex), 'k', 'LineWidth', 1.3);
    plot(atDistance(out.motorTorques), ...
        out.motorTorques.Data(:, wheelIndex), ':', 'LineWidth', 1.5);
    yline(Max_Motor_Torque, ':k', 'Stall limit', 'HandleVisibility', 'off');
    shadeEvents(gca, patchBand, cornerBand);
    grid on;
    xlim([0, xMax]);
    xlabel('Distance [m]');
    ylabel('Motor torque [N m]');
    title([wheelLabels{wheelIndex}, ' controller']);
    legend({'Feedforward', 'PID', 'TC request', 'Delivered'}, 'Location', 'best');
end
controllerTitle = sgtitle({'Feedforward plus PID at full throttle', ...
    scenarioLabel, bandLabel});
controllerTitle.Color = 'k';
styleSimulationFigure(controllerFigure);

%% Car on track: top-down animation
% Chase view of the car: wheels colored by tire slip relative to target,
% arrows for each tire's force vector, and faint outlines of where the car
% has been every 0.25 s. The course map marks every second, and the slip
% history has a time cursor. Drag the slider or press Play.
% The car points along the line (body sideslip is not modeled) and the
% front wheels show the kinematic steer angle atan(W*kappa).
roadHalfWidth = 2.5;                        % Road drawn 5 m wide [m]
frameStep = 0.02;                           % Animation frame spacing [s]
ghostStep = 0.25;                           % Outline spacing behind the car [s]
chaseHalfWidth = 8;                         % Chase view half-width [m]
forceScale = 1e-3;                          % Force arrows: 1 m per kN
slipColorMax = 2.5;                         % Top of the slip color scale [x target]
slipColorMap = interp1([0, 1, 1.5, slipColorMax], ...
    [0.60 0.60 0.60; 0.15 0.70 0.25; 1.00 0.75 0.00; 0.85 0.10 0.10], ...
    linspace(0, slipColorMax, 256));        % Gray (no slip) to green (target) to red

% Road and low-grip patch outlines from the line's left normal.
leftNormalX = -sin(heading);
leftNormalY = cos(heading);
roadX = [trackX + roadHalfWidth*leftNormalX; flipud(trackX - roadHalfWidth*leftNormalX)];
roadY = [trackY + roadHalfWidth*leftNormalY; flipud(trackY - roadHalfWidth*leftNormalY)];
patchX = [];
patchY = [];
onPatchGrid = sGrid >= Low_Grip_Start & sGrid <= Low_Grip_End;
if Low_Grip_Fact ~= Grip_Fact && any(onPatchGrid)
    patchLeft = Low_Grip_Wheels(1)*roadHalfWidth;     % Left half covered
    patchRight = -Low_Grip_Wheels(2)*roadHalfWidth;   % Right half covered
    patchX = [trackX(onPatchGrid) + patchLeft*leftNormalX(onPatchGrid); ...
        flipud(trackX(onPatchGrid) + patchRight*leftNormalX(onPatchGrid))];
    patchY = [trackY(onPatchGrid) + patchLeft*leftNormalY(onPatchGrid); ...
        flipud(trackY(onPatchGrid) + patchRight*leftNormalY(onPatchGrid))];
end

% Car pose and tire states at each animation frame.
frameTime = (0:frameStep:finishTime)';
frameS = min(interp1(distanceTime, distanceData, frameTime), Course_Length);
frameOffset = interp1(out.lateralOffset.Time(:), lateralOffset, frameTime);
frameX = interp1(sGrid, trackX, frameS) + frameOffset.*interp1(sGrid, outwardX, frameS);
frameY = interp1(sGrid, trackY, frameS) + frameOffset.*interp1(sGrid, outwardY, frameS);
frameHeading = interp1(sGrid, heading, frameS);
frameSteer = atan(W*interp1(Track_Breakpoints, Track_Curvature, frameS));
frameSlip = interp1(out.trueSlips.Time(:), trueSlip, frameTime);
frameFx = interp1(out.tireForces.Time(:), logData(out.tireForces), frameTime);
frameFy = interp1(out.lateralForces.Time(:), lateralForce, frameTime);
frameSpeed = interp1(out.vehicleSpeed.Time(:), out.vehicleSpeed.Data(:), frameTime);
frameAy = interp1(out.lateralAccel.Time(:), ayActual, frameTime);

% Car outline in the body frame (x forward, y left), origin at the CG.
carBody = [-cg_r - 0.35, -0.30; cg_f + 0.70, -0.10; cg_f + 0.70, 0.10; ...
    -cg_r - 0.35, 0.30];                    % Tapered open-wheel body [m]
wheelBox = [-r, -0.10; r, -0.10; r, 0.10; -r, 0.10];  % 16 in tire, 0.2 m wide
wheelCenters = [Wheel_Longitudinal_Position, Wheel_Lateral_Position];

% Outlines of the car every ghostStep. Each frame shows the ones already
% passed: ghostEnd(g) is the last point of outline g in ghostX/ghostY.
ghostFrames = 1:round(ghostStep/frameStep):numel(frameTime);
ghostX = [];
ghostY = [];
ghostEnd = zeros(size(ghostFrames));
for ghostIndex = 1:numel(ghostFrames)
    frame = ghostFrames(ghostIndex);
    [ghostBody, ghostWheels] = carPolygons([frameX(frame), frameY(frame), ...
        frameHeading(frame)], frameSteer(frame), carBody, wheelBox, wheelCenters);
    for shape = [{ghostBody}, ghostWheels]
        ghostX = [ghostX; shape{1}([1:end, 1], 1); NaN]; %#ok<AGROW>
        ghostY = [ghostY; shape{1}([1:end, 1], 2); NaN]; %#ok<AGROW>
    end
    ghostEnd(ghostIndex) = numel(ghostX);
end
clear frame

animation = struct();
animation.wheelPatches = gobjects(1, 4);
animation.forceLines = gobjects(1, 4);
animationFigure = figure('Name', ['CP27E Track Car Animation' scenarioName], ...
    'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.05 0.05 0.9 0.88]);
animationLayout = tiledlayout(animationFigure, 3, 3, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
animationLayout.Units = 'normalized';
animationLayout.OuterPosition = [0, 0.07, 1, 0.93];  % Room for the controls

% Chase view.
chaseAxes = nexttile(animationLayout, 1, [2, 2]);
patch(chaseAxes, roadX, roadY, [0.86 0.86 0.86], 'EdgeColor', [0.45 0.45 0.45]);
hold(chaseAxes, 'on');
if ~isempty(patchX)
    patch(chaseAxes, patchX, patchY, [0.35 0.60 1.00], 'FaceAlpha', 0.5, ...
        'EdgeColor', 'none');
end
plot(chaseAxes, trackX, trackY, '--', 'Color', [1 1 1], 'LineWidth', 1.2);
animation.ghosts = plot(chaseAxes, NaN, NaN, 'Color', [0.45 0.45 0.45 0.5]);
animation.trail = plot(chaseAxes, NaN, NaN, 'r-', 'LineWidth', 1.2);
animation.bodyPatch = patch(chaseAxes, NaN, NaN, [0.15 0.25 0.45], 'EdgeColor', 'k');
for wheelIndex = 1:4
    animation.wheelPatches(wheelIndex) = patch(chaseAxes, NaN, NaN, 'k', 'EdgeColor', 'k');
    animation.forceLines(wheelIndex) = plot(chaseAxes, NaN, NaN, 'k-', 'LineWidth', 2);
end
axis(chaseAxes, 'equal');
grid(chaseAxes, 'on');
colormap(chaseAxes, slipColorMap);
clim(chaseAxes, [0, slipColorMax]);
slipColorbar = colorbar(chaseAxes);
slipColorbar.Label.String = 'Wheel color: tire slip / target';
slipColorbar.Ticks = [0, 1, 1.5, slipColorMax];
slipColorbar.TickLabels = {'0', 'target', '1.5x', sprintf('%.1fx+', slipColorMax)};
slipColorbar.Color = 'k';
slipColorbar.Label.Color = 'k';
animation.label = text(chaseAxes, 0.02, 0.98, '', 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'FontName', 'FixedWidth', ...
    'BackgroundColor', 'w', 'Margin', 4);
xlabel(chaseAxes, 'x [m]');
ylabel(chaseAxes, 'y [m]');
title(chaseAxes, 'Chase view (arrows: tire force, 1 m = 1 kN; outlines every 0.25 s)');

% Course map with one-second marks.
courseAxes = nexttile(animationLayout, 3, [2, 1]);
patch(courseAxes, roadX, roadY, [0.90 0.90 0.90], 'EdgeColor', 'none');
hold(courseAxes, 'on');
if ~isempty(patchX)
    patch(courseAxes, patchX, patchY, [0.35 0.60 1.00], 'EdgeColor', 'none');
end
plot(courseAxes, trackX, trackY, 'k', 'LineWidth', 0.8);
plot(courseAxes, frameX, frameY, 'r--', 'LineWidth', 1.0);
markIndex = round((1:floor(finishTime))/frameStep) + 1;
plot(courseAxes, frameX(markIndex), frameY(markIndex), 'k.', 'MarkerSize', 12);
text(courseAxes, frameX(markIndex) + 2, frameY(markIndex), ...
    compose('%d s', 1:numel(markIndex)), 'FontSize', 8);
animation.marker = plot(courseAxes, NaN, NaN, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
axis(courseAxes, 'equal');
grid(courseAxes, 'on');
xlabel(courseAxes, 'x [m]');
ylabel(courseAxes, 'y [m]');
title(courseAxes, 'Course (dots every 1 s)');

% Slip history with a time cursor.
slipAxes = nexttile(animationLayout, 7, [1, 3]);
[uniqueDistance, firstIndex] = unique(distanceData, 'first');
timeAtDistance = @(d) interp1(uniqueDistance, distanceTime(firstIndex), ...
    min(max(d, 0), Course_Length));
if ~isempty(patchBand)
    xregion(slipAxes, timeAtDistance(patchBand(1)), timeAtDistance(patchBand(2)), ...
        'FaceColor', [0.2 0.45 0.95], 'FaceAlpha', 0.12, 'HandleVisibility', 'off');
end
xregion(slipAxes, timeAtDistance(cornerBand(1)), timeAtDistance(cornerBand(2)), ...
    'FaceColor', [0.95 0.55 0.1], 'FaceAlpha', 0.10, 'HandleVisibility', 'off');
hold(slipAxes, 'on');
plot(slipAxes, out.trueSlips.Time, trueSlip, 'LineWidth', 1.1);
set(slipAxes, 'ColorOrderIndex', 1);
plot(slipAxes, [0, finishTime], [slipTargets; slipTargets], ':', ...
    'LineWidth', 1.0, 'HandleVisibility', 'off');
animation.cursor = xline(slipAxes, 0, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
grid(slipAxes, 'on');
xlim(slipAxes, [0, finishTime]);
ylim(slipAxes, [-0.05, max(0.3, min(1.0, 1.1*max(trueSlip(:))))]);
xlabel(slipAxes, 'Time [s]');
ylabel(slipAxes, 'Tire slip [-]');
title(slipAxes, 'Tire slip (dotted: targets; blue band: patch, orange band: corner)');
legend(slipAxes, wheelLabels, 'Location', 'eastoutside');

% Frame data and playback controls.
animation.time = frameTime;
animation.s = frameS;
animation.x = frameX;
animation.y = frameY;
animation.heading = frameHeading;
animation.steer = frameSteer;
animation.slip = frameSlip;
animation.fx = frameFx;
animation.fy = frameFy;
animation.speed = frameSpeed;
animation.ay = frameAy;
animation.gravity = gravity;
animation.targets = slipTargets;
animation.carBody = carBody;
animation.wheelBox = wheelBox;
animation.wheelCenters = wheelCenters;
animation.forceScale = forceScale;
animation.ghostX = ghostX;
animation.ghostY = ghostY;
animation.ghostFrames = ghostFrames;
animation.ghostEnd = ghostEnd;
animation.colorMap = slipColorMap;
animation.colorMax = slipColorMax;
animation.halfWidth = chaseHalfWidth;
animation.chaseAxes = chaseAxes;
animation.playing = false;
animation.frame = 1;
animation.playButton = uicontrol(animationFigure, 'Style', 'pushbutton', ...
    'String', 'Play', 'Units', 'normalized', 'Position', [0.02, 0.015, 0.07, 0.04], ...
    'Callback', @playCarAnimation);
animation.speedMenu = uicontrol(animationFigure, 'Style', 'popupmenu', ...
    'String', {'Real time', '1/2 speed', '1/4 speed', '1/10 speed'}, 'Value', 3, ...
    'Units', 'normalized', 'Position', [0.10, 0.015, 0.08, 0.04]);
animation.slider = uicontrol(animationFigure, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.20, 0.02, 0.77, 0.03], ...
    'Min', 0, 'Max', frameTime(end), 'Value', 0, ...
    'SliderStep', [min(1, frameStep/frameTime(end)), min(1, 0.25/frameTime(end))]);
animationFigure.UserData = animation;
animation.slider.Callback = @(source, ~) drawCarFrame(ancestor(source, 'figure'), ...
    round(source.Value/frameStep) + 1);
addlistener(animation.slider, 'ContinuousValueChange', ...
    @(source, ~) drawCarFrame(ancestor(source, 'figure'), round(source.Value/frameStep) + 1));
animationTitle = title(animationLayout, {'Car on track', scenarioLabel});
animationTitle.Color = 'k';
styleSimulationFigure(animationFigure);
drawCarFrame(animationFigure, 1);
clear animation

function [body, wheels] = carPolygons(pose, steer, carBody, wheelBox, wheelCenters)
% Car body and wheel outlines in track coordinates for pose [x, y, heading].
% The front wheels (FL, FR) are turned by the steer angle.
rotation = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
body = (rotation*carBody.').' + pose(1:2);
wheels = cell(1, 4);
for wheel = 1:4
    wheelSteer = steer*(wheel <= 2);
    steerRotation = [cos(wheelSteer), -sin(wheelSteer); sin(wheelSteer), cos(wheelSteer)];
    local = (steerRotation*wheelBox.').' + wheelCenters(wheel, :);
    wheels{wheel} = (rotation*local.').' + pose(1:2);
end
end

function drawCarFrame(figureHandle, frame)
% Draw animation frame number "frame" in the car animation figure.
a = figureHandle.UserData;
frame = min(max(round(frame), 1), numel(a.time));
pose = [a.x(frame), a.y(frame), a.heading(frame)];
[body, wheels] = carPolygons(pose, a.steer(frame), a.carBody, a.wheelBox, ...
    a.wheelCenters);
set(a.bodyPatch, 'XData', body(:, 1), 'YData', body(:, 2));
rotation = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
for wheel = 1:4
    ratio = min(max(a.slip(frame, wheel)/a.targets(wheel), 0), a.colorMax);
    color = a.colorMap(1 + round(ratio/a.colorMax*(size(a.colorMap, 1) - 1)), :);
    set(a.wheelPatches(wheel), 'XData', wheels{wheel}(:, 1), ...
        'YData', wheels{wheel}(:, 2), 'FaceColor', color);
    center = mean(wheels{wheel}, 1);
    force = rotation*[a.fx(frame, wheel); a.fy(frame, wheel)]*a.forceScale;
    set(a.forceLines(wheel), 'XData', center(1) + [0, force(1)], ...
        'YData', center(2) + [0, force(2)]);
end
set(a.trail, 'XData', a.x(1:frame), 'YData', a.y(1:frame));
passed = nnz(a.ghostFrames < frame);
if passed > 0
    set(a.ghosts, 'XData', a.ghostX(1:a.ghostEnd(passed)), ...
        'YData', a.ghostY(1:a.ghostEnd(passed)));
else
    set(a.ghosts, 'XData', NaN, 'YData', NaN);
end
set(a.marker, 'XData', a.x(frame), 'YData', a.y(frame));
a.cursor.Value = a.time(frame);
xlim(a.chaseAxes, a.x(frame) + a.halfWidth*[-1, 1]);
ylim(a.chaseAxes, a.y(frame) + a.halfWidth*[-1, 1]);
a.label.String = sprintf(['t = %.2f s   s = %.1f m   v = %.1f m/s   ' ...
    'a_y = %.2f g\nslip  FL %.3f  FR %.3f  RL %.3f  RR %.3f'], a.time(frame), ...
    a.s(frame), a.speed(frame), a.ay(frame)/a.gravity, a.slip(frame, :));
a.slider.Value = a.time(frame);
a.frame = frame;
figureHandle.UserData = a;
end

function playCarAnimation(button, ~)
% Play or pause the car animation at the selected fraction of real time.
% Moving the slider while playing continues from the new position.
figureHandle = ancestor(button, 'figure');
a = figureHandle.UserData;
if a.playing
    a.playing = false;
    figureHandle.UserData = a;
    return
end
a.playing = true;
figureHandle.UserData = a;
button.String = 'Pause';
rates = [1, 0.5, 0.25, 0.1];
frame = a.frame;
if frame >= numel(a.time)
    frame = 1;
end
rate = rates(a.speedMenu.Value);
clockStart = tic;
timeStart = a.time(frame);
lastDrawn = frame;
while isvalid(figureHandle) && figureHandle.UserData.playing && frame < numel(a.time)
    if figureHandle.UserData.frame ~= lastDrawn || rates(a.speedMenu.Value) ~= rate
        frame = figureHandle.UserData.frame;   % Slider moved or speed changed
        rate = rates(a.speedMenu.Value);
        clockStart = tic;
        timeStart = a.time(frame);
    end
    due = find(a.time <= timeStart + toc(clockStart)*rate, 1, 'last');
    frame = min(max(frame + 1, due), numel(a.time));   % Skip frames to keep pace
    drawCarFrame(figureHandle, frame);
    lastDrawn = frame;
    pause(max((a.time(frame) - timeStart)/rate - toc(clockStart), 0.001));
end
if isvalid(figureHandle)
    a = figureHandle.UserData;
    a.playing = false;
    figureHandle.UserData = a;
    button.String = 'Play';
end
end

function data = logData(ts)
% Logged values as a samples-by-signals matrix, whatever the logged shape.
data = squeeze(ts.Data);
if size(data, 1) ~= numel(ts.Time)
    data = data.';
end
end

function shadeEvents(ax, patchBand, cornerBand)
% Shade the low-grip patch (blue) and the corner (orange) on a distance axis.
if ~isempty(patchBand)
    xregion(ax, patchBand(1), patchBand(2), 'FaceColor', [0.2 0.45 0.95], ...
        'FaceAlpha', 0.12, 'HandleVisibility', 'off');
end
xregion(ax, cornerBand(1), cornerBand(2), 'FaceColor', [0.95 0.55 0.1], ...
    'FaceAlpha', 0.10, 'HandleVisibility', 'off');
end

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
