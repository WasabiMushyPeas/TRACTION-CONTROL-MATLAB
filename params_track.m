%% CP27E MIS lap traction-control simulation
% Same car and the same traction controller as params.m / TC_organized,
% driven round one lap of the MIS track (GPS-derived, MIS_Simulink_Track):
%   - a driver block floors it whenever the car is at or below a target
%     speed for each corner, and lifts and brakes above it, so every corner
%     exit is a full-throttle traction test,
%   - lateral load transfer and a combined-slip tire, so the tires share
%     their grip between cornering and traction,
%   - optional low-grip sections (Low_Grip_Sections) anywhere on the lap.
% The controller blocks are unchanged, so this tests the controller that
% the C++ port implements. The controller still assumes the dry grip
% factor and a single vehicle speed for all four wheels; the plant has
% per-wheel road grip and per-wheel ground speeds.
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

%% MIS track
% MIS_Simulink_Track/mis_track.mat is the smoothed GPS path of one MIS lap:
% distance s, x east, y north, heading (radians from east), and signed
% curvature (positive = left turn). The car does one lap from a standing
% start. Road width, banking, and grade are not in the data; the road
% drawn in the animation is a nominal 5 m.
% The GPS lap begins in a tight hairpin, so the lap is re-started at
% Track_Start_Distance along the GPS lap. 100 m is the start of the
% longest straight (GPS 96-173 m). All distances below (low-grip sections,
% plots) are measured from this start.
Track_Start_Distance = 100;                 % Start line, GPS lap distance [m]
misTrack = load(fullfile(fileparts(mfilename('fullpath')), ...
    'MIS_Simulink_Track', 'mis_track.mat'), 'track');
misTrack = misTrack.track;
Track_Length = misTrack.length_m;                          % One lap [m]
Track_Breakpoints = double(misTrack.s_m(:));               % [m]
gpsDistance = mod(Track_Breakpoints + Track_Start_Distance, Track_Length);
atGpsDistance = @(field) interp1(Track_Breakpoints, double(field(:)), gpsDistance);
Track_Curvature = atGpsDistance(misTrack.curvature_1pm);   % [1/m]
Track_X = atGpsDistance(misTrack.x_m);                     % East [m]
Track_Y = atGpsDistance(misTrack.y_m);                     % North [m]
Track_Heading = unwrap(atan2(atGpsDistance(misTrack.heading_sin), ...
    atGpsDistance(misTrack.heading_cos)));                 % Unwrapped [rad]
Track_Recorded_Speed = atGpsDistance(misTrack.speed_recorded_mps); % GPS, comparison only [m/s]
Track_Recorded_Lap_Time = misTrack.time_recorded_s(end) - misTrack.time_recorded_s(1); % [s]
clear misTrack gpsDistance atGpsDistance
Course_Length = Track_Length;               % The simulation stops after one lap [m]

%% Low-grip sections
% One row per section: {start [m], length [m], grip factor [-], sides}.
% The grip factor scales tire friction like Grip_Fact (dry = 0.60): about
% 0.45 is a damp line, 0.30 standing water, 0.06 ice. Sides is "both",
% "left", or "right" (split mu). Each wheel reads the surface at its own
% position. The controller is not told; it keeps assuming Grip_Fact.
% Use Low_Grip_Sections = {} for a dry lap.
Low_Grip_Sections = {
    220, 15, 0.30, "both"                   % Standing water on a corner exit
    530, 10, 0.45, "left"                   % Damp left side on a corner exit
    };
Surface_Transition_Length = 0.2;            % Grip changes over about one contact patch [m]

lowGripCount = size(Low_Grip_Sections, 1);
Low_Grip_Start = zeros(lowGripCount, 1);    % [m]
Low_Grip_End = zeros(lowGripCount, 1);      % [m]
Low_Grip_Fact = zeros(lowGripCount, 1);     % [-]
Low_Grip_Wheels = false(lowGripCount, 4);   % Wheels each section affects (FL FR RL RR)
surfaceBreaks = [-10, Course_Length + 50];
for section = 1:lowGripCount
    [Low_Grip_Start(section), sectionLength, Low_Grip_Fact(section), sides] = ...
        Low_Grip_Sections{section, :};
    assert(any(sides == ["both", "left", "right"]), 'CP27E:LowGripSides', ...
        'Low-grip section %d: sides must be "both", "left", or "right".', section);
    Low_Grip_End(section) = Low_Grip_Start(section) + sectionLength;
    Low_Grip_Wheels(section, :) = [sides ~= "right", sides ~= "left"]*[1 0 1 0; 0 1 0 1] > 0;
    surfaceBreaks = [surfaceBreaks, Low_Grip_Start(section) + [-Surface_Transition_Length, 0], ...
        Low_Grip_End(section) + [0, Surface_Transition_Length]]; %#ok<AGROW>
end
% Surface_Grip_Map: rows are the left and right sides of the road, columns
% Surface_Breakpoints; grip ramps over Surface_Transition_Length at each edge.
Surface_Breakpoints = unique(surfaceBreaks);
Surface_Grip_Table = Grip_Fact*ones(2, numel(Surface_Breakpoints));
for section = 1:lowGripCount
    weight = min(max(min(Surface_Breakpoints - (Low_Grip_Start(section) - ...
        Surface_Transition_Length), Low_Grip_End(section) + ...
        Surface_Transition_Length - Surface_Breakpoints)/Surface_Transition_Length, 0), 1);
    sectionGrip = Grip_Fact + (Low_Grip_Fact(section) - Grip_Fact)*weight;
    for side = find(Low_Grip_Wheels(section, 1:2))
        Surface_Grip_Table(side, :) = min(Surface_Grip_Table(side, :), sectionGrip);
    end
end
clear surfaceBreaks section sectionLength sides weight sectionGrip side

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

%% Driver
% The Driver block follows a target speed: flat out whenever the car is at
% or below it, lifting over Driver_Lift_Band above it, and braking beyond
% that. The target is the steady cornering speed on dry road using
% Driver_Grip_Use of the tire friction (with downforce), cut back so the
% car can brake into every corner at Driver_Brake_Decel, and kept below
% the motor speed taper. The driver plans for dry road everywhere.
% Brakes act on the car body: they get the car round the lap, and the TC
% is only tested on throttle. The driver also steers back to the line
% after running wide, once the tires have grip to spare.
Driver_Grip_Use = 0.85;                     % Fraction of dry grip planned for corners [-]
Driver_Brake_Decel = 1.2*gravity;           % Planned braking [m/s^2]
Driver_Brake_Max_Decel = 1.5*gravity;       % Full brake pedal [m/s^2]
Driver_Preview_Time = 0.3;                  % Looks this far ahead at the target [s]
Driver_Lift_Band = 0.5;                     % Throttle fades to zero this far over target [m/s]
Driver_Brake_Band = 1.0;                    % Brake builds to full over this much more [m/s]
Driver_Line_Recovery_Hz = 0.3;              % Steering back to the line [Hz]

muPlan = Driver_Grip_Use*tireMuAtLoad(Mv*gravity/4);
cornerDenominator = Mv*abs(Track_Curvature) - muPlan*0.5*rho*CLA;
speedCap = Max_Wheel_Omega*r*(1 - Speed_Limit_Taper_RPM/Max_Motor_RPM); % Taper start [m/s]
Driver_Speed_Target = speedCap*ones(size(Track_Curvature));            % [m/s]
canLimit = cornerDenominator > 0;
Driver_Speed_Target(canLimit) = min(speedCap, ...
    sqrt(muPlan*Mv*gravity./cornerDenominator(canLimit)));
for point = numel(Driver_Speed_Target) - 1:-1:1
    Driver_Speed_Target(point) = min(Driver_Speed_Target(point), ...
        sqrt(Driver_Speed_Target(point + 1)^2 + 2*Driver_Brake_Decel* ...
        (Track_Breakpoints(point + 1) - Track_Breakpoints(point))));
end
clear muPlan cornerDenominator speedCap canLimit point

%% Simulation
timeMax = 150;                              % Simulation timeout [s]
timedomain = timeMax;                       % Compatibility alias [s]

% Stop here when the model's InitFcn only needs the parameters.
if exist('TC_paramsOnly', 'var') && TC_paramsOnly
    return
end

%% Run TC_track: one MIS lap
modelName = 'TC_track';
in = Simulink.SimulationInput(modelName);
in = in.setModelParameter('StopTime', num2str(timeMax));
out = sim(in);
simout = out;  % Compatibility alias for interactive workspace use.

%% Distance axis and lap time
distanceTime = out.distance.Time(:);
distanceData = out.distance.Data(:);
if isempty(distanceData) || any(~isfinite(distanceData))
    error('CP27E:InvalidDistance', ...
        'The simulation did not return a finite distance history.');
end
if distanceData(end) < Course_Length - 1e-3
    error('CP27E:CourseNotFinished', ...
        ['The simulation stopped at %.1f m after %.1f s without finishing ' ...
         'the %.0f m lap. Increase timeMax or inspect the model.'], ...
        distanceData(end), distanceTime(end), Course_Length);
end
finishIndex = find(distanceData >= Course_Length, 1, 'first');
finishTime = interp1(distanceData(finishIndex-1:finishIndex), ...
    distanceTime(finishIndex-1:finishIndex), Course_Length);

% Distance travelled at each logged sample (logs run at 10 kHz or 500 Hz).
atDistance = @(ts) interp1(distanceTime, distanceData, ts.Time(:), ...
    'linear', 'extrap');
[uniqueDistance, firstIndex] = unique(distanceData, 'first');
timeAtDistance = @(d) interp1(uniqueDistance, distanceTime(firstIndex), ...
    min(max(d, 0), Course_Length));

wheelLabels = {'FL', 'FR', 'RL', 'RR'};
slipTargets = [Slip_Target_FL, Slip_Target_FR, ...
    Slip_Target_RL, Slip_Target_RR];

slipTime = out.trueSlips.Time(:);
trueSlip = logData(out.trueSlips);           % Tire slip (own ground speed)
trueSlipS = atDistance(out.trueSlips);
measuredSlip = logData(out.wheelSlips);      % Controller slip (CG speed)
measuredSlipS = atDistance(out.wheelSlips);
measuredSlipAtTrue = interp1(out.wheelSlips.Time(:), measuredSlip, slipTime, ...
    'previous', 'extrap');
plantSpeedAtSlip = interp1(out.vehicleSpeed.Time(:), out.vehicleSpeed.Data(:), slipTime);
wheelPosition = trueSlipS + Wheel_Longitudinal_Position.'; % Each wheel's s [m]

ayDemand = logData(out.lateralAccelDemand);
ayActual = logData(out.lateralAccel);
lateralS = atDistance(out.lateralAccel);
ayAtSlip = interp1(out.lateralAccel.Time(:), ayActual, slipTime);
lateralOffset = logData(out.lateralOffset);  % Left of the line [m]
lateralOffsetS = atDistance(out.lateralOffset);
gripUse = logData(out.gripUse);
gripUseS = atDistance(out.gripUse);
lateralForce = logData(out.lateralForces);
normalLoad = logData(out.normalLoads);
normalLoadS = atDistance(out.normalLoads);
driverLog = logData(out.driver);             % [throttle N*m, brake N, target m/s]
driverS = atDistance(out.driver);
throttleAtSlip = interp1(out.driver.Time(:), driverLog(:, 1), slipTime, 'previous');

%% Corner exits: floored while still cornering
% Inside/outside wheels follow the direction of each turn.
exitMask = throttleAtSlip >= 0.99*Max_Motor_Torque & ...
    abs(ayAtSlip) >= 0.5*gravity & plantSpeedAtSlip >= 2;
turningLeft = ayAtSlip > 0;
insideSlip = [turningLeft.*trueSlip(:, 1) + ~turningLeft.*trueSlip(:, 2), ...
    turningLeft.*trueSlip(:, 3) + ~turningLeft.*trueSlip(:, 4)];   % [front, rear]
outsideSlip = [~turningLeft.*trueSlip(:, 1) + turningLeft.*trueSlip(:, 2), ...
    ~turningLeft.*trueSlip(:, 3) + turningLeft.*trueSlip(:, 4)];
slipBias = measuredSlipAtTrue - trueSlip;    % Controller minus tire slip
insideBias = mean([turningLeft.*slipBias(:, 1) + ~turningLeft.*slipBias(:, 2); ...
    turningLeft.*slipBias(:, 3) + ~turningLeft.*slipBias(:, 4)], 'omitnan');
exitTime = nnz(exitMask)*mean(diff(slipTime));
if any(exitMask)
    exitInsidePeak = max(insideSlip(exitMask, :), [], 1);
    exitOutsidePeak = max(outsideSlip(exitMask, :), [], 1);
    exitInsideMean = mean(insideSlip(exitMask, :), 1);
    exitOutsideMean = mean(outsideSlip(exitMask, :), 1);
    insideBias = mean([turningLeft(exitMask).*slipBias(exitMask, 1) + ...
        ~turningLeft(exitMask).*slipBias(exitMask, 2); ...
        turningLeft(exitMask).*slipBias(exitMask, 3) + ...
        ~turningLeft(exitMask).*slipBias(exitMask, 4)]);
end

% PID state each time the driver goes back to full throttle: while lifted,
% tracking anti-windup drives the PID toward minus the feedforward.
floored = driverLog(:, 1) >= 0.99*Max_Motor_Torque;
reapplyIndex = find(diff(floored) == 1) + 1;
reapplyPid = interp1(out.pidCorrections.Time(:), out.pidCorrections.Data, ...
    out.driver.Time(reapplyIndex), 'previous', 'extrap');

%% Low-grip section metrics, per section and wheel
% On the section: the wheel's own position is on it, or within one
% wheelbase after it (the wheel is still spinning down).
sectionPeakSlip = nan(lowGripCount, 4);
sectionOverTime = nan(lowGripCount, 4);     % Time above 1.5x target slip [s]
sectionThrottle = nan(lowGripCount, 1);
slipSampleTime = mean(diff(slipTime));
for section = 1:lowGripCount
    onSection = trueSlipS >= Low_Grip_Start(section) - cg_f & ...
        trueSlipS <= Low_Grip_End(section) + cg_r;
    sectionThrottle(section) = mean(throttleAtSlip(onSection))/Max_Motor_Torque;
    for wheelIndex = find(Low_Grip_Wheels(section, :))
        onPatch = wheelPosition(:, wheelIndex) >= Low_Grip_Start(section) & ...
            wheelPosition(:, wheelIndex) <= Low_Grip_End(section) + W;
        if ~any(onPatch)
            continue
        end
        sectionPeakSlip(section, wheelIndex) = max(trueSlip(onPatch, wheelIndex));
        sectionOverTime(section, wheelIndex) = slipSampleTime*nnz(onPatch & ...
            trueSlip(:, wheelIndex) > 1.5*slipTargets(wheelIndex));
    end
end

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
brakeAtAccel = interp1(out.driver.Time(:), driverLog(:, 2), ...
    out.acceleration.Time(:), 'previous', 'extrap');
normalLoadResidual = max(abs(sum(out.normalLoads.Data, 2) - ...
    (Mv * gravity + out.downforce.Data)));
tireForceResidual = max(abs(out.totalTireForce.Data - ...
    sum(out.tireForces.Data, 2)));
vehicleForceResidual = max(abs(Mv * out.acceleration.Data - ...
    (out.totalTireForce.Data - out.aeroDrag.Data - brakeAtAccel)));
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
% The power limit uses the previous 2 ms tick, so a wheel spinning up or the
% TC releasing torque quickly can push the command briefly over the limit.
% That is reported as a warning instead of stopping the run.
powerOvershootPercent = 100 * max(commandPowerViolation, 0) / maxTractivePower;
if powerOvershootPercent > 1 || commandedExcessEnergyFraction > 0.01
    warning('CP27E:CommandPowerLimit', ...
        'Commanded power peaked %.1f%% over the limit (excess energy %.3f%%).', ...
        powerOvershootPercent, 100 * commandedExcessEnergyFraction);
end
% A wheel spinning up near top speed can overrun the speed taper during the
% motor torque lag, so this is reported, not an error.
if motorSpeedViolation > constraintTolerance
    warning('CP27E:MotorSpeedLimit', ...
        'A motor reached %.0f rpm, %.0f rpm over the speed limit.', ...
        max(motorRPM(:)), motorSpeedViolation);
end

%% Report
[maxOffset, maxOffsetIndex] = max(abs(lateralOffset));
fprintf('CP27E MIS lap TC simulation (unchanged TC_organized controller)\n');
fprintf('  Lap time:                   %.2f s (recorded GPS lap %.1f s)\n', ...
    finishTime, Track_Recorded_Lap_Time);
fprintf('  Top speed:                  %.1f m/s (%.0f km/h)\n', ...
    max(out.vehicleSpeed.Data), 3.6*max(out.vehicleSpeed.Data));
fprintf('  Full throttle:              %.0f%% of the lap; braking %.0f%%\n', ...
    100*mean(floored), 100*mean(driverLog(:, 2) > 0));
fprintf('  Peak lateral accel:         %.2f g\n', max(abs(ayActual))/gravity);
fprintf('  Furthest off the line:      %.2f m at %.0f m\n', maxOffset, ...
    lateralOffsetS(maxOffsetIndex));
if any(exitMask)
    fprintf('  Corner exits (floored, |a_y| >= 0.5 g, %.1f s of the lap):\n', exitTime);
    fprintf('    Inside  front/rear slip:  peak %.3f / %.3f, mean %.3f / %.3f\n', ...
        exitInsidePeak, exitInsideMean);
    fprintf('    Outside front/rear slip:  peak %.3f / %.3f, mean %.3f / %.3f\n', ...
        exitOutsidePeak, exitOutsideMean);
    fprintf('    Targets front/rear:       %.3f / %.3f\n', Slip_Target_FL, Slip_Target_RL);
    fprintf('    Controller - tire slip:   %+.3f on inside wheels (single-speed bias)\n', ...
        insideBias);
end
if ~isempty(reapplyPid)
    fprintf(['  Back on full throttle:      PID at %.1f front / %.1f rear N m ' ...
        '(mean of %d re-applications)\n'], mean(reapplyPid(:, 1:2), 'all'), ...
        mean(reapplyPid(:, 3:4), 'all'), numel(reapplyIndex));
end
for section = 1:lowGripCount
    fprintf('  Low-grip %d: %.0f-%.0f m, grip %.2f, %s, %.0f%% throttle on it\n', ...
        section, Low_Grip_Start(section), Low_Grip_End(section), ...
        Low_Grip_Fact(section), Low_Grip_Sections{section, 4}, ...
        100*sectionThrottle(section));
    for wheelIndex = find(~isnan(sectionPeakSlip(section, :)))
        fprintf('    %s: peak slip %.3f (target %.3f), %.2f s above 1.5x target\n', ...
            wheelLabels{wheelIndex}, sectionPeakSlip(section, wheelIndex), ...
            slipTargets(wheelIndex), sectionOverTime(section, wheelIndex));
    end
end
fprintf(['  Peak commanded power:       %.1f kW (limit %.1f kW, ' ...
    'excess energy %.3f%%)\n'], max(commandedElectricalPower) / 1e3, ...
    maxTractivePower / 1e3, 100 * commandedExcessEnergyFraction);
fprintf('  Peak motor speed:           %.0f rpm (limit %.0f rpm)\n', ...
    max(motorRPM(:)), Max_Motor_RPM);
fprintf(['  Math checks:                PASS (load %.1e N, tire sum %.1e N, ' ...
    'F=ma %.1e N, lateral %.1e N)\n'], normalLoadResidual, ...
    tireForceResidual, vehicleForceResidual, lateralForceResidual);

%% Figure labels shared by every figure
sectionNames = strings(lowGripCount, 1);
for section = 1:lowGripCount
    sectionNames(section) = sprintf('%.0f-%.0f m grip %.2f %s', ...
        Low_Grip_Start(section), Low_Grip_End(section), ...
        Low_Grip_Fact(section), Low_Grip_Sections{section, 4});
end
if lowGripCount == 0
    scenarioLabel = sprintf('MIS lap, dry (grip %.2f)', Grip_Fact);
else
    scenarioLabel = sprintf('MIS lap, dry grip %.2f; low grip: %s', ...
        Grip_Fact, strjoin(sectionNames, ', '));
end
bandLabel = 'Blue bands: low-grip sections under any wheel.';
scenarioName = sprintf(' - MIS, %d low-grip sections', lowGripCount);
sectionBands = [Low_Grip_Start - cg_f, Low_Grip_End + cg_r];  % CG distance [m]
xMax = Course_Length;

%% Track overview
trackFigure = figure('Name', ['CP27E Track Overview' scenarioName], 'Color', 'w');
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Car path = line + lateral offset along the line's left normal.
leftNormalX = -sin(Track_Heading);
leftNormalY = cos(Track_Heading);
[offsetDistance, movingIndex] = unique(lateralOffsetS, 'last');  % Skip standstill
offsetOnLine = interp1(offsetDistance, lateralOffset(movingIndex), ...
    Track_Breakpoints, 'linear', 'extrap');
nexttile([2, 1]);
plot(Track_X, Track_Y, 'k', 'LineWidth', 1.0);
hold on;
plot(Track_X + offsetOnLine.*leftNormalX, Track_Y + offsetOnLine.*leftNormalY, ...
    'r--', 'LineWidth', 1.2);
for section = 1:lowGripCount
    onSection = Track_Breakpoints >= Low_Grip_Start(section) & ...
        Track_Breakpoints <= Low_Grip_End(section);
    plot(Track_X(onSection), Track_Y(onSection), 'b', 'LineWidth', 6, ...
        'HandleVisibility', 'off');
end
plot(Track_X(1), Track_Y(1), 'ko', 'MarkerFaceColor', 'g');
axis equal;
grid on;
xlabel('East [m]');
ylabel('North [m]');
title('MIS track (blue: low grip)');
legend({'Racing line', 'Car path', 'Start/finish'}, 'Location', 'best');

nexttile;
plot(atDistance(out.vehicleSpeed), out.vehicleSpeed.Data, 'k', 'LineWidth', 1.5);
hold on;
plot(driverS, driverLog(:, 3), 'Color', [0.95 0.55 0.1], 'LineWidth', 1.1);
plot(Track_Breakpoints, Track_Recorded_Speed, ':', 'Color', [0.4 0.4 0.4], ...
    'LineWidth', 1.1);
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Speed [m/s]');
title(sprintf('Speed -- lap %.2f s', finishTime));
legend({'Car', 'Driver target', 'Recorded GPS (real driver)'}, 'Location', 'best');

nexttile;
plot(lateralS, ayDemand / gravity, 'k--', 'LineWidth', 1.1);
hold on;
plot(lateralS, ayActual / gravity, 'r', 'LineWidth', 1.1);
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Lateral acceleration [g]');
title('Lateral acceleration: line needs vs tires give');
legend({'Demanded by the line', 'Achieved'}, 'Location', 'best');

nexttile;
yyaxis left;
plot(driverS, 100 * driverLog(:, 1) / Max_Motor_Torque, 'LineWidth', 1.1);
ylabel('Throttle [%]');
ylim([-5, 105]);
yyaxis right;
plot(driverS, driverLog(:, 2) / (Mv * gravity), 'LineWidth', 1.1);
ylabel('Brake [g]');
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
title('Driver inputs');

nexttile;
plot(lateralOffsetS, lateralOffset, 'r', 'LineWidth', 1.2);
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
ylim([min(-0.5, 1.1 * min(lateralOffset)), max(0.5, 1.1 * max(lateralOffset))]);
xlabel('Distance [m]');
ylabel('Offset [m]');
title('Car left of the line (running wide when the tires run out)');
trackTitle = sgtitle({'MIS lap overview', scenarioLabel, bandLabel});
trackTitle.Color = 'k';
styleSimulationFigure(trackFigure);

%% Wheel slip: what the tire sees vs what the controller sees
slipFigure = figure('Name', ['CP27E Track Wheel Slip' scenarioName], 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for wheelIndex = 1:4
    nexttile;
    plot(trueSlipS, trueSlip(:, wheelIndex), 'LineWidth', 1.1);
    hold on;
    plot(measuredSlipS, measuredSlip(:, wheelIndex), '--', 'LineWidth', 0.9);
    yline(slipTargets(wheelIndex), ':k', 'Target', 'LineWidth', 1.2);
    yline(Pacejka_Slip_Peak, ':', 'Peak \mu', 'Color', [0.5 0.5 0.5]);
    shadeSections(gca, sectionBands);
    grid on;
    xlim([0, xMax]);
    ylim([-0.05, max(0.3, min(1.0, 1.1*max(trueSlip(:, wheelIndex))))]);
    xlabel('Distance [m]');
    ylabel('Slip ratio [-]');
    title(wheelLabels{wheelIndex});
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
plot(normalLoadS, normalLoad, 'LineWidth', 1.0);
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('Normal load [N]');
title('Normal loads (longitudinal + lateral transfer)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(atDistance(out.tireMu), logData(out.tireMu), 'LineWidth', 1.0);
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
xlabel('Distance [m]');
ylabel('\mu [-]');
title('Tire friction (surface grip and load sensitivity)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(gripUseS, gripUse, 'LineWidth', 1.0);
yline(1, '--k', 'Friction limit');
shadeSections(gca, sectionBands);
grid on;
xlim([0, xMax]);
ylim([0, 1.1]);
xlabel('Distance [m]');
ylabel('Friction used [-]');
title('Friction-ellipse use (1 = at the limit)');
legend(wheelLabels, 'Location', 'best');

nexttile;
plot(atDistance(out.tireForces), logData(out.tireForces), 'LineWidth', 1.0);
hold on;
set(gca, 'ColorOrderIndex', 1);
plot(atDistance(out.lateralForces), lateralForce, '--', 'LineWidth', 1.0);
shadeSections(gca, sectionBands);
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
        out.feedforwardTorques.Data(:, wheelIndex), 'LineWidth', 1.2);
    hold on;
    plot(atDistance(out.pidCorrections), ...
        out.pidCorrections.Data(:, wheelIndex), '--', 'LineWidth', 1.0);
    plot(atDistance(out.torqueRequests), ...
        out.torqueRequests.Data(:, wheelIndex), 'k', 'LineWidth', 1.1);
    plot(atDistance(out.motorTorques), ...
        out.motorTorques.Data(:, wheelIndex), ':', 'LineWidth', 1.3);
    yline(Max_Motor_Torque, ':k', 'Stall limit', 'HandleVisibility', 'off');
    shadeSections(gca, sectionBands);
    grid on;
    xlim([0, xMax]);
    xlabel('Distance [m]');
    ylabel('Motor torque [N m]');
    title([wheelLabels{wheelIndex}, ' controller']);
    legend({'Feedforward', 'PID', 'TC request', 'Delivered'}, 'Location', 'best');
end
controllerTitle = sgtitle({'Feedforward plus PID', scenarioLabel, bandLabel});
controllerTitle.Color = 'k';
styleSimulationFigure(controllerFigure);

%% Car on track: top-down animation
% Chase view of the car: wheels colored by tire slip relative to target,
% arrows for each tire's force vector, and faint outlines of where the car
% has been every 0.25 s. The course map marks every 10 s, and the slip
% history has a time cursor. Drag the slider or press Play.
% The car points along the line (body sideslip is not modeled) and the
% front wheels show the kinematic steer angle atan(W*kappa).
roadHalfWidth = 2.5;                        % Road drawn 5 m wide [m]
frameStep = 0.02;                           % Animation frame spacing [s]
ghostStep = 0.25;                           % Outline spacing behind the car [s]
chaseHalfWidth = 12;                        % Chase view half-width [m]
forceScale = 1e-3;                          % Force arrows: 1 m per kN
slipColorMax = 2.5;                         % Top of the slip color scale [x target]
slipColorMap = interp1([0, 1, 1.5, slipColorMax], ...
    [0.60 0.60 0.60; 0.15 0.70 0.25; 1.00 0.75 0.00; 0.85 0.10 0.10], ...
    linspace(0, slipColorMax, 256));        % Gray (no slip) to green (target) to red

% Road and low-grip sections as quad strips between two offsets from the line.
[roadVertices, roadFaces] = roadStrip(Track_X, Track_Y, leftNormalX, leftNormalY, ...
    true(size(Track_X)), -roadHalfWidth, roadHalfWidth);
sectionStrips = cell(lowGripCount, 2);
for section = 1:lowGripCount
    onSection = Track_Breakpoints >= Low_Grip_Start(section) & ...
        Track_Breakpoints <= Low_Grip_End(section);
    [sectionStrips{section, :}] = roadStrip(Track_X, Track_Y, leftNormalX, ...
        leftNormalY, onSection, -Low_Grip_Wheels(section, 2)*roadHalfWidth, ...
        Low_Grip_Wheels(section, 1)*roadHalfWidth);
end

% Car pose and tire states at each animation frame.
frameTime = (0:frameStep:finishTime)';
frameS = min(interp1(distanceTime, distanceData, frameTime), Course_Length);
frameOffset = interp1(out.lateralOffset.Time(:), lateralOffset, frameTime);
frameHeading = interp1(Track_Breakpoints, Track_Heading, frameS);
frameX = interp1(Track_Breakpoints, Track_X, frameS) - frameOffset.*sin(frameHeading);
frameY = interp1(Track_Breakpoints, Track_Y, frameS) + frameOffset.*cos(frameHeading);
frameSteer = atan(W*interp1(Track_Breakpoints, Track_Curvature, frameS));
frameSlip = interp1(slipTime, trueSlip, frameTime);
frameFx = interp1(out.tireForces.Time(:), logData(out.tireForces), frameTime);
frameFy = interp1(out.lateralForces.Time(:), lateralForce, frameTime);
frameSpeed = interp1(out.vehicleSpeed.Time(:), out.vehicleSpeed.Data(:), frameTime);
frameAy = interp1(out.lateralAccel.Time(:), ayActual, frameTime);
frameDriver = interp1(out.driver.Time(:), driverLog(:, 1:2), frameTime, 'previous', 'extrap');

% Car outline in the body frame (x forward, y left), origin at the CG.
carBody = [-cg_r - 0.35, -0.30; cg_f + 0.70, -0.10; cg_f + 0.70, 0.10; ...
    -cg_r - 0.35, 0.30];                    % Tapered open-wheel body [m]
wheelBox = [-r, -0.10; r, -0.10; r, 0.10; -r, 0.10];  % 16 in tire, 0.2 m wide
wheelCenters = [Wheel_Longitudinal_Position, Wheel_Lateral_Position];

% Outlines of the car every ghostStep. Each frame shows the ones already
% passed: ghostEnd(g) is the last point of outline g in ghostX/ghostY.
ghostFrames = 1:round(ghostStep/frameStep):numel(frameTime);
ghostX = cell(numel(ghostFrames), 1);
ghostY = cell(numel(ghostFrames), 1);
for ghostIndex = 1:numel(ghostFrames)
    frame = ghostFrames(ghostIndex);
    [ghostBody, ghostWheels] = carPolygons([frameX(frame), frameY(frame), ...
        frameHeading(frame)], frameSteer(frame), carBody, wheelBox, wheelCenters);
    shapes = [{ghostBody}, ghostWheels];
    ghostX{ghostIndex} = cell2mat(cellfun(@(p) [p([1:end, 1], 1); NaN], shapes(:), ...
        'UniformOutput', false));
    ghostY{ghostIndex} = cell2mat(cellfun(@(p) [p([1:end, 1], 2); NaN], shapes(:), ...
        'UniformOutput', false));
end
ghostEnd = cumsum(cellfun(@numel, ghostX));
ghostX = vertcat(ghostX{:});
ghostY = vertcat(ghostY{:});
clear frame shapes

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
patch(chaseAxes, 'Vertices', roadVertices, 'Faces', roadFaces, ...
    'FaceColor', [0.86 0.86 0.86], 'EdgeColor', 'none');
hold(chaseAxes, 'on');
for section = 1:lowGripCount
    patch(chaseAxes, 'Vertices', sectionStrips{section, 1}, ...
        'Faces', sectionStrips{section, 2}, 'FaceColor', [0.35 0.60 1.00], ...
        'FaceAlpha', 0.5, 'EdgeColor', 'none');
end
plot(chaseAxes, Track_X + roadHalfWidth*leftNormalX, Track_Y + roadHalfWidth*leftNormalY, ...
    'Color', [0.45 0.45 0.45]);
plot(chaseAxes, Track_X - roadHalfWidth*leftNormalX, Track_Y - roadHalfWidth*leftNormalY, ...
    'Color', [0.45 0.45 0.45]);
plot(chaseAxes, Track_X, Track_Y, '--', 'Color', [1 1 1], 'LineWidth', 1.2);
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
xlabel(chaseAxes, 'East [m]');
ylabel(chaseAxes, 'North [m]');
title(chaseAxes, 'Chase view (arrows: tire force, 1 m = 1 kN; outlines every 0.25 s)');

% Whole lap with 10 s marks.
courseAxes = nexttile(animationLayout, 3, [2, 1]);
patch(courseAxes, 'Vertices', roadVertices, 'Faces', roadFaces, ...
    'FaceColor', [0.90 0.90 0.90], 'EdgeColor', 'none');
hold(courseAxes, 'on');
plot(courseAxes, Track_X, Track_Y, 'k', 'LineWidth', 0.8);
for section = 1:lowGripCount
    onSection = Track_Breakpoints >= Low_Grip_Start(section) & ...
        Track_Breakpoints <= Low_Grip_End(section);
    plot(courseAxes, Track_X(onSection), Track_Y(onSection), ...
        'Color', [0.35 0.60 1.00], 'LineWidth', 6);
end
markIndex = round((10:10:finishTime)/frameStep) + 1;
plot(courseAxes, frameX(markIndex), frameY(markIndex), 'k.', 'MarkerSize', 12);
text(courseAxes, frameX(markIndex) + 6, frameY(markIndex), ...
    compose('%d s', 10*(1:numel(markIndex))), 'FontSize', 8);
animation.marker = plot(courseAxes, NaN, NaN, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
axis(courseAxes, 'equal');
grid(courseAxes, 'on');
xlabel(courseAxes, 'East [m]');
ylabel(courseAxes, 'North [m]');
title(courseAxes, 'MIS lap (dots every 10 s, blue: low grip)');

% Slip history with a time cursor.
slipAxes = nexttile(animationLayout, 7, [1, 3]);
for section = 1:lowGripCount
    xregion(slipAxes, timeAtDistance(sectionBands(section, 1)), ...
        timeAtDistance(sectionBands(section, 2)), 'FaceColor', [0.2 0.45 0.95], ...
        'FaceAlpha', 0.15, 'HandleVisibility', 'off');
end
hold(slipAxes, 'on');
plot(slipAxes, slipTime, trueSlip, 'LineWidth', 1.0);
set(slipAxes, 'ColorOrderIndex', 1);
plot(slipAxes, [0, finishTime], [slipTargets; slipTargets], ':', ...
    'LineWidth', 1.0, 'HandleVisibility', 'off');
animation.cursor = xline(slipAxes, 0, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
grid(slipAxes, 'on');
xlim(slipAxes, [0, finishTime]);
ylim(slipAxes, [-0.05, max(0.3, min(1.0, 1.1*max(trueSlip(:))))]);
xlabel(slipAxes, 'Time [s]');
ylabel(slipAxes, 'Tire slip [-]');
title(slipAxes, 'Tire slip (dotted: targets; blue bands: low grip)');
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
animation.throttle = frameDriver(:, 1)/Max_Motor_Torque;
animation.brake = frameDriver(:, 2)/(Mv*gravity);
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
animation.rates = [2, 1, 0.5, 0.25, 0.1];
animation.playButton = uicontrol(animationFigure, 'Style', 'pushbutton', ...
    'String', 'Play', 'Units', 'normalized', 'Position', [0.02, 0.015, 0.07, 0.04], ...
    'Callback', @playCarAnimation);
animation.speedMenu = uicontrol(animationFigure, 'Style', 'popupmenu', ...
    'String', {'2x', 'Real time', '1/2 speed', '1/4 speed', '1/10 speed'}, 'Value', 2, ...
    'Units', 'normalized', 'Position', [0.10, 0.015, 0.08, 0.04]);
animation.slider = uicontrol(animationFigure, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.20, 0.02, 0.77, 0.03], ...
    'Min', 0, 'Max', frameTime(end), 'Value', 0, ...
    'SliderStep', [min(1, frameStep/frameTime(end)), min(1, 1/frameTime(end))]);
animationFigure.UserData = animation;
animation.slider.Callback = @(source, ~) drawCarFrame(ancestor(source, 'figure'), ...
    round(source.Value/frameStep) + 1);
addlistener(animation.slider, 'ContinuousValueChange', ...
    @(source, ~) drawCarFrame(ancestor(source, 'figure'), round(source.Value/frameStep) + 1));
animationTitle = title(animationLayout, {'Car on the MIS lap', scenarioLabel});
animationTitle.Color = 'k';
styleSimulationFigure(animationFigure);
drawCarFrame(animationFigure, 1);
clear animation

function [vertices, faces] = roadStrip(x, y, normalX, normalY, keep, rightOffset, leftOffset)
% Quad strip between two offsets from the line (left positive) over the
% points in keep, for a patch with 'Vertices' and 'Faces'.
x = x(keep); y = y(keep); normalX = normalX(keep); normalY = normalY(keep);
n = numel(x);
vertices = [x + leftOffset*normalX, y + leftOffset*normalY; ...
    x + rightOffset*normalX, y + rightOffset*normalY];
quad = (1:n - 1)';
faces = [quad, quad + 1, n + quad + 1, n + quad];
end

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
a.label.String = sprintf(['t = %.2f s   s = %.0f m   v = %.1f m/s\n' ...
    'a_y = %.2f g   throttle %3.0f%%   brake %.2f g\n' ...
    'slip  FL %.3f  FR %.3f  RL %.3f  RR %.3f'], a.time(frame), a.s(frame), ...
    a.speed(frame), a.ay(frame)/a.gravity, 100*a.throttle(frame), ...
    a.brake(frame), a.slip(frame, :));
a.slider.Value = a.time(frame);
a.frame = frame;
figureHandle.UserData = a;
end

function playCarAnimation(button, ~)
% Play or pause the car animation at the selected playback speed.
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
frame = a.frame;
if frame >= numel(a.time)
    frame = 1;
end
rate = a.rates(a.speedMenu.Value);
clockStart = tic;
timeStart = a.time(frame);
lastDrawn = frame;
while isvalid(figureHandle) && figureHandle.UserData.playing && frame < numel(a.time)
    if figureHandle.UserData.frame ~= lastDrawn || a.rates(a.speedMenu.Value) ~= rate
        frame = figureHandle.UserData.frame;   % Slider moved or speed changed
        rate = a.rates(a.speedMenu.Value);
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

function shadeSections(ax, bands)
% Shade each low-grip section (rows of [start, end]) on a distance axis.
for band = 1:size(bands, 1)
    xregion(ax, bands(band, 1), bands(band, 2), 'FaceColor', [0.2 0.45 0.95], ...
        'FaceAlpha', 0.15, 'HandleVisibility', 'off');
end
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
