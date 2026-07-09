clear P

%% --- Mass & geometry (from your PARAMS.m / VDS) ---
P.m    = 278.10;            % total mass incl. driver [kg]
P.g    = 9.80665;
P.L    = 1.53035;           % wheelbase [m]
P.t    = 1.22;              % track [m]                         <-- confirm CP27E
P.hcg  = 0.254;             % CG height [m]
P.a    = 29.402*0.0254;     % front axle -> CG [m]   (0.7468)
P.b    = 30.848*0.0254;     % CG -> rear axle [m]    (0.7835)
P.rwf  = P.a/P.L;           % static REAR weight fraction (0.488)

%% --- Tire / wheel ---
P.Rw   = 0.203;             % loaded radius [m]
P.B    = 10.4;
P.C    = 1.58;
P.D1   = 3.02;
P.D2   = 0.0008/4.448221615;    % per N (converted from per lbf)
P.grip = 1.0;                   % global grip scale
P.Lrelax = 0.25;                % tire relaxation LENGTH [m]     <-- from TTC
P.use_relax = true;

% --- surface preset -> per-wheel grip [FL FR RL RR] ---
MU_PRESET = 'dry';              % 'dry' | 'wet' | 'ice' | 'split'
switch MU_PRESET
    case 'dry',   P.mu_scale = [1.00 1.00 1.00 1.00];
    case 'wet',   P.mu_scale = [0.20 0.20 0.20 0.20];
    case 'ice',   P.mu_scale = [0.12 0.12 0.12 0.12];
    case 'split', P.mu_scale = [1.00 0.20 1.00 0.20];   % right side low-grip
end

%% --- Powertrain: 4x AMK DD5-14-10-POW hub motors (datasheet-confirmed) ---
P.gear     = 13.2;          % motor -> wheel reduction (typical AMK DD5 FSE ~13-16)
P.eta      = 0.89;          % drivetrain efficiency
P.Tmot_pk  = 21.0;          % PEAK motor torque / motor [Nm]  (AMK Mmax)
P.Tmot_rat = 9.8;           % rated torque [Nm]  (reference)
P.wmot_max = 12000*2*pi/60; % rated motor speed [rad/s] (mech limit 20000 rpm)
P.wwheel_max = P.wmot_max/P.gear;
P.tau_motor = 0.010;        % inverter+motor torque lag [s]  (AMK fast; was 0.03)
P.regen_on  = false;        % allow negative motor torque

%% --- Per-corner rotational inertia (RIGID hub, NO half-shaft) ---
% Jc = Jrotor*gear^2 + Jwheel_single   (wheel-referenced, per corner)
P.Jrotor = 2.74e-4;         % AMK rotor inertia [kg m^2] (2.74 kg cm^2)
P.Jwheel = 0.16490/2;       % single wheel/tire/hub/upright rot. assy [kg m^2]
P.Jc     = P.Jrotor*P.gear^2 + P.Jwheel;

%% --- Aero (from PARAMS.m) ---
P.rho = 1.225;  P.A = 1.0;  P.Cd = 1.0;  P.Cl = 3.8;  P.Cp = 0.53;   % Cp = rear frac
P.Crr = 0.015;              % rolling resistance coeff

%% --- Controller: per-wheel slip PI + grip feedforward + speed schedule ---
P.slip_tgt = 0.13;          % target slip ratio
P.Kp0 = 120;                % base P gain [Nm per unit slip]     <-- tune
P.Ki0 = 400;                % base I gain [Nm/s per unit slip]   <-- tune
P.Kaw = 8;                  % back-calc anti-windup gain [1/s]   <-- tune
% speed schedule: PI authority 0 (launch) -> 1 (grip), replaces the old hard
% START/END_BLEND switch. Feedforward carries the launch. Smooth => no
% switching transient.
P.v_lo = 2.0;               % PI fade-in start [m/s]
P.v_hi = 6.0;               % PI fade-in end   [m/s]
P.v_floor = 0.7;            % slip-denominator / relaxation floor [m/s]
P.use_ff  = true;           % grip-based feedforward on/off
P.slip_lpf_fc = 0;          % slip meas. LPF corner [Hz], 0 = off (handled in est.)
P.use_ellipse   = true;     % combined-slip friction ellipse (derate Fx by lateral use)
P.use_corner_vel= true;     % per-corner ground speed from yaw (r ~ ay/vx)
P.Tbrk_pk = 2.0*P.Tmot_pk;  % brake torque cap, motor-referenced [Nm]. Motor regen
                            % covers up to Tmot_pk*regen_on; friction brakes the rest.
                            % Set high so the GRIP ceiling is the limiter (brake-side TC).

%% --- Maneuver / sim -------------------------------------------------------
% This is NOT just a launch run. maneuver.m sweeps the car through several
% states so the model reports the per-wheel traction CEILING in each:
%   [0, t1)      standing launch, straight        (drive-limited / launch TC)
%   [t1, t2)     partial throttle + hard cornering (COMBINED slip, ellipse)
%   [t2, t3)     trail-brake into the corner        (brake TC + lateral)
%   [t3, tEnd]   straight-line braking              (brake-side TC)
P.accel_only = true;
P.t1 = 2.5;                 % end of launch [s]
P.t2 = 4.0;                 % end of corner-exit accel [s]
P.t3 = 5.0;                 % end of trail-brake [s]
P.ay_corner = 12.0;         % prescribed lateral accel in the corner [m/s^2] (~1.2g)
P.man_blend = 0.15;         % phase-transition smoothing time [s]
P.T_request = P.Tmot_pk;    % reference full drive torque/motor [Nm]
P.v0   = 0.3;               % initial speed [m/s]
P.tEnd = 6;                 % sim time [s]
P.solver = 'ode23t';        % stiff (relaxation + fast wheel modes)
P.maxStep = 1e-3;

%% --- Initial state vector  X = [vx; w(4); Fx_dyn(4); Tmot(4)]  (13x1) ---
w0 = P.v0/P.Rw;
X0 = [P.v0;  w0*ones(4,1);  zeros(4,1);  zeros(4,1)];
I0 = zeros(1,4);            % controller integral state (ROW, matches ctrl I/O)

assignin('base','P',P);
assignin('base','X0',X0);
assignin('base','I0',I0);

fprintf('params.m: MU_PRESET=%s  gear=%.2f  Tmot_pk=%.1f Nm  Jc=%.4f kg m^2\n', ...
    MU_PRESET, P.gear, P.Tmot_pk, P.Jc);
