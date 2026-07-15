clear P

% --- Mass / geometry ---
P.m    = 278.10;            % car mass w/ driver [kg]
P.g    = 9.80665;           % gravity [m/s^2]
P.L    = 1.53035;           % wheelbase [m]
P.t    = 1.22;              % track [m]
P.hcg  = 0.254;             % CG height [m]
P.a    = 29.402*0.0254;     % front axle to CG [m]   
P.b    = 30.848*0.0254;     % CG to rear axle [m]    
P.rwf  = P.a/P.L;           % static rear weight fraction

% --- Tire / wheel ---
P.Rw   = 0.203;             % loaded radius [m]
P.B    = 10.4;
P.C    = 1.58;
P.D1   = 3.02;
P.D2   = 0.0008/4.448221615;    % per N (converted from per lbf)
P.grip = 1.0;                   % global grip scale
P.Lrelax = 0.25;                % tire relaxation length [m]
P.use_relax = true;
P.v_floor_slip = 1.0;       % slip denominator floor [m/s]


MU_PRESET = 'dry';
switch MU_PRESET
    case 'dry',   P.mu_scale = [1.00 1.00 1.00 1.00];
    case 'wet',   P.mu_scale = [0.20 0.20 0.20 0.20];
    case 'ice',   P.mu_scale = [0.12 0.12 0.12 0.12];
    case 'messed-up', P.mu_scale = [1.00 0.20 1.00 0.20];
end

% --- Powertrain ---
P.gear     = 10;            % gear ratio
P.eta      = 0.89;          % drivetrain efficiency
P.Tmot_pk  = 21.0;          % peak motor torque [Nm]
P.Pmot_pk  = 15.0e3;        % PEAK power per motor [W]
P.Pcap_veh = 80.0e3;        % FSAE EV total power rule [W]
P.wmot_max = 12000*2*pi/60; % rated motor speed [rad/s] THIS PROB NEEDS TO BE CHANGED
P.wwheel_max = P.wmot_max/P.gear;
P.tau_motor = 0.010;        % inverter plus motor torque lag [s]

% --- Rotational inertia ---
P.Jrotor = 2.74e-4;         % Rotor inertia [kg m^2]
P.Jwheel = 0.16490/2;       % single wheel/tire/hub/gear stuff [kg m^2] PROB NEEDS TO CHANGE
P.Jc     = P.Jrotor*P.gear^2 + P.Jwheel;

% --- Aero ---
P.rho = 1.225;
P.A = 1.0;
P.Cd = 1.0;
P.Cl = 3.8;
P.Cp = 0.53;   % Cp = rear frac
P.Crr = 0.015; % rolling resistance coeff

% --- Controller ---
P.slip_tgt = 0.17;           % target slip ratio
P.Kp0 = 30;
P.Ki0 = 300;
P.Kaw = 15;                  % back-calc anti-windup gain [1/s]   <-- tune

P.v_floor = 0.1;            % slip-denominator / relaxation floor [m/s]
P.use_ff  = true;           % grip-based feedforward on/off
P.slip_lpf_fc = 0;          % slip meas. LPF corner [Hz], 0 = off (handled in est.)
P.use_ellipse   = true;     % combined-slip friction ellipse (derate Fx by lateral use)
P.use_corner_vel= true;     % per-corner ground speed from yaw (r ~ ay/vx)

% --- Maneuver ---
P.accel_only = true;
P.t1 = 2.5;                 % end of launch [s]
P.ay_corner = 12.0;         % prescribed lateral accel in the corner [m/s^2]
P.man_blend = 0.15;         % phase-transition smoothing time [s]
P.T_request = P.Tmot_pk;    % reference full drive torque/motor [Nm]
P.v0   = 0.3;               % initial speed [m/s]
P.tEnd = 10;                 % sim time [s]
P.solver = 'ode23t';        % stiff (relaxation + fast wheel modes)
P.maxStep = 1e-3;

% --- Initial stuff ---
w0 = P.v0/P.Rw;
X0 = [P.v0;  w0*ones(4,1);  zeros(4,1);  zeros(4,1)];
I0 = zeros(1,4);            % controller integral state (ROW, matches ctrl I/O)

assignin('base','P',P);
assignin('base','X0',X0);
assignin('base','I0',I0);

fprintf('params.m: MU_PRESET=%s  gear=%.2f  Tmot_pk=%.1f Nm  Jc=%.4f kg m^2\n', ...
    MU_PRESET, P.gear, P.Tmot_pk, P.Jc);
