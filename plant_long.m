function dX = plant_long(X, Tcmd, Fx_ss, P)
%PLANT_LONG  4-wheel longitudinal plant, hub motors (RIGID, no half-shaft).
%  State X (13x1, COLUMN) = [vx; w(4); Fx_dyn(4); Tmot(4)]
%  Inputs Tcmd 1x4 motor torque cmd [Nm] (signed: +drive/-brake),
%         Fx_ss 1x4 steady tire force from tire_long [N]
%  Output dX 13x1 column
%
%  Per-wheel signals arrive as ROW 1x4; the state is COLUMN -> coerce commands
%  to columns here so dX assembles cleanly. Hub motors: rotor+gear+wheel rigid,
%  so NO half-shaft spring/damper (old 2WD K,C gone) -> that driveline resonance
%  is removed:  Jc*dw = Tmot*gear*eta - Fx*Rw.
%  Braking is just negative Tmot -> negative Twheel (works unchanged).
%  Tire force uses a relaxation-length lag tau = Lrelax/max(vx,v_floor): slow at
%  low speed (physical), fast at speed. Using Fx_dyn (a state) for the wheel/veh
%  dynamics and Fx_ss only to drive relaxation keeps the model algebraic-loop-free.
%#codegen
    vx   = X(1);
    w    = X(2:5);                          % 4x1 col
    Fxd  = X(6:9);                          % 4x1 col
    Tmot = X(10:13);                        % 4x1 col
    Tcmd  = Tcmd(:);                         % -> 4x1 col
    Fx_ss = Fx_ss(:);                        % -> 4x1 col

    drag = 0.5*P.rho*P.Cd*P.A*vx.^2;
    rr   = P.Crr*P.m*P.g*tanh(vx/0.5);      % smooth rolling-resistance sign near 0
    dvx  = (sum(Fxd) - drag - rr) / P.m;

    Twheel = Tmot .* P.gear .* P.eta;       % 4x1
    dw     = (Twheel - Fxd.*P.Rw) ./ P.Jc;  % 4x1

    if P.use_relax
        tau_r = P.Lrelax ./ max(abs(vx), P.v_floor);
        dFxd  = (Fx_ss - Fxd) ./ tau_r;
    else
        dFxd  = (Fx_ss - Fxd) ./ P.maxStep;
    end

    dTmot = (Tcmd - Tmot) ./ P.tau_motor;   % 4x1

    dX = [dvx; dw; dFxd; dTmot];            % 13x1 col
end
