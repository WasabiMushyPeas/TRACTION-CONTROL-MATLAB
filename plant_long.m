function dX = plant_long(X, Tcmd, Fx_ss, P)
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
