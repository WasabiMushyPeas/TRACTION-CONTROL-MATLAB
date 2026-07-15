function dX = plant_long(X, Tcmd, Fx_ss, P)
    vx   = X(1);
    w    = X(2:5);                          % 4x1 col
    Fxd  = X(6:9);                          % 4x1 col
    Tmot = X(10:13);                        % 4x1 col
    Tcmd  = Tcmd(:);                         % -> 4x1 col
    Fx_ss = Fx_ss(:);                        % -> 4x1 col

    drag = 0.5*P.airDensity*P.dragCoefficient*P.frontalArea*vx.^2;
    rr   = P.rollingResistanceCoefficient*P.vehicleMass*P.gravity*tanh(vx/0.5); % smooth sign near 0
    dvx  = (sum(Fxd) - drag - rr) / P.vehicleMass;

    Twheel = Tmot .* P.gearRatio .* P.drivetrainEfficiency;      % 4x1
    dw     = (Twheel - Fxd.*P.wheelRadius) ./ P.combinedWheelInertia; % 4x1
     
    if P.useTireRelaxation
        vrel  = max(max(abs(vx), abs(w).*P.wheelRadius), 1.0);   % 4x1: tire sweeps at wheel speed
        tau_r = P.tireRelaxationLength ./ vrel;
        dFxd  = (Fx_ss - Fxd) ./ tau_r;
    else
        dFxd  = (Fx_ss - Fxd) ./ P.maxTimeStep;
    end

    dTmot = (Tcmd - Tmot) ./ P.motorTorqueTimeConstant;   % 4x1

    dX = [dvx; dw; dFxd; dTmot];            % 13x1 col
end
