function [Tcmd, dI, e, sched, uff] = tc_control(slip, I, vx, Fz, Treq, Tmax_drv, ay, P)

    slip = slip(:).';  I = I(:).';  Fz = max(Fz(:).',0);
    Tmax_drv = Tmax_drv(:).';
    sched = 1;

    s_tgt = P.targetSlipRatio;                        % scalar
    e     = s_tgt - slip;                             % 1x4

    if P.useGripFeedforward
        mu = max((P.tirePeakMuBase - P.tirePeakMuLoadSensitivity.*Fz) .* ...
            P.globalGripScale .* P.tireGripScaleByWheel, 0.1);
        if P.useFrictionEllipse
            u_lat = min(abs(ay)/P.gravity ./ mu, 1);
            ell   = sqrt(max(1 - u_lat.^2, 0));
        else
            ell   = ones(1,4);
        end
        Fxf = (mu.*ell) .* Fz .* sin(P.tireMagicFormulaC*atan(P.tireMagicFormulaB*s_tgt));
        uff = Fxf .* P.wheelRadius ./ (P.gearRatio*P.drivetrainEfficiency);
    else
        uff = zeros(1,4);
    end
    
    Kp = P.slipProportionalGain;  Ki = P.slipIntegralGain;

    up      = Kp .* e;
    u_unsat = uff + up + I;

    wmot     = max(abs(vx)/P.wheelRadius*P.gearRatio, 1);       % motor speed [rad/s], floored
    Pshare   = min(P.peakMotorPowerPerMotor, P.vehiclePowerLimit/4); % usable peak power / motor [W]
    Tdrv_mot = min(P.peakMotorTorque, Pshare./wmot);             % drive torque envelope [Nm]

    Tmax = min( max(Treq,0), min(Tdrv_mot, Tmax_drv) );   % 1x4
    Tcmd = min(max(u_unsat, 0), Tmax);

    dI = Ki .* e + P.antiWindupGain .* (Tcmd - u_unsat); % back-calc anti-windup
end
