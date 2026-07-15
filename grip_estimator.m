function [Fx_est, mu_util, Fx_cap, Tmax_drv, margin] = grip_estimator(Fx_dyn, Fz, slip, ay, P)

    Fx_dyn = Fx_dyn(:).';  Fz = max(Fz(:).',0);  slip = slip(:).';   % ROW 1x4
    mu = max((P.tirePeakMuBase - P.tirePeakMuLoadSensitivity.*Fz) .* ...
        P.globalGripScale .* P.tireGripScaleByWheel, 0.1);
    if P.useFrictionEllipse
        u_lat = min(abs(ay)/P.gravity ./ mu, 1);
        ell   = sqrt(max(1 - u_lat.^2, 0));
    else
        ell   = ones(1,4);
    end
    Fx_est  = Fx_dyn;
    Fx_cap  = (mu.*ell) .* Fz;                        % combined long. capacity [N]
    mu_util = abs(Fx_est) ./ max(Fx_cap, 1);
    Tmax_drv =  Fx_cap .* P.wheelRadius ./ (P.gearRatio*P.drivetrainEfficiency);
    slip_pk = tan(pi/(2*P.tireMagicFormulaC)) / P.tireMagicFormulaB; % scalar
    margin  = slip_pk - abs(slip);
end
