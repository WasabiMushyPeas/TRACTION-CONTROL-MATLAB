function Fx = tire_long(slip, Fz, ay, P)
    slip = slip(:).';  Fz = max(Fz(:).', 0);    % ROW 1x4
    mu = max((P.tirePeakMuBase - P.tirePeakMuLoadSensitivity.*Fz) .* ...
        P.globalGripScale .* P.tireGripScaleByWheel, 0.1);
    if P.useFrictionEllipse
        u_lat = min(abs(ay)/P.gravity ./ mu, 1);
        ell   = sqrt(max(1 - u_lat.^2, 0));
    else
        ell   = ones(1,4);
    end
    Fx = (mu.*ell) .* Fz .* sin(P.tireMagicFormulaC.*atan(P.tireMagicFormulaB.*slip));
end
