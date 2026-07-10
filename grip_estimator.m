function [Fx_est, mu_util, Fx_cap, Tmax_drv, margin] = grip_estimator(Fx_dyn, Fz, slip, ay, P)

    Fx_dyn = Fx_dyn(:).';  Fz = max(Fz(:).',0);  slip = slip(:).';   % ROW 1x4
    mu = max((P.D1 - P.D2.*Fz).*P.grip.*P.mu_scale, 0.1);
    if P.use_ellipse
        u_lat = min(abs(ay)/P.g ./ mu, 1);
        ell   = sqrt(max(1 - u_lat.^2, 0));
    else
        ell   = ones(1,4);
    end
    Fx_est  = Fx_dyn;
    Fx_cap  = (mu.*ell) .* Fz;                        % combined long. capacity [N]
    mu_util = abs(Fx_est) ./ max(Fx_cap, 1);
    Tmax_drv =  Fx_cap .* P.Rw ./ (P.gear*P.eta);
    slip_pk = tan(pi/(2*P.C)) / P.B;                 % scalar
    margin  = slip_pk - abs(slip);
end
