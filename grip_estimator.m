function [Fx_est, mu_util, Fx_cap, Tmax_drv, Tmax_brk, margin] = ...
        grip_estimator(Fx_dyn, Fz, slip, ay, P)
%GRIP_ESTIMATOR  Per-wheel grip state + traction CEILING -- the core "how much
%  can this tire take right now, in ANY state" block. All I/O ROW 1x4.
%
%  Inputs  Fx_dyn 1x4 current tire long. force [N]; Fz 1x4 [N]; slip 1x4;
%          ay scalar [m/s^2]
%  Outputs Fx_est   1x4 est. long. force [N] (= Fx_dyn; on-car reconstruct
%                   Fx = (Tmot*gear*eta - Jc*domega)/Rw from torque + wheel accel)
%          mu_util  1x4 COMBINED utilization |Fx|/Fx_cap  (1.0 = ellipse edge)
%          Fx_cap   1x4 MAX longitudinal force available now [N] (ellipse-derated)
%          Tmax_drv 1x4 max DRIVE motor torque to reach +Fx_cap [Nm] (+)
%          Tmax_brk 1x4 max BRAKE motor torque to reach -Fx_cap [Nm] (-)
%          margin   1x4 slip margin to the mu-slip peak (>0 below peak)
%
%  Tmax_drv / Tmax_brk are the per-wheel friction-circle constraints -- feed
%  them to the controller now, and to the TV allocator (QP/WLS) later.
%#codegen
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
    Tmax_brk = -Fx_cap .* P.Rw ./ (P.gear*P.eta);
    slip_pk = tan(pi/(2*P.C)) / P.B;                 % scalar
    margin  = slip_pk - abs(slip);
end
