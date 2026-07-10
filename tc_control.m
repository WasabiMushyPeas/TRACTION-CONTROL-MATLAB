function [Tcmd, dI, e, sched, uff] = tc_control(slip, I, vx, Fz, Treq, Tmax_drv, ay, P)

    slip = slip(:).';  I = I(:).';  Fz = max(Fz(:).',0);
    Tmax_drv = Tmax_drv(:).';

    s_tgt = P.slip_tgt;                               % scalar
    e     = s_tgt - slip;                             % 1x4

    if P.use_ff
        mu = max((P.D1 - P.D2.*Fz).*P.grip.*P.mu_scale, 0.1);
        if P.use_ellipse
            u_lat = min(abs(ay)/P.g ./ mu, 1);
            ell   = sqrt(max(1 - u_lat.^2, 0));
        else
            ell   = ones(1,4);
        end
        Fxf = (mu.*ell) .* Fz .* sin(P.C*atan(P.B*s_tgt));
        uff = Fxf .* P.Rw ./ (P.gear*P.eta);
    else
        uff = zeros(1,4);
    end
    
    if P.sched_on
        sched = min(max((vx - P.v_lo)/(P.v_hi - P.v_lo), 0), 1);
    else
        sched = 1;                 % full PID from the start
    end
    Kp = P.Kp0*sched;  Ki = P.Ki0*sched;

    up      = Kp .* e;
    u_unsat = uff + up + I;

    wmot     = max(abs(vx)/P.Rw*P.gear, 1);          % motor speed [rad/s], floored
    Pshare   = min(P.Pmot_pk, P.Pcap_veh/4);         % usable peak power / motor [W]
    Tdrv_mot = min(P.Tmot_pk, Pshare./wmot);         % drive torque envelope [Nm]

    Tmax = min( max(Treq,0), min(Tdrv_mot, Tmax_drv) );   % 1x4
    Tcmd = min(max(u_unsat, 0), Tmax);

    dI = Ki .* e + P.Kaw .* (Tcmd - u_unsat);        % back-calc anti-windup
end
