function [Tcmd, dI, e, sched, uff] = ...
        tc_control(slip, I, vx, Fz, Treq, Tmax_drv, ay, P)
%TC_CONTROL  Per-wheel DRIVE slip controller. ROW 1x4.
%  Grip FEEDFORWARD + speed-scheduled PI + back-calc anti-windup, saturated to
%  the per-wheel GRIP CEILING (grip_estimator) intersected with the driver
%  request and the motor limit. Floating-point design model; int32 firmware
%  port is separate (keep loop BW < ~1/5 sample rate).
%
%  Inputs  slip 1x4, I 1x4 integral state, vx scalar, Fz 1x4, Treq scalar
%          [Nm] (drive request, >=0), Tmax_drv 1x4 (+), ay scalar
%  Outputs Tcmd 1x4 [Nm]; dI 1x4; e,sched,uff diagnostics
%
%  Any-state behaviour:
%   * window [0,Tmax]: Tmax=min(drive req, motor peak, GRIP drive ceiling)
%     -> combined-slip capacity limits torque in a corner as well as straight;
%   * speed schedule fades PI 0->1 over [v_lo v_hi]; at launch the fast
%     high-gain plant is carried by the (ellipse-derated) feedforward,
%     killing the low-speed limit cycle.
%#codegen
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
