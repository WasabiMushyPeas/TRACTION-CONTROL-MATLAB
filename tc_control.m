function [Tcmd, dI, e, sched, uff] = ...
        tc_control(slip, I, vx, Fz, Treq, Tmax_drv, Tmax_brk, ay, P)
%TC_CONTROL  Per-wheel BIDIRECTIONAL slip controller (drive + brake). ROW 1x4.
%  Grip FEEDFORWARD + speed-scheduled PI + back-calc anti-windup, saturated to
%  the per-wheel GRIP CEILING (grip_estimator) intersected with the driver
%  request and the motor limit. Floating-point design model; int32 firmware
%  port is separate (keep loop BW < ~1/5 sample rate).
%
%  Inputs  slip 1x4, I 1x4 integral state, vx scalar, Fz 1x4, Treq scalar SIGNED
%          [Nm] (+drive/-brake), Tmax_drv 1x4 (+), Tmax_brk 1x4 (-), ay scalar
%  Outputs Tcmd 1x4 [Nm]; dI 1x4; e,sched,uff diagnostics
%
%  Any-state behaviour:
%   * target slip sign follows requested torque (drive->+slip, brake->-slip),
%     blended smoothly through Treq=0;
%   * window [Tmin,Tmax]: Tmax=min(drive req, motor peak, GRIP drive ceiling),
%     Tmin=max(brake req, GRIP brake ceiling) -> combined-slip capacity limits
%     torque BOTH directions, in a corner as well as straight;
%   * speed schedule fades PI 0->1 over [v_lo v_hi]; at launch AND near a stop
%     the fast high-gain plant is carried by the (ellipse-derated) feedforward,
%     killing the low-speed limit cycle.
%#codegen
    slip = slip(:).';  I = I(:).';  Fz = max(Fz(:).',0);
    Tmax_drv = Tmax_drv(:).';  Tmax_brk = Tmax_brk(:).';

    s_dir = tanh(Treq / (0.25*P.Tmot_pk + eps));     % ~+1 drive, ~-1 brake
    s_tgt = P.slip_tgt * s_dir;                      % scalar
    e     = s_tgt - slip;                            % 1x4

    if P.use_ff
        mu = max((P.D1 - P.D2.*Fz).*P.grip.*P.mu_scale, 0.1);
        if P.use_ellipse
            u_lat = min(abs(ay)/P.g ./ mu, 1);
            ell   = sqrt(max(1 - u_lat.^2, 0));
        else
            ell   = ones(1,4);
        end
        Fxf = (mu.*ell) .* Fz .* sin(P.C*atan(P.B*s_tgt));   % signed via s_tgt
        uff = Fxf .* P.Rw ./ (P.gear*P.eta);
    else
        uff = zeros(1,4);
    end

    sched = min(max((vx - P.v_lo)/(P.v_hi - P.v_lo), 0), 1);
    Kp = P.Kp0*sched;  Ki = P.Ki0*sched;

    up      = Kp .* e;
    u_unsat = uff + up + I;

    Tdrv_mot = P.Tmot_pk;
    Tbrk_mot = -P.Tmot_pk * double(P.regen_on);      % motor regen; friction adds rest
    if P.regen_on
        Tb_floor = max(Tmax_brk, Tbrk_mot);          % motor-only: cap at regen torque
    else
        Tb_floor = Tmax_brk;                         % friction brakes -> grip ceiling
    end
    Tmax = min( max(Treq,0), min(Tdrv_mot, Tmax_drv) );   % 1x4
    Tmin = max( min(Treq,0), Tb_floor );                  % 1x4
    Tcmd = min(max(u_unsat, Tmin), Tmax);

    dI = Ki .* e + P.Kaw .* (Tcmd - u_unsat);        % back-calc anti-windup
end
