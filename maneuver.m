function [Treq, ay] = maneuver(t, P)

    b = max(P.man_blend, 1e-3);
    sd = @(t0) smooth_step((t - t0)/b);     % 0->1 ramp centred after t0

    Tdrv_full  = 0.2*P.Tmot_pk;
    Tdrv_corner= 0.5*P.Tmot_pk;

    % torque request: full drive -> half drive through the corner
    Treq = Tdrv_full - (Tdrv_full - Tdrv_corner) * sd(P.t1);

    % lateral: 0 -> full corner
    ay = P.ay_corner * sd(P.t1);

    if P.accel_only
        Treq = 0.2*P.Tmot_pk; ay = 0; return
    end
end

function s = smooth_step(u)
% clamped cubic smoothstep S(u)=u^2(3-2u), u in [0,1]
    u = min(max(u, 0), 1);
    s = u.*u.*(3 - 2*u);
end
