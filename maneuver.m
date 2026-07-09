function [Treq, ay] = maneuver(t, P)
%MANEUVER  Prescribed driver/scenario demand vs time -> exercises the whole
%  traction envelope (launch, combined-slip cornering, braking). This is what
%  makes the sim "max grip in ANY state" rather than a single launch run.
%
%  Treq  scalar signed torque request per motor [Nm]  (+ drive, - brake)
%  ay    scalar prescribed lateral accel [m/s^2]  (drives lateral load
%        transfer + friction-ellipse consumption; NOT a solved yaw state)
%
%  ay is an INPUT, not an integrated state: the model answers "given this much
%  lateral grip use, how much longitudinal torque can each tire take" without
%  a full yaw-plane vehicle model. Swap this for a real driver/track profile,
%  or feed measured ay, when you couple it to the TV plant.
%#codegen
    b = max(P.man_blend, 1e-3);
    sd = @(t0) smooth_step((t - t0)/b);     % 0->1 ramp centred after t0

    Tdrv_full  = P.Tmot_pk;
    Tdrv_corner= 0.5*P.Tmot_pk;
    Tbrk       = P.Tbrk_pk;

    % torque request: full drive -> half drive -> brake -> brake
    Treq =  Tdrv_full ...
          - (Tdrv_full - Tdrv_corner) * sd(P.t1) ...     % ease off entering corner
          - (Tdrv_corner + Tbrk)      * sd(P.t2) ...     % roll into braking
          + 0*sd(P.t3);                                  % (stays on the brakes)

    % lateral: 0 -> full corner -> partial (trail) -> 0
    ay =  P.ay_corner * sd(P.t1) ...
        - 0.4*P.ay_corner * sd(P.t2) ...                 % unwind to trail-brake level
        - 0.6*P.ay_corner * sd(P.t3);                    % straighten for the stop

    if P.accel_only
        Treq = P.Tmot_pk; ay = 0; return
    end
end

function s = smooth_step(u)
% clamped cubic smoothstep S(u)=u^2(3-2u), u in [0,1]
    u = min(max(u, 0), 1);
    s = u.*u.*(3 - 2*u);
end
