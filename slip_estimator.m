function slip = slip_estimator(w, vx, ay, P)
%SLIP_ESTIMATOR  Per-wheel longitudinal slip ratio (driving convention).
%  w    wheel speeds [rad/s]  [FL FR RL RR]  (any orientation -> coerced 1x4)
%  vx   scalar vehicle longitudinal speed [m/s]
%  ay   scalar lateral accel [m/s^2]  (for per-corner ground speed)
%  slip 1x4 slip ratio, clamped to [-1, 1]
%
%  Convention: all per-wheel signals are ROW 1x4. Only the plant state vector
%  and its derivative are columns (see plant_long).
%
%  In a corner the outer wheels travel faster than the CG. Using a steady-state
%  yaw estimate r ~ ay/vx, each corner ground speed is vx +/- r*t/2 -- the
%  "any state" correction to the old straight-line slip. use_corner_vel=false
%  recovers straight-line slip. Merge with the TV plant later for true r + steer.
%
%  Low speed: reference floored at v_floor so slip stays defined at launch and
%  down to a stop instead of blowing up as vx -> 0.
%#codegen
    w = w(:).';                                 % force ROW 1x4
    vref0 = max(abs(vx), P.v_floor);
    if P.use_corner_vel
        r    = ay / vref0;                      % steady-state yaw rate estimate
        sgn  = [-1 1 -1 1];                      % [FL FR RL RR]: left -, right +
        vx_c = vx + sgn*(r*P.t/2);              % 1x4 per-corner ground speed
    else
        vx_c = vx*ones(1,4);
    end
    vref = max(abs(vx_c), P.v_floor);
    slip = (w*P.Rw - vx_c) ./ vref;             % 1x4
    slip = max(min(slip, 1), -1);
end
