function slip = slip_estimator(w, vx, ay, P)
    w = w(:).';                                 % force ROW 1x4
    vref0 = max(abs(vx), P.lowSpeedFloor);
    if P.useCornerSpeedEstimate
        r    = ay / vref0;                      % steady-state yaw rate estimate
        sgn  = [-1 1 -1 1];                      % [FL FR RL RR]: left -, right +
        vx_c = vx + sgn*(r*P.trackWidth/2);     % 1x4 per-corner ground speed
    else
        vx_c = vx*ones(1,4);
    end
    vref = max(abs(vx_c), P.slipSpeedFloor);
    slip = (w*P.wheelRadius - vx_c) ./ vref;    % 1x4
    slip = max(min(slip, 1), -1);
end
