function slipRatio = slip_estimator(wheelAngularSpeed, vehicleSpeed, params)
    wheelAngularSpeed = wheelAngularSpeed(:).';
    slipRatio = (wheelAngularSpeed * params.wheelRadius - vehicleSpeed) ./ max(abs(vehicleSpeed), params.slipSpeedFloor);
    slipRatio = max(min(slipRatio, 1), -1);
end