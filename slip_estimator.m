function slipRatio = slip_estimator(wheelAngularSpeed, vehicleSpeed, ...
    lateralAcceleration, params)

    wheelAngularSpeed = wheelAngularSpeed(:).';

    vehicleSpeedForYawEstimate = max(abs(vehicleSpeed), params.lowSpeedFloor);
    if params.useCornerSpeedEstimate
        yawRateEstimate = lateralAcceleration / vehicleSpeedForYawEstimate;
        cornerSideSign = [-1, 1, -1, 1];
        cornerGroundSpeed = vehicleSpeed + cornerSideSign * ...
            (yawRateEstimate * params.trackWidth / 2);
    else
        cornerGroundSpeed = vehicleSpeed * ones(1, 4);
    end

    referenceSpeed = max(abs(cornerGroundSpeed), params.slipSpeedFloor);
    slipRatio = (wheelAngularSpeed * params.wheelRadius - cornerGroundSpeed) ...
        ./ referenceSpeed;
    slipRatio = max(min(slipRatio, 1), -1);
end
