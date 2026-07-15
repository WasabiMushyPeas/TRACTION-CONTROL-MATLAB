function maxDriveTorque = grip_estimator(normalLoad, params)
    normalLoad = max(normalLoad(:).', 0);
    availableFriction = max((params.tirePeakMuBase - ...
    params.tirePeakMuLoadSensitivity .* normalLoad) .* params.globalGripScale, 0.1);
    maxDriveTorque = availableFriction .* normalLoad .* params.wheelRadius ./ (params.gearRatio * params.drivetrainEfficiency);
end