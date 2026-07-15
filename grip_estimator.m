function [estimatedLongitudinalForce, frictionUtilization, ...
    longitudinalForceCapacity, maxDriveTorque, slipMargin] = ...
    grip_estimator(dynamicLongitudinalForce, normalLoad, slipRatio, lateralAcceleration, params)

    dynamicLongitudinalForce = dynamicLongitudinalForce(:).';
    normalLoad = max(normalLoad(:).', 0);
    slipRatio = slipRatio(:).';

    availableFriction = max( ...
        (params.tirePeakMuBase - params.tirePeakMuLoadSensitivity .* normalLoad) .* ...
        params.globalGripScale .* params.tireGripScaleByWheel, ...
        0.1);

    if params.useFrictionEllipse
        lateralGripFraction = min(abs(lateralAcceleration) / params.gravity ./ availableFriction, 1);
        longitudinalGripFraction = sqrt(max(1 - lateralGripFraction.^2, 0));
    else
        longitudinalGripFraction = ones(1,4);
    end

    estimatedLongitudinalForce = dynamicLongitudinalForce;
    longitudinalForceCapacity = availableFriction .* longitudinalGripFraction .* normalLoad;
    frictionUtilization = abs(estimatedLongitudinalForce) ./ max(longitudinalForceCapacity, 1);
    maxDriveTorque = longitudinalForceCapacity .* params.wheelRadius ./ ...
        (params.gearRatio * params.drivetrainEfficiency);

    peakSlipRatio = tan(pi / (2 * params.tireMagicFormulaC)) / params.tireMagicFormulaB;
    slipMargin = peakSlipRatio - abs(slipRatio);
end
