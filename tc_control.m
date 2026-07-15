function [commandedMotorTorque, integralStateRate, slipError] = tc_control(slipRatio, integralState, vehicleSpeed, normalLoad, ...
    gripLimitedDriveTorque, params)

    slipRatio = slipRatio(:).';
    integralState = integralState(:).';
    normalLoad = max(normalLoad(:).', 0);
    gripLimitedDriveTorque = gripLimitedDriveTorque(:).';

    slipError = params.targetSlipRatio - slipRatio;

    if params.useGripFeedforward
        availableFrictionCoefficient = max((params.tirePeakMuBase - params.tirePeakMuLoadSensitivity .* normalLoad) .* ...
        params.globalGripScale, 0.1);

        feedforwardTireForce = availableFrictionCoefficient .* normalLoad .* sin(params.tireMagicFormulaC * atan( ...
        params.tireMagicFormulaB * params.targetSlipRatio));

        feedforwardTorque = feedforwardTireForce .* params.wheelRadius ./ (params.gearRatio * params.drivetrainEfficiency);
    else
        feedforwardTorque = zeros(1, 4);
    end

    proportionalTorque = params.slipProportionalGain .* slipError;
    unlimitedTorqueCommand = feedforwardTorque + proportionalTorque + integralState;

    motorSpeed = max(abs(vehicleSpeed) / params.wheelRadius * params.gearRatio, 1);
    usablePowerPerMotor = min(params.peakMotorPowerPerMotor, params.vehiclePowerLimit / 4);
    motorDriveTorqueLimit = min(params.peakMotorTorque, usablePowerPerMotor ./ motorSpeed);

    maximumAllowedTorque = min(motorDriveTorqueLimit, gripLimitedDriveTorque);
    commandedMotorTorque = min(max(unlimitedTorqueCommand, 0), maximumAllowedTorque);

    integralStateRate = params.slipIntegralGain .* slipError + params.antiWindupGain .* (commandedMotorTorque - unlimitedTorqueCommand);
end