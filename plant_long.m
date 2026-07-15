function stateDerivative = plant_long(stateVector, commandedMotorTorque, ...
    steadyStateTireForce, params)

    vehicleSpeed = stateVector(1);
    wheelSpeed = stateVector(2:5);
    dynamicTireForce = stateVector(6:9);
    motorTorque = stateVector(10:13);

    commandedMotorTorque = commandedMotorTorque(:);
    steadyStateTireForce = steadyStateTireForce(:);

    aerodynamicDrag = 0.5 * params.airDensity * params.dragCoefficient * ...
        params.frontalArea * vehicleSpeed.^2;
    rollingResistance = params.rollingResistanceCoefficient * ...
        params.vehicleMass * params.gravity * tanh(vehicleSpeed / 0.5);
    vehicleAcceleration = (sum(dynamicTireForce) - aerodynamicDrag - ...
        rollingResistance) / params.vehicleMass;

    wheelTorque = motorTorque .* params.gearRatio .* ...
        params.drivetrainEfficiency;
    wheelAcceleration = (wheelTorque - dynamicTireForce .* params.wheelRadius) ...
        ./ params.combinedWheelInertia;

    if params.useTireRelaxation
        tireContactSpeed = max(max(abs(vehicleSpeed), ...
            abs(wheelSpeed) .* params.wheelRadius), 1.0);
        tireForceTimeConstant = params.tireRelaxationLength ./ tireContactSpeed;
        tireForceRate = (steadyStateTireForce - dynamicTireForce) ...
            ./ tireForceTimeConstant;
    else
        tireForceRate = (steadyStateTireForce - dynamicTireForce) ...
            ./ params.maxTimeStep;
    end

    motorTorqueRate = (commandedMotorTorque - motorTorque) ...
        ./ params.motorTorqueTimeConstant;

    stateDerivative = [vehicleAcceleration; wheelAcceleration; ...
        tireForceRate; motorTorqueRate];
end
