function [normalLoad, longitudinalAcceleration] = ...
    load_transfer(dynamicLongitudinalForce, vehicleSpeed, params)

    dynamicLongitudinalForce = dynamicLongitudinalForce(:).';

    aerodynamicDrag = 0.5 * params.airDensity * params.dragCoefficient * params.frontalArea * vehicleSpeed.^2;
    rollingResistance = params.rollingResistanceCoefficient * params.vehicleMass * params.gravity * tanh(vehicleSpeed / 0.5);
    
    totalLongitudinalForce = sum(dynamicLongitudinalForce);

    longitudinalAcceleration = (totalLongitudinalForce - aerodynamicDrag - rollingResistance) / params.vehicleMass;

    vehicleWeight = params.vehicleMass * params.gravity;
    aeroDownforce = 0.5 * params.airDensity * params.downforceCoefficient * params.frontalArea * vehicleSpeed.^2;

    frontAxleLoad = vehicleWeight * (params.rearAxleToCg / params.wheelbase) + aeroDownforce * (1 - params.rearDownforceFraction);
    rearAxleLoad = vehicleWeight * (params.frontAxleToCg / params.wheelbase) + aeroDownforce * params.rearDownforceFraction;

    longitudinalLoadTransfer = params.vehicleMass * longitudinalAcceleration * ...
        params.centerOfGravityHeight / params.wheelbase;
    frontAxleLoad = frontAxleLoad - longitudinalLoadTransfer;
    rearAxleLoad = rearAxleLoad + longitudinalLoadTransfer;


    normalLoad = max([frontAxleLoad/2, frontAxleLoad/2, rearAxleLoad/2,  rearAxleLoad/2], 0);
end
