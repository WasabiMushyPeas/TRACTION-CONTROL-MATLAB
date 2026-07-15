function [torqueRequest, lateralAcceleration] = maneuver(currentTime, params)

    blendTime = max(params.maneuverBlendTime, 1e-3);
    launchToCornerProgress = smoothStep((currentTime - params.launchEndTime) / blendTime);

    launchTorqueRequest = params.fullDriveTorqueRequest;
    cornerTorqueRequest = 0.5 * params.fullDriveTorqueRequest;

    torqueRequest = launchTorqueRequest - ...
        (launchTorqueRequest - cornerTorqueRequest) * launchToCornerProgress;

    lateralAcceleration = ...
        params.cornerLateralAcceleration * launchToCornerProgress;
    
    if params.accelerationOnly
        torqueRequest = params.fullDriveTorqueRequest;
        lateralAcceleration = 0;
    end
end

function progress = smoothStep(rawProgress)
    progress = min(max(rawProgress, 0), 1);
    progress = progress .* progress .* (3 - 2 * progress);
end
