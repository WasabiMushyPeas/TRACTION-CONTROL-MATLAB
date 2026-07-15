function longitudinalTireForce = tire_long(slipRatio, normalLoad, params)
    slipRatio = slipRatio(:).';
    normalLoad = max(normalLoad(:).', 0);
    
    availableFrictionCoefficient = max((params.tirePeakMuBase - params.tirePeakMuLoadSensitivity .* normalLoad) .* ...
    params.globalGripScale, 0.1);
    
    longitudinalTireForce = availableFrictionCoefficient .* normalLoad .* sin(params.tireMagicFormulaC .* atan( ...
    params.tireMagicFormulaB .* slipRatio));
end