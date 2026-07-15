function [Fz, ax] = load_transfer(Fx_dyn, vx, ay, P)
%LOAD_TRANSFER  Per-wheel normal load: static + aero + LONGITUDINAL + LATERAL.
%  Fx_dyn dynamic tire long. forces [N] (coerced 1x4)
%  vx     scalar speed [m/s]
%  ay     scalar lateral accel [m/s^2]  (>0 loads the RIGHT tires)
%  Fz     1x4 normal load [N]  [FL FR RL RR]
%  ax     scalar longitudinal accel (from Fx_dyn state -> no algebraic loop)
%
%  Lateral transfer per axle uses axle load share as the axle-mass proxy and the
%  single track width; front/rear split follows static+aero balance. Proper roll-
%  stiffness / ARB distribution is the next refinement.
%#codegen
    Fx_dyn = Fx_dyn(:).';                        % ROW 1x4
    drag = 0.5*P.airDensity*P.dragCoefficient*P.frontalArea*vx.^2;
    rr   = P.rollingResistanceCoefficient*P.vehicleMass*P.gravity;
    totalLongitudinalForce = Fx_dyn(1) + Fx_dyn(2) + Fx_dyn(3) + Fx_dyn(4);
    ax   = (totalLongitudinalForce - drag - rr) / P.vehicleMass;

    Wt = P.vehicleMass*P.gravity;
    DF = 0.5*P.airDensity*P.downforceCoefficient*P.frontalArea*vx.^2; % total aero downforce
    Ff = Wt*(P.rearAxleToCg/P.wheelbase) + DF*(1 - P.rearDownforceFraction);
    Fr = Wt*(P.frontAxleToCg/P.wheelbase) + DF*P.rearDownforceFraction;

    dFx = P.vehicleMass*ax*P.centerOfGravityHeight/P.wheelbase; % long. transfer (rearward under +ax)
    Ff  = Ff - dFx;
    Fr  = Fr + dFx;

    dFy_f = (Ff/P.gravity)*ay*P.centerOfGravityHeight/P.trackWidth; % lateral transfer per axle
    dFy_r = (Fr/P.gravity)*ay*P.centerOfGravityHeight/P.trackWidth;

    Fz = [Ff/2 - dFy_f, Ff/2 + dFy_f, Fr/2 - dFy_r, Fr/2 + dFy_r];  % 1x4
    Fz = max(Fz, 0);
end
