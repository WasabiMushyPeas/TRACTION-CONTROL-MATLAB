function Fx = tire_long(slip, Fz, ay, P)
%TIRE_LONG  Single-wheel longitudinal force with COMBINED-slip derating.
%  slip 1x4 slip ratio, Fz 1x4 normal load [N], ay scalar [m/s^2]
%  Fx   1x4 STEADY-STATE longitudinal force [N] (relaxation applied in plant)
%
%  mu(Fz) = (D1 - D2*Fz)*grip*mu_scale. Lateral use consumes u_lat = |ay|/(g*mu)
%  of the circle, leaving longitudinal fraction sqrt(1-u_lat^2) (friction
%  ellipse). u_lat is Fz-independent under Fy_i ~ Fz_i*ay/g. use_ellipse=false
%  recovers the pure-longitudinal tire.
%#codegen
    slip = slip(:).';  Fz = max(Fz(:).', 0);    % ROW 1x4
    mu = max((P.D1 - P.D2.*Fz).*P.grip.*P.mu_scale, 0.1);
    if P.use_ellipse
        u_lat = min(abs(ay)/P.g ./ mu, 1);
        ell   = sqrt(max(1 - u_lat.^2, 0));
    else
        ell   = ones(1,4);
    end
    Fx = (mu.*ell) .* Fz .* sin(P.C.*atan(P.B.*slip));
end
