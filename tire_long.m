function Fx = tire_long(slip, Fz, ay, P)
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
