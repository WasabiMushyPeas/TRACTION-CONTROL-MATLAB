function ref = sample_mis_track(track, distance_m)
%SAMPLE_MIS_TRACK Distance-based periodic lookup for MATLAB inspection.
% Simulink implementation: mod(s,L), 1-D Lookup Tables, and atan2(sin,cos).
% This helper is ordinary MATLAB; code generation has not been tested.
s = mod(distance_m, track.length_m);
ref.s_m = s;
ref.x_m = interp1(track.s_m, track.x_m, s, 'linear');
ref.y_m = interp1(track.s_m, track.y_m, s, 'linear');
ref.z_m = interp1(track.s_m, track.z_m, s, 'linear');
c = interp1(track.s_m, track.heading_cos, s, 'linear');
h = interp1(track.s_m, track.heading_sin, s, 'linear');
ref.heading_rad = atan2(h,c);
ref.curvature_1pm = interp1(track.s_m, track.curvature_1pm, s, 'linear');
% Recorded speed is deliberately not exposed as a controller target.
end
