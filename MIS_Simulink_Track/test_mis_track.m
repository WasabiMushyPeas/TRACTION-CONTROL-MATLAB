function test_mis_track()
%TEST_MIS_TRACK Run in MATLAB after adding this folder to the path.
track = load_mis_track();
assert(all(diff(track.s_m)>0));
assert(numel(track.s_m)==1990);
assert(abs(trapz(track.s_m,track.curvature_1pm)-2*pi)<0.03);
ref = sample_mis_track(track, [0, track.length_m, 2*track.length_m]);
assert(max(abs(ref.x_m-ref.x_m(1)))<1e-10);
assert(max(abs(ref.y_m-ref.y_m(1)))<1e-10);
assert(max(abs(ref.curvature_1pm-ref.curvature_1pm(1)))<1e-10);
ref = sample_mis_track(track,[-0.001,0.001]);
assert(hypot(diff(ref.x_m),diff(ref.y_m))<0.01);
assert(abs(atan2(sin(diff(ref.heading_rad)),cos(diff(ref.heading_rad))))<0.01);
figure('Name','MIS reference path');
plot(track.x_m,track.y_m);axis equal;grid on;
xlabel('East (m)');ylabel('North (m)');
title('MIS GPS-derived driven path');
fprintf('Track checks passed. Length %.3f m, %d points.\n', ...
    track.length_m,numel(track.s_m));
end
