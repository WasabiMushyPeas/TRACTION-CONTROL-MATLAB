function track = load_mis_track(filePath)
%LOAD_MIS_TRACK Load and check the GPS-derived reference path. No toolboxes.
% Usage: track = load_mis_track();
if nargin < 1
    filePath = fullfile(fileparts(mfilename('fullpath')), 'mis_track.mat');
end
data = load(filePath, 'track');
track = data.track;
names = {'s_m','x_m','y_m','z_m','heading_rad','heading_cos', ...
    'heading_sin','curvature_1pm','speed_recorded_mps', ...
    'gps_altitude_recorded_m','time_recorded_s','gps_hdop'};
for i = 1:numel(names)
    v = track.(names{i});
    assert(isnumeric(v) && isvector(v) && all(isfinite(v)), ...
        'Invalid track field: %s', names{i});
    track.(names{i}) = double(v(:));
    assert(numel(v) == numel(track.s_m), 'Track field lengths differ.');
end
assert(all(diff(track.s_m) > 0), 'Distance breakpoints must increase.');
assert(track.s_m(1) == 0 && abs(track.s_m(end)-track.length_m) < 1e-8);
assert(track.closed && hypot(track.x_m(end)-track.x_m(1), ...
    track.y_m(end)-track.y_m(1)) < 1e-8);
end
