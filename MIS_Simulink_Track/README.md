# MIS GPS track for MATLAB / Simulink

This package provides a closed, smooth **reference driving path**, reconstructed from the supplied June 20, 2026 MoTeC GPS trace. It is ready for model integration, but is not a surveyed track centerline or a complete road surface. The existing Simulink model was not provided and has not been modified.

## Start here

1. Extract this folder.
2. Give the whole folder to Claude Code and ask it to follow `CLAUDE_HANDOFF.md` in your model repository.
3. In MATLAB, add this folder to the path and run:

```matlab
track = load_mis_track();
test_mis_track();
ref = sample_mis_track(track, 125); % 125 m along the track
```

`mis_track.mat` is a standard compressed MAT v5 file with one `track` struct and double-precision column vectors. No Mapping Toolbox or Automated Driving Toolbox is required to load or query it. JSON and CSV contain the same numeric samples for non-MATLAB tooling.

## Files

| File | Purpose |
|---|---|
| `mis_track.mat` | Primary MATLAB data: `track` struct with vectors and metadata |
| `mis_track.csv` | Same sampled fields with units in column names |
| `mis_track.json` | Same sampled fields plus conventions, provenance, and validation |
| `load_mis_track.m` | Load data and validate its structure |
| `sample_mis_track.m` | Periodic distance lookup example |
| `test_mis_track.m` | MATLAB smoke tests and track plot |
| `CLAUDE_HANDOFF.md` | Instructions for integrating with the actual model |
| `track_preview.png` | Geometry, signed curvature, recorded speed |
| `validation.json` | Build-time numerical validation summary |
| `source_gps.csv` | Original reduced GPS trace, all 35,736 samples |
| `build_track.py` | Reproducible processing, using NumPy, SciPy, Matplotlib |

## Geometry and conventions

- Modeled lap length: **994.416 m**. Treat this as approximately 994 m, not survey-level precision.
- **1,990 points**, including a repeated endpoint at the lap length. Spacing is **0.499958 m**, constant and at most 0.5 m.
- `s_m` is horizontal arc length along the smooth reference, starting at zero. It is not time.
- Local ENU: x east, y north, z up, meters. WGS84 origin: latitude 42.0685345 degrees, longitude -84.2369143 degrees. Horizontal projection uses constant ellipsoid height zero; this is not an assertion of site elevation.
- Start/finish is defined by the first recorded location and initial travel direction, not an independently verified official timing line. The fitted first point may be slightly offset from the ENU origin due to seam blending and smoothing.
- Heading is radians counterclockwise from east: 0 east, pi/2 north. `heading_rad` is continuous/unwrapped and gains 2*pi over the lap. `heading_cos` and `heading_sin` are periodic.
- Curvature is signed 1/m, positive left turns in the recorded travel direction.
- `z_m = 0` is an explicit flat-road assumption. GPS altitude is preserved separately, with unknown vertical datum. Width, road edges, banking, grip and road grade are not inferred.

## Field dictionary

All sampled fields have the same length and are indexed by `s_m`.

| Field | Meaning |
|---|---|
| `s_m` | Increasing distance breakpoints, 0 through lap length |
| `x_m`, `y_m`, `z_m` | Reference position, meters; z is assumed flat |
| `heading_rad` | Unwrapped geometric tangent heading, radians |
| `heading_cos`, `heading_sin` | Periodic heading components for safe lookup across the seam |
| `curvature_1pm` | Signed planar curvature, 1/m |
| `speed_recorded_mps` | Recorded GPS speed mapped onto the processed path; not a target |
| `gps_altitude_recorded_m` | Recorded GPS altitude, not trusted road elevation |
| `time_recorded_s` | Time from start of source selection, mapped onto the processed path |
| `gps_hdop` | Dimensionless GPS geometry dilution; not accuracy in meters |
| `length_m` | Scalar total processed lap length |
| `closed` | True: periodic path |
| `meta` | Units/conventions, method settings, provenance and checks |

The recorded telemetry fields need not match at the seam. They describe this particular pass, not a periodic steady-state simulation. Do not use these fields as authoritative speed, altitude, or lap-time constraints after path smoothing.

## Processing and limits

1. Convert geographic coordinates through WGS84 ECEF to a local tangent ENU plane. Do not mix GPS heading (clockwise from north) with the generated tangent heading.
2. Use the first 0.5 s of displacement to define a transverse start gate. The final forward crossing occurs at 71.31294 s. Remove the final 0.15706 s of overlap. The transverse separation at this crossing is 0.968 m; the original final point is 1.683 m from the first.
3. Resample the observed path at about 0.5 m and blend the gate endpoints to their midpoint over the first/last 10 m, using cosine weights.
4. Apply periodic Gaussian smoothing with spatial sigma 2 m. This is an explicit modeling setting, not measured GPS uncertainty. The 500 Hz CSV export contains densely interpolated positions, not proof of 500 Hz independent GPS fixes.
5. Fit a periodic cubic spline. Numerically integrate its speed on a 0.02 m parameter grid and resample by actual smooth arc length. Derive tangent and curvature analytically from spline derivatives.

Curvature = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2). Distance = integral sqrt(x'^2 + y'^2) du.

The raw planar path is 1,010.518 m. Closure trimming and smoothing reduce the reference to 994.416 m. Fit displacement measured at corresponding path parameters is 0.102 m RMS and 0.719 m maximum. Those numbers measure processing displacement, not absolute GPS accuracy. Peak absolute curvature is 0.19353 1/m. Curvature is particularly sensitive to GPS error and smoothing, so validate against track knowledge before vehicle-limit conclusions. This trace alone cannot prove the official layout or driving corridor.

Build checks passed: finite arrays, increasing breakpoints, equal vector lengths, exact geometry closure, periodic spline first/second derivatives, MAT round-trip equivalence, heading winding, integrated curvature (~2*pi), maximum sample spacing and bounded fitting displacement. MATLAB and Simulink are not installed in the build environment, so the supplied `.m` files and actual model integration still require execution there.

Rebuild with Python 3, NumPy 2+, SciPy and Matplotlib:

```sh
python build_track.py --source source_gps.csv --out rebuilt
```

The rebuild generates data, provenance and preview files. It does not regenerate the hand-written MATLAB helpers or instructions.

## API references

- MathWorks 1-D Lookup Table: https://www.mathworks.com/help/simulink/slref/1dlookuptable.html
- SciPy MAT file writer: https://docs.scipy.org/doc/scipy/reference/generated/scipy.io.savemat.html
- SciPy periodic cubic spline: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
