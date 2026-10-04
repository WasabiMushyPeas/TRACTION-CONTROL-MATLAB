# Integrate the MIS GPS reference into the existing Simulink model

User objective: add this reconstructed track to the existing vehicle simulation. The input preprocessing is complete. Inspect the model and adapt the interface to its architecture, without asking the user to redo track reconstruction.

## First inspect

Read `README.md`, `mis_track.json` metadata and existing repository instructions. Identify the model entry point, initialization scripts, data dictionary/model workspace usage, vehicle coordinate convention, existing path reference interface, solver, sample times and units. Do not replace unrelated vehicle/controller logic.

Use `mis_track.mat` as the primary source; it contains `track`. Use the CSV or JSON only if the repository's importer requires it. Keep a portable repository-relative data path. Load into the workspace/data dictionary the model actually uses. Do not assume a script-local variable is visible to block parameter evaluation, and do not overwrite existing model initialization callbacks.

## Integration contract

Input: continuous reference-path progress `s` in meters. Output: reference x, y, heading, signed curvature, with optional flat z. For closed-loop path-following, use the model's existing progress estimate or a continuity-constrained projection of vehicle position onto the reference. Do not treat elapsed time as distance. Integrating forward speed is suitable for a nominal on-path reference generator but is not generally exact path progress for a vehicle with lateral/heading error.

1. Wrap progress using `s_wrapped = mod(s, track.length_m)`. Use mathematical modulo, including for negative progress.
2. For each needed signal, use a Simulink 1-D Lookup Table with **explicit breakpoints** `track.s_m`, corresponding table data, and **linear interpolation**. Keep double precision initially. Configure clipping as a defensive extrapolation policy, but wrap the distance input first.
3. Query x from `track.x_m`, y from `track.y_m`, curvature from `track.curvature_1pm`. All are equal-length double column vectors. The repeated geometry endpoint is intentional; its distance breakpoint is distinct and increasing.
4. Query `track.heading_sin` and `track.heading_cos` separately and compute `atan2(sinValue, cosValue)`. Do not directly interpolate a wrapped angle. For controller heading error, use `atan2(sin(psi_ref-psi_vehicle),cos(psi_ref-psi_vehicle))`.
5. If the vehicle model expects x north/y east or clockwise heading, perform one explicit coordinate/sign conversion consistently for position, heading and curvature. ENU is right-handed with z up; positive curvature is left turn. If the model uses body coordinates, transform the reference through its existing world-to-body pipeline.
6. Keep `speed_recorded_mps` as optional comparison telemetry. It is not an optimized or validated target speed profile. Generate any desired speed schedule separately using the model's own vehicle/tire assumptions. Do not convert GPS altitude to road grade or invent widths, road boundaries, banking, friction or grip.
7. Initialize reference progress at zero. If this scenario's initial vehicle pose should lie on the track, set it to `track.x_m(1)`, `track.y_m(1)`, `track.heading_rad(1)` through the model's established initialization mechanism, preserving other scenario configurations.

If the existing model consumes curvature-versus-distance only, integrate that interface directly. If it needs a different road format, derive it from these coordinates and document required user/model assumptions. OpenDRIVE geometry or visual road widths are not supplied because the source does not establish road boundaries.

## Verification to run in MATLAB/Simulink

- Run `track = load_mis_track(); test_mis_track();` and resolve any local compatibility issues.
- Compile/update the actual model and verify reference signal dimensions, types and units.
- Evaluate at s = 0, L/4, L-0.001, L, L+0.001 and -0.001. Check position, heading direction and curvature sign. Verify continuity at the seam using wrapped angular differences.
- Run the model's normal smoke test and an appropriate traversal scenario. A full-lap replay/reference sweep should follow the supplied preview in the recorded direction. Do not claim vehicle/controller stability from data checks alone.
- Ensure no solver/algebraic-loop regression and no hidden data dependencies. Use the project's normal code-generation tests only if code generation is part of its requirements.
- Report edited files, how the reference connects, scenario assumptions, commands used and actual test results.

The package's Python-side numerical validation passed. MATLAB helpers were not executed by the package author because MATLAB/Simulink was unavailable. Actual integration and model execution are your remaining work.
