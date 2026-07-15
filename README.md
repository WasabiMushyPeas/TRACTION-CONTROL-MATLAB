# CP27E 4-Wheel Hub-Motor Traction Control (Simulink)

Per-wheel traction control for the four AMK DD5-14-10-POW hub motors. Built on
the old launch-control physics, but generalized: this is **not just a launch
sim** — it estimates the **maximum longitudinal force / motor torque each tire
can take in any state of the car** (launch, combined-slip cornering) and holds
the wheel there. Launch is the biggest single case, not the only one.

Same layout convention as the TV sim: **every Simulink MATLAB Function block is
a thin wrapper over one `.m` file** here. Flat directory. Run `run_sim`.

---

## What it computes

The core output (per wheel, every step) is the **traction ceiling**:

```
longitudinalForceCapacity = availableFriction * sqrt(1 - lateralGripFraction^2) * Fz
maxDriveTorque = longitudinalForceCapacity * wheelRadius / (gearRatio*drivetrainEfficiency)
```

`lateralGripFraction = |ay| / (g*availableFriction)` is how much of the friction circle the tire is already
spending on lateral grip. So in a corner the longitudinal ceiling drops — the
controller won't command drive torque the tire can't hold while it's also
turning. `grip_estimator.m` produces `maxDriveTorque`; `tc_control.m` saturates the
per-wheel command to that window. That ceiling is exactly the per-wheel
friction-circle constraint the TV allocator will consume later.

**Lateral is a prescribed scenario input (`ay`), not a solved yaw-plane state.**
The model answers "given this lateral usage, how much longitudinal torque can
each tire take" without dragging in a full vehicle/steering model. Swap
`maneuver.m` for a measured or track `ay`/torque profile when you couple it to
the TV plant.

---

## Why the old model oscillated (and what changed)

Fixed-gain PI at a 10 ms sample is fine at speed but violent at launch:

1. The slip plant is sub-millisecond fast at low speed → undersampled → limit
   cycle.
2. Plant gain blows up near the µ-slip peak (`dFx/dσ→0`, σ_pk≈0.147 next to the
   0.13 target).
3. Tire relaxation lag `τ=L/v` balloons at low v, adding phase lag.

Plus the 2WD driveline had a half-shaft torsion (`K`,`C`) resonance.

### Changes vs. old launch sim

| Old 2WD launch sim | New 4-wheel TC | Why |
|---|---|---|
| 2 driven wheels | 4 independent wheels, per-wheel slip + grip ceiling | hub motors, no diff/half-shaft |
| Launch only (straight, drive) | **launch + combined-slip corner** | "max grip in any state" |
| Longitudinal tire only | **friction-ellipse combined slip** (`ay` derates `Fx`) | corner capacity is the point |
| Cap = motor peak | **cap = grip ceiling** ∩ motor peak ∩ request | this is the deliverable |
| Half-shaft `K`,`C` torsion | rigid `combinedWheelInertia = rotorInertia*gearRatio^2 + wheelInertia` | kills driveline resonance |
| Fixed-gain PI | grip FF + speed-scheduled PI | FF carries launch, PI trims |
| Hard START/END_BLEND switch | smooth `sched` ramp `v_lo=2→v_hi=6` | no switching transient |
| Straight-line slip | **per-corner ground speed** from yaw (`r≈ay/vx`) | slip is right in a corner |
| Long. load transfer only | **+ lateral** load transfer from `ay` | inside wheels unload in a corner |
| int32 fixed-point in loop | floating-point design | remove quantization limit cycles |
| EMRAX 208, high peak torque | AMK DD5, `peakMotorTorque=21` Nm/motor | datasheet |

---

## Files

| file | role |
|---|---|
| `params.m` | struct `P`, `X0`, `I0`; `gripPreset` surface; maneuver timing; ellipse/corner-speed flags |
| `maneuver.m` | scenario `[torqueRequest, lateralAcceleration] = f(currentTime)`: launch → corner |
| `slip_estimator.m` | per-wheel slip; per-corner ground speed from `ay`; floored low-speed |
| `load_transfer.m` | per-wheel `normalLoad`: static + aero + longitudinal + **lateral** load transfer |
| `tire_long.m` | longitudinal Magic Formula with **combined-slip ellipse** derate |
| `grip_estimator.m` | **traction ceiling**: `longitudinalForceCapacity`, `maxDriveTorque`, `frictionUtilization`, `slipMargin` |
| `tc_control.m` | grip FF + speed-scheduled PI + back-calc anti-windup, saturated to the ceiling |
| `plant_long.m` | 13-state 4-wheel longitudinal plant (rigid hub, tire relaxation) |
| `build_cp27e_tc.m` | programmatic Simulink build (wrappers, integrators, wiring, solver) |
| `run_sim.m` | build if needed, `sim`, metrics, 2×3 full-envelope plots |

Signal convention: every per-wheel signal is **row 1×4**; only the plant state
`X` / `dX` are columns (13×1). Each function coerces its inputs at the top.

---

## Block topology

```
   Clock ─► maneuver ─┬─ Treq ─────────────────────────────► ctrl
                      └─ ay ─► slip, load, tire, grip, ctrl
        ┌──────────── int_plant (X, 13-state) ───────────────────────┐
        │                                                            │
  X ─► Demux[1 4 4 4] ─┬─ vx ─► slip ─┐                              │
                       ├─ w  ─►       ├─► slip ─┬─► tire ─► Fx_ss ────┤
                       ├─ Fxd─► load ─► Fz ─────┼─► grip ─┬─ frictionUtilization │
                       └─(Tmot,internal)        │         └─ maxDriveTorque──────┤►ctrl
  int_ctrl (I,4) ───────────────────────────────► ctrl ─► Tcmd ─► plant ─► dX ─► int_plant
                                          ctrl ─► dI  ─► int_ctrl
```

No algebraic loops: `dynamicLongitudinalForce` is a state (tire relaxation) so
slip→Fx→ω is broken; `longitudinalAcceleration` comes from that state;
`lateralAcceleration` is exogenous; the grip ceiling depends on
`normalLoad`/`lateralAcceleration`, not on `Tcmd`.

---

## Build / run

```matlab
>> run_sim
```

Prints launch time, peak `vx`/`ax`/`ay`, peak combined `frictionUtilization`, and draws six
panels: slip (±target), torque command with the drive grip ceiling, speed
& accelerations, per-wheel `normalLoad` (watch the lateral transfer in the corner),
combined friction utilization, and the scenario. Phase boundary is dotted.

Rebuild from scratch: `close_system('cp27e_tc',0); build_cp27e_tc;`

Edit the run in `params.m`: `gripPreset` (surface), `launchEndTime` (phase
time), `cornerLateralAcceleration` (corner severity), and the
`useFrictionEllipse`, `useCornerSpeedEstimate`, and `useGripFeedforward` flags.

---

## Open parameters — CONFIRM THESE

1. **Gear `P.gearRatio = 10`** — representative AMK DD5 FSE reduction (published
   ~13–16:1; 13.2 lap-sim-optimized; AMK kit box ~14.4). Regime depends on tire
   µ: grip-limited on **drive** for realistic µ≈1.5 (breaks ~11:1); with the
   high placeholder tire (µ≈2.9) dry drive is still torque-limited (needs ~21:1).
   **Cornering hits the ceiling regardless**, so `gripPreset='dry'` still
   exercises TC. Confirm the exact CP27E value.
2. **Peak torque = 21 Nm/motor** (datasheet Mmax). The 120 Nm in the old notes
   is not the AMK shaft torque.
3. **Tire `P.tirePeakMuBase = 3.02`** -> peak mu around 2.8. Placeholder pending the TTC fit;
   `tireGripScaleByWheel`/`gripPreset` dial grip meanwhile.
4. **`P.trackWidth = 1.22` track** and **`cornerLateralAcceleration = 12` m/s²** — set to real CP27E
   values; they set lateral transfer and the ellipse derate magnitude.

---

## Next steps

- **Merge with the TV allocator** — feed `maxDriveTorque` straight into `allocate.m`
  as a per-wheel friction-circle constraint (QP/WLS) instead of a separate
  loop. The ceiling is already in the right form.
- **Solve the lateral plant** — replace prescribed `ay` with a real yaw-plane
  model (slip angles, per-axle `Fy`), so the ellipse uses actual per-wheel
  lateral force instead of the uniform-utilization proxy `Fy_i∝Fz_i`.
- **Roll-stiffness / ARB distribution** in `load_transfer` for the true
  front/rear lateral transfer split.
- **Online µ estimation** — reconstruct `Fx=(Tmot*gearRatio*drivetrainEfficiency - combinedWheelInertia*domega)/wheelRadius` on-car and
  identify µ from the slip-slope; needs true ground speed → **PAW3395 optical
  sensor** (your high-priority item).
- **Fixed-point firmware port** of `tc_control` to int32 (permille/permyriad),
  BW < ~1/5 sample rate.
- **Wheel-hop** from hub-motor unsprung mass vs. loop bandwidth (VDS data).
- **Half-shaft option** — if the AMK are inboard-per-wheel not true hubs, add a
  per-corner `K`,`C` torsion element in `plant_long` (4 extra states).
