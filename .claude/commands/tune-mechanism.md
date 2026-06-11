# /tune-mechanism — Tune a YAMS mechanism (LQR or PID)

Agent playbook for tuning the mechanism subsystems built on
`org.frc5010.common.mechanisms` (see [docs/mechanisms.md](../../docs/mechanisms.md)).

## Identify the controller first

- `YamsElevator` / `YamsArm` / `YamsPivot` / `YamsFlywheel` with
  `ControlStyle.LQR` (default) → **LQR** — tune tolerances under `/Tuning/<name>/lqr_*`.
- The same four with `ControlStyle.PROFILED_PID` → **profiled PID** — tune gains under
  `/Tuning/<name>/pid_kP|kI|kD`. Gains are in **mechanism rotations** on TalonFX
  (onboard MotionMagic / VelocityVoltage): kP = volts per rotation of error, kV = volts
  per rotation/s (theoretical kV = 12 ÷ free speed in rot/s — flywheels need it set).
- `YamsDoubleJointedArm` / `YamsDifferentialMechanism` → **profiled PID** —
  tune gains under `/Tuning/<name>/*_kP|kI|kD`.

## LQR tuning workflow

LQR has no kP/kI/kD. You tune three *physical tolerances*; the regulator computes
optimal gains from the plant model (motor, gearing, mass/MOI):

| NT entry | Meaning | Move it when... |
|---|---|---|
| `lqr_qelmsPosition` | allowed position error (m or rotations) | too sluggish → smaller; oscillates → larger |
| `lqr_qelmsVelocity` | allowed velocity error (m/s or rot/s) | overshoot/ringing → smaller (more damping) |
| `lqr_relms` | allowed control effort, volts | violent/brownouts → smaller; weak → keep 12 |

Procedure (sim: `./gradlew simulateJava`; real robot: tethered, mechanism clear):
1. Open AdvantageScope/Shuffleboard → `/Tuning/<name>/`. Changes apply live — the
   wrapper rebuilds the regulator and restarts the loop at the current state.
2. Command a mid-range setpoint, observe the YAMS telemetry (position vs setpoint).
3. Adjust one weight at a time, factor-of-2 steps. Don't go below ~1 in / ~1° position
   tolerance: the RIO loop has 20–40 ms delay and tighter weights oscillate.
4. If it *never* settles at the target (steady offset): that's gravity, not weights —
   fix `kG` (step below), LQR has no integrator.
5. **Bake the final values into the `Settings`** in the team's subsystem class —
   NT tunables reset on reboot.

### Plant accuracy beats weight tuning
The LQR is only as good as its model. If behavior is wildly off, re-check settings:
gearing stages, carriage mass / arm length+mass / MOI, drum circumference. A 2x mass
error degrades control more than any weight change can fix. If the mass/MOI can't be
measured (or you've checked everything and it's still off), characterize the plant
from a SysId run instead — see the next section; the measured kV/kA make mass
irrelevant to the controller.

### SysId characterization (real robot) — kG AND the plant itself
1. Run the wrapper's `sysId()` command (quasistatic+dynamic, logs via WPILib SysId).
2. Load the log in the SysId tool (units: meters for elevators, rotations otherwise)
   → read kG, kV, kA.
3. Set `settings.kG` (gravity stays a feedforward — it's not part of the linear plant).
4. **Set `settings.characterizedKv` / `characterizedKa`** — this replaces the
   mass/MOI-based LQR plant with one identified from the real mechanism's measured
   response, capturing friction, gear losses, and the true inertia. This is the answer
   to "we don't know the carriage mass": you don't need it — kA implies it
   (`m = kA·G·kT/(R·r)`, `J = kA·G·kT/R`). Full explanation + sanity checks in
   docs/mechanisms.md → "Characterized plants".
   (Do NOT also add kV as a feedforward in LQR style — the loop provides
   plant-inversion feedforward itself.)

### Profile limits
`maxVelocity` must stay below free speed ÷ gearing × circumference (or the rotational
equivalent). If the mechanism saturates at 12 V chasing the profile, lower
`maxVelocity`/`maxAcceleration` — the symptom is big overshoot at the end of motion
with the LQR weights blameless.

## Profiled PID tuning (DJA / differential)

`/Tuning/<name>/lowerJoint_kP` etc. apply live (config re-applied to the motor on
change — lands onboard for TalonFX). Standard PID workflow: raise kP until slight
overshoot, add kD to damp; kI almost never needed with the trapezoid profile. Bake
final gains into `Settings`.

## Validate

`./gradlew test --tests "frc.robot.mechanisms.YamsMechanismsFunctionalTest"` must stay
green with the baked-in values. If tuning sessions changed defaults, update the
matching tolerance/time budget in the test rather than deleting assertions.
