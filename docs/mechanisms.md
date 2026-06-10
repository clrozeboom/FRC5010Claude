# Mechanisms — YAMS + LQR

This library wraps [YAMS](https://github.com/Yet-Another-Software-Suite/YAMS)
("Yet Another Mechanism System", vendordep `yams.json`, version 2026.4.10.3) to give
teams declarative, simulation-ready mechanism subsystems with **state-space LQR
control** and live NetworkTables tuning.

## Architecture

```
Settings (public fields — robot-specific numbers ONLY, incl. controlStyle)
   │
   ▼
Common wrapper (org.frc5010.common.mechanisms)        Controller (ControlStyle.LQR default)
 ├── YamsElevator        ─ ELEVATOR-type LQR (meters) + trapezoid profile + kG FF
 ├── YamsArm             ─ ARM-type LQR (rotations) + trapezoid profile + kG·cos(θ) FF
 ├── YamsPivot           ─ ARM-type LQR (no gravity FF) — turrets, hoods, wrists
 ├── YamsFlywheel        ─ FLYWHEEL-type LQR (velocity, plant-inversion FF built in)
 │     └── ...or ControlStyle.PROFILED_PID on any of the four above:
 │         trapezoid profile + kP/kI/kD + kS/kV/kG FF (TalonFX: onboard MotionMagic /
 │         VelocityVoltage; gains in mechanism rotations)
 ├── YamsDoubleJointedArm ─ profiled PID per joint (LQR doesn't model coupled joints)
 └── YamsDifferentialMechanism ─ profiled PID per motor (tilt + twist)
   │
   ▼
YAMS mechanism (Elevator / Arm / Pivot / FlyWheel / DoubleJointedArm / DifferentialMechanism)
 ├── SmartMotorController wrapper (TalonFX preferred; TalonFXS / SparkMax / SparkFlex via MechanismMotor.Vendor)
 ├── RIO-side closed loop in a 20 ms WPILib Notifier (LQR always runs on the RIO)
 ├── @AutoLog inputs (AdvantageKit replay bubble) — see "AdvantageKit integration" below
 └── Built-in physics sim (ElevatorSim / SingleJointedArmSim / DCMotorSim) + Mechanism2d
```

**Common vs robot-specific:** all control logic, LQR construction, and tuning plumbing
live in `org.frc5010.common.mechanisms`. Team code only fills in a `Settings` object —
see the examples in `src/main/java/frc/robot/mechanisms/`, all Kraken X60 on TalonFX:

| LQR style (CAN 21–28) | Profiled-PID style (CAN 31–34) |
|---|---|
| `ExampleElevator` | `ExampleProfiledElevator` |
| `ExampleArm` | `ExampleProfiledArm` |
| `ExampleTurret` | `ExampleProfiledTurret` |
| `ExampleShooter` | `ExampleProfiledShooter` |
| `ExampleDoubleJointedArm` (profiled PID — only style) | |
| `ExampleDifferentialWrist` (profiled PID — only style) | |

## Choosing a control style

Both styles share the same settings, commands, telemetry, profile limits, and tests —
switching is `s.controlStyle = ControlStyle.PROFILED_PID;` plus gains.

- **LQR** (default): gains computed from the plant model (motor, gearing, mass/MOI).
  Tune physical tolerances, not gains — but the model must be accurate, and the loop
  always runs on the RIO in the YAMS Notifier.
- **PROFILED_PID**: classic trapezoid profile + kP/kI/kD with kS/kV/kG feedforward.
  On TalonFX everything runs *onboard* (MotionMagic for position, VelocityVoltage for
  flywheels; YAMS maps `ElevatorFeedforward`/`ArmFeedforward`/`SimpleMotorFeedforward`
  into Slot0 kS/kV/kG with the right GravityType). Gains are in **mechanism rotations**
  (kP = volts per rotation of error), even for elevators. Simpler to reason about,
  tolerant of model error, 1 kHz onboard execution — but hand-tuned.
  For flywheels in this style, kV does most of the work (≈ 12 V ÷ free speed in rot/s).

## Adding a mechanism (short version — see `/new-yams-mechanism` for the playbook)

```java
public class MyElevator extends YamsElevator {
  public MyElevator() { super(settings()); }
  private static Settings settings() {
    var s = new Settings();
    s.name = "Elevator";
    s.canId = 21;                                   // TalonFX CAN ID
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {4, 3};    // 12:1
    s.drumCircumference = Inches.of(0.25 * 22);     // travel per drum rotation
    s.carriageMass = Kilograms.of(6.0);
    s.minHeight = Meters.of(0);  s.maxHeight = Meters.of(1.5);
    s.maxVelocity = MetersPerSecond.of(0.9);        // MUST be physically achievable!
    s.kG = Volts.of(0.19);                          // from sysId() on a real robot
    return s;
  }
}
```

Commands: `goToHeight(Distance)` / `goToAngle(Angle)` / `goToSpeed(AngularVelocity)`,
`setDutyCycle(...)`, `sysId()`; triggers: `isAtHeight` / `isAtAngle` / `isAtSpeed`;
mechanism access: `getMechanism()` / `getMotor()`.

## Why LQR (and what to know)

The YAMS docs and examples mostly show plain/profiled **PID** — those examples are out
of date relative to what the library can do. YAMS ships `yams.math.LQRController`
(WPILib `LinearSystemLoop` = LQR gain + Kalman filter + plant-inversion feedforward),
plugged in via `SmartMotorControllerConfig.withClosedLoopController(LQRController)`.
LQR is tuned with *physical tolerances*, not abstract gains:

| Weight | Meaning | Default |
|---|---|---|
| `qelmsPosition` | position error you tolerate (m or rot). Smaller = more aggressive | 2 in / 1.5° |
| `qelmsVelocity` | velocity error you tolerate (m/s or rot/s) | 0.5 m/s / 20°/s |
| `relms` | control effort you allow (volts). Smaller = gentler | 12 V |

All three are live-tunable under `/Tuning/<name>/lqr_*` (AdvantageScope / Shuffleboard).
On change, the wrapper rebuilds the regulator and restarts the loop at the current
state. In PROFILED_PID style the tunables are `/Tuning/<name>/pid_kP|kI|kD` instead
(re-applied to the motor on change — lands onboard for TalonFX). See `/tune-mechanism`
for the full tuning workflow.

LQR supports exactly three plants — **ELEVATOR**, **ARM** (also used for pivots), and
**FLYWHEEL**. DoubleJointedArm and DifferentialMechanism are coupled multi-motor
systems the LQR types don't model, so those wrappers use profiled PID with
`TunableGains` (`/Tuning/<name>/<joint>_k*`).

## Characterized plants — when you can't (or shouldn't) trust mass/MOI

**The problem:** an LQR computes its gains from a model of the mechanism (the
"plant"). By default that plant is built on paper: motor datasheet + gearing +
mass/MOI. But carriage mass is hard to measure, CAD always misses something, and the
paper model assumes a lossless gearbox. A wrong plant means the regulator confidently
computes the *optimal gains for a mechanism you don't have*.

**The insight students should internalize:** the controller never needed mass — it
needs to know *how the mechanism responds to voltage*. Mass is just one way to predict
that. A SysId test **measures** it instead, as two numbers:

| Value | Physical meaning | Question it answers |
|---|---|---|
| **kV** | volts per unit velocity | "how much voltage to hold a steady speed?" |
| **kA** | volts per unit acceleration | "how much *extra* voltage to speed up?" |

kA is where the mass/inertia "lives": a heavier carriage needs more voltage to
accelerate, so it shows up as a bigger kA. Friction and gear losses show up too —
things no spreadsheet model includes. Set `characterizedKv`/`characterizedKa` in the
settings and the wrapper builds the LQR plant with WPILib's
`LinearSystemId.identifyPositionSystem` (or `identifyVelocitySystem` for flywheels)
instead of the mass-based model. See `ExampleCharacterizedElevator` (CAN 35) for a
fully-commented walkthrough; the functional test
`characterizedPlantElevatorConverges` proves the path works end to end.

**Procedure:**
1. Run the wrapper's `sysId()` command on the real mechanism (tethered, clear travel).
2. Open the log in the WPILib SysId tool. Units: **meters** for elevators,
   **rotations** for arms/pivots/flywheels (the settings expect those units; the
   wrapper converts to the plant's radians internally).
3. Copy kV and kA into `characterizedKv` / `characterizedKa`. Keep kG from the same
   run — gravity is a constant force, not part of the linear plant, so it stays a
   feedforward either way.
4. Sanity-check kV: theory says kV = 12 V ÷ free speed (motor free speed ÷ gearing ×
   circumference). Measured kV slightly *higher* than theory = normal losses; much
   higher = a mechanical problem worth finding; *lower* = wrong units or gearing.

**Backing out the physical value** (optional — for the settings file or a CAD
cross-check) from kA, the motor constants (kT = stall torque ÷ stall current,
R = 12 V ÷ stall current), and gearing G:

- Elevator: `m = kA · G · kT / (R · r_drum)`  (kA in V/(m/s²))
- Arm / pivot / flywheel: `J = kA · G · kT / R`  (kA in V/(rad/s²))

In simulation the mechanism is lossless, so theoretical and "measured" values match —
the educational payoff comes on the real robot, where the difference between them *is*
the model error the characterized plant eliminates.

## Gotchas (hard-won)

1. **YAMS 2026.4.10.3 ships a broken Kalman filter for ARM/ELEVATOR LQR.**
   `LQRConfig.getKalmanFilter` passes WPILib's 2-output plant with a 1-dim measurement
   noise vector; the native DARE solver reads garbage ("R was not symmetric", or—worse—
   silently useless gains, mechanism barely moves). Fixed upstream on main but not
   released. `MechanismLqrConfig` overrides `getKalmanFilter` with the correct
   `plant.slice(0)`. **Always build LQR configs through `MechanismLqrConfig`**, never
   raw `LQRConfig`, or retuning will go through the broken path. FLYWHEEL (1-state) is
   unaffected.

2. **The LQR closed loop must run in the units of its plant.** The ELEVATOR plant is in
   meters: `withMechanismCircumference(...)` + a linear trapezoid profile (or
   `ElevatorFeedforward`) put the YAMS loop in linear mode. Without it, the loop feeds
   rotations into a meters model and slams the mechanism.

3. **`withClosedLoopController(lqr)` must be LAST in the config chain.** Every
   PID-flavored `withClosedLoopController(...)` overload clears the stored LQR.

4. **Profile cruise velocity must be physically achievable** (below
   free speed ÷ gearing × circumference). The profile doesn't know about saturation; if
   it outruns the mechanism the LQR chases an unreachable reference and overshoots
   badly. Same for arms/pivots in rotational units.

5. **kG feedforward is required for elevators/arms.** The linearized LQR plants have no
   gravity term and LQR has no integrator, so uncompensated gravity = steady-state
   error. Get kG from `sysId()` on the real robot (the examples compute it from motor
   constants for sim). YAMS only applies `ArmFeedforward` when a motion profile is
   configured — keep the trapezoid profile on arms.

6. **No `withClosedLoopTolerance` with LQR** — YAMS throws by design.

7. **Tests: never use synchronous `SimHooks.stepTiming` with YAMS mechanisms.** The
   YAMS closed loop lives in a Notifier; synchronous stepping deadlocks waiting for an
   alarm ack. Pump with `stepTimingAsync(0.02)` + ~10 ms real sleep
   (see `YamsMechanismsFunctionalTest.runScheduledFor`).

8. **Tests: feed the Phoenix enable watchdog.** TalonFX outputs silently neutral (duty
   reads 0, no error) if fresh DS packets stop for ~100 ms of *real* time. Call
   `DriverStationSim.notifyNewData()` + `Unmanaged.feedEnable(...)` every test cycle.

9. **The released jar's API differs from YAMS GitHub main** (e.g. `withSoftLimit` vs
   `withSoftLimits`, `ArmConfig.withHardLimit` vs `withHardLimits`). When in doubt,
   `javap` the jar in the Gradle cache, not the GitHub source.

## AdvantageKit integration

Each wrapper follows the YAMS AdvantageKit pattern (the upstream
`examples/advantage_kit` project): everything read back from the motor/mechanism
crosses the replay bubble through an `@AutoLog` inputs class.

- **Inputs** (`<Mechanism>Inputs`, nested in each wrapper): position, velocity,
  closed-loop setpoint, applied volts, stator current — per motor for the dual-motor
  mechanisms. Fields are `double` with unit-suffixed names (`positionMeters`,
  `velocityRPM`, ...) per this repo's convention — never `Measure<>` in `@AutoLog`
  (CLAUDE.md gotcha #9; the upstream example uses `Distance` fields, don't copy that).
- **`periodic()`** runs `updateInputs()` → `Logger.processInputs(name, inputs)` before
  tuning and YAMS telemetry, so every cycle's state lands in the `.wpilog`.
- **Getters are replay-safe**: `getHeight()`, `getAngle()`, `getSpeed()`, `getTilt()`,
  etc. read from the *inputs* object, not the hardware — in REPLAY mode they return
  the logged values. The `isAt...` triggers are built on the inputs too. Anything that
  must survive replay should go through these getters; `getMechanism()` bypasses the
  bubble (live sim/hardware only).
- **Commands record their targets** as outputs (`<name>/CommandedHeightMeters`, ...),
  so commanded vs. actual is directly plottable in AdvantageScope.

After changing what the mechanisms log, run `/validate-replay` to confirm replay
fidelity end-to-end.

## Try them in the sim

`RealRobot` instantiates every example mechanism in simulation (sim only — the CAN
IDs don't exist on a real robot) and binds the **X button** to drive them all to a
mid-travel point in parallel (elevators → 0.75 m, arms/turrets → 90°, shooters →
3000 RPM, DJA → 90°/0°, wrist → 45°/30°). Run `./gradlew simulateJava`, enable,
press X, and watch the Mechanism2d widgets under SmartDashboard →
`<name>/mechanism`. Tests that construct `RobotContainer` must call
`RealRobot.closeDemoMechanisms()` in teardown and pump time asynchronously
(see gotcha 7) — `RobotContainerSmokeTest.pumpCycles` is the reference.

## Functional tests

`src/test/java/frc/robot/mechanisms/YamsMechanismsFunctionalTest.java` builds each
example subsystem for real (TalonFX wrapper + YAMS sim + closed loop), schedules its
public command, and asserts the mechanism reaches the commanded state — covering both
control styles (LQR and profiled PID) plus one live-retune-over-NT test. They run in
the normal `./gradlew test` suite.

## Real-robot bring-up

1. Copy an example, set real CAN IDs / gearing / masses / limits.
2. Verify in sim (`./gradlew test`, or `simulateJava` and watch the Mechanism2d widget
   under SmartDashboard → `<name>/mechanism`).
3. On the robot: run `sysId()` to characterize kG, kV, and kA; set `kG` and (for LQR
   style) `characterizedKv`/`characterizedKa` so the plant matches the real mechanism
   (see "Characterized plants" above).
4. Tune qelms/relms live over NT (`/tune-mechanism`), then bake the final values into
   the settings.
