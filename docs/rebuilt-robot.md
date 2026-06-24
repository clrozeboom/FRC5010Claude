# Rebuilt 2026 Robot (`frc.robot.rebuilt`)

A port of Team 5010's **Rebuilt2026** competition robot onto the FRC5010Claude primitives.
It lives entirely under the team package `frc.robot.rebuilt` (subsystems, field model,
container, profile) and is the robot `frc.robot.RobotContainer` builds by default. Climb is
out of scope and omitted.

> Source of truth for parameters: `C:\workspace\Rebuilt2026` and
> `REIMPLEMENT_IN_FRC5010CLAUDE.md`. This doc covers what the port adds and the
> Rebuilt-specific gotchas — start here before touching the subsystems.

---

## Package layout

| File | Role |
|------|------|
| [`RebuiltRobot`](../src/main/java/frc/robot/rebuilt/RebuiltRobot.java) | Container — assembles subsystems, controls, couplings, autos |
| [`RebuiltRobotProfile`](../src/main/java/frc/robot/rebuilt/RebuiltRobotProfile.java) | Self-contained profile — drivetrain (`robot.json`) + 4 cameras |
| [`Constants`](../src/main/java/frc/robot/rebuilt/Constants.java) | Tuning constants (verbatim from source, minus climb) |
| [`FieldConstants`](../src/main/java/frc/robot/rebuilt/FieldConstants.java) | Field geometry keyed off `org.frc5010.common.vision.AprilTags` |
| [`HubTracker`](../src/main/java/frc/robot/rebuilt/HubTracker.java) | Hub "shift" schedule (pure time/strategy math) |
| `subsystems/RebuiltIntake` | Hopper `Arm` + spintakes + `IntakeState` machine |
| `subsystems/RebuiltIndexer` | Spindexer + transfer + `IndexerState` machine |
| `subsystems/RebuiltLauncher` | Flywheel + hood + turret + `LauncherState` machine |
| `subsystems/ShotCalculator` | Moving-shot solver (lookup tables + virtual-target lead) |
| `subsystems/HubStatus`, `subsystems/RebuiltLeds`, `subsystems/HopperArm` | Dashboard / LEDs / homing-aware Arm subclass |

Tests: `FieldModelTest`, `ShotCalculatorTest` (Layer 1, deterministic), `RebuiltRobotSmokeTest`
(Layer 3, full robot). All run under `.\gradlew.bat test`.

---

## The `track(...)` API (mechanism addition)

The Rebuilt launcher and intake re-aim mechanisms toward a **continuously changing setpoint
every cycle** (the turret/hood/flywheel chase a moving `ShotCalculator` solution; the hopper
follows a state-machine target). The mechanism classes' command factories —
`Arm.goToAngle`, `Pivot.goToAngle`, `Flywheel.goToSpeed` — return *never-ending commands that
require the subsystem*. Re-scheduling a fresh one every 20 ms to move the setpoint churns the
command lifecycle (each schedule interrupts the last) and is awkward to drive from another
subsystem's `periodic()`.

So the port added a small, purely-additive **non-command goal setter** to the mechanism
engine:

| Class | Method | Sets |
|-------|--------|------|
| `SingleDofMechanism` (base) | `protected void commandGoalNative(double targetNative)` | `mode = GOAL`, `goalNative` |
| `Arm` | `public void track(Angle angle)` | goal = `angle` |
| `Pivot` | `public void track(Angle angle)` | goal = `angle` |
| `Flywheel` | `public void track(AngularVelocity speed)` | goal = `speed` |

`track()` sets the goal **directly from a periodic loop, with no command and no requirement** —
the mechanism's own `periodic()` then acts on `mode == GOAL` exactly as it would for a
`goToAngle`/`goToSpeed` command (trapezoid profile + LQR, or onboard MotionMagic).

### Usage contract

```java
// In the OWNING subsystem's periodic / default command — call it every cycle:
hood.track(Degrees.of(params.hoodAngleDegrees()));
flywheel.track(RadiansPerSecond.of(params.flywheelSpeed()));
```

(The turret is *not* a mechanism `track()` user — it has its own torque-current-FOC controller;
see [The turret](#the-turret--fast-foc-tracking) below.)

- **Call it every cycle.** The goal is *latched*, not a one-shot: if you stop calling `track()`
  the last goal persists until the robot disables or a command takes over the mechanism. To
  "stop", command a rest goal (e.g. `flywheel.track(RPM.of(0))`) — see `RebuiltLauncher.IDLE`.
- **The owner and the mechanism are separate subsystems.** `RebuiltLauncher` requires only
  itself; `hood`/`flywheel` are their own subsystems with no default command, so there's no
  requirement conflict — `track()` just writes their goal fields.
- **Disable-safe.** On disable the mechanism neutralizes; re-asserting `track()` each cycle
  re-arms it cleanly on the next enable.
- **Prefer the command factories** (`goToAngle`, `goToSpeed`) for one-shot moves bound to a
  button or sequenced in an auto — they own the requirement and read naturally in command
  compositions. Use `track()` only for a subsystem that re-asserts a moving setpoint itself.

`RebuiltIntake` drives the hopper the same way (`hopper.track(targetAngle)` from its state
machine), so the deploy/retract goal updates without re-scheduling a command each loop.

---

## Subsystems & state machines

State is modelled with WPILib `Trigger`s / per-cycle logic (the source's config-driven
`StateMachine` stack isn't in this framework).

- **Intake** — `IntakeState`: `UNKNOWN, RETRACTED, RETRACTING, DEPLOYING, INTAKING, DEPLOYED,
  ANGLED`. Hopper `Arm` (CAN 15 lead / 14 follower, 24:1) + logged spintake duty. The auto-rezero
  logic (stall current + angle thresholds) is ported. **Deploy/retract complete on a settle
  timer** — see gotchas.
- **Indexer** — `IndexerState`: `IDLE, CHURN, HARD_CHURN, FORCE, FEED`. Duty-cycle spindexer +
  transfer (logged). CHURN is gated on `launcher.isFlywheelReadyForChurn()`.
- **Launcher** — `LauncherState`: `IDLE, LOW_SPEED, PREP, HAMMERTIME, PRESET` (initial
  `HAMMERTIME`). Flywheel (CAN 16/17, PROFILED), hood `Arm` (CAN 19, legacy-angle offset applied),
  and the `SmartTurretController` turret (CAN 18, CANivore). Aims from `ShotCalculator`. The
  trench-duck states from the source are dropped — the robot falls back to `LOW_SPEED` when not
  shooting.

### Couplings (wired in `RebuiltRobot.updateCouplings`, runs every cycle, `ignoringDisable`)

- Launcher `PREP` **and** `isReadyToFire()` → indexer `FEED`; `PREP` **and** spinning up → `CHURN`.
- `intake.isCollecting()` ⇄ start/stop the IronMaple Fuel game-piece sim.
- `isReadyToFire()` **and** indexer feeding **and** held Fuel > 0 → fire one Fuel (rate-limited).
- `launcher.setTurretBlocked(intake::isBlockingTurret)` — the hopper arm blocks the turret while
  retracting/retracted, forcing the launcher to `HAMMERTIME` (you must deploy the intake before
  the launcher will `PREP`).

The actual Fuel collection + ballistic hub scoring (and the web/Field2d state) reuse the
sim-only `org.frc5010.examples.DemoIntake` game-piece layer, created in simulation only.

---

## ShotCalculator

Self-contained moving-shot solver. Holds three interpolating lookup tables keyed on
distance-to-target (m): **hood angle** (authored in *legacy* degrees, converted to the real hood
frame via `Constants.Launcher.offsetLegacyHoodAngleDegrees`), **flywheel speed**, and
**time-of-flight** — the default tables are copied verbatim from the source. Per shot it
phase-delays the pose (linear field-frame extrapolation) and solves a **virtual target** by
fixed-point iteration (lead the target by `velocity × TOF`), respecting the turret's ±165°
travel. `flywheelMultiplier` defaults to 1.05 (operator-adjustable ±0.01).

`ShotCalculator` condenses the source's 543-line bisection-based `TurretControlPhysics`
moving-target *geometry* into a fixed-point virtual-target lead, and supplies the turret with a
**velocity feedforward** (`turretVelocityRadPerSec`) so the controller below can track it.

---

## The turret — fast FOC tracking

[`SmartTurretController`](../src/main/java/frc/robot/rebuilt/subsystems/SmartTurretController.java)
is a faithful port of the source's 2-state **torque-current-FOC** turret loop, built for fast
tracking of a moving shot. It deliberately goes **outside** the framework's synchronous-mechanism
convention (the `org.frc5010.common.mechanisms` classes run in `periodic()` with no Notifier) —
turret tracking needs the high-rate loop and direct torque control, and the robot has Phoenix Pro.

- **Owns its TalonFX** (CAN 18, CANivore), configured with two slots and torque-current limits;
  works in mechanism rotations (`SensorToMechanismRatio = gearRatio`).
- **SEEKING** (large error): `MotionMagicExpoTorqueCurrentFOC` (Slot 0) — smooth, physically
  optimal long travel from the motor model.
- **TRACKING** (small error): `PositionTorqueCurrentFOC` (Slot 1) with explicit **velocity
  feedforward** + position-error-directed `kS` — tracks a moving target with near-zero lag.
- **Hysteresis** SEEK→TRACK at 10°, TRACK→SEEK at 13°, so it doesn't chatter at the boundary.
- **200 Hz `Notifier`** runs `step(0.005)`; the 20 ms launcher loop calls
  `setTarget(angle, velocityRadPerSec, accelRadPerSecSq)` (from the `ShotCalculator` solution).
  The handoff is thread-safe via an `AtomicReference<TurretTarget>`.
- **Simulation**: `step()` also integrates a `DCMotorSim` and feeds the TalonFX sim state, so the
  plant advances at the 200 Hz control rate. Defaults (gains in **amps** for torque current):
  SEEK/TRACK `kP=225 / kD=50`, `kS=10 A`, `kA=5 A`, expo auto from 3 rot/s / 2.78 rot/s²,
  ±150° soft limits, 240 A peak.
- **Disable**: the launcher's `periodic()` (which always runs) calls `turret.disable()` while the
  robot is disabled — the default command stops updating the target, so the 200 Hz loop would
  otherwise keep driving the Talon.

`SmartTurretControllerTest` drives `step()` directly (no Notifier) and verifies it seeks + settles
into TRACKING, respects the soft limit, and tracks a 40°/s target with **< 6° lag**.

---

## Controls

The driver controller is shared by the keyboard/Xbox sim and the web UI; the six face buttons
**A/B/X/Y/LB/RB work over the web UI** (they map to `WebXboxController`). `Back`/`Start` and the
operator controller are sim/hardware only.

**Driver (port 0)**

| Button | Action |
|--------|--------|
| A | Intake: deploy + collect Fuel (`INTAKING`) |
| B (hold) | Launcher `PREP` (aim + spin, auto-feeds when ready); release → `LOW_SPEED` |
| X | Fire one Fuel at the hub (sim game-piece layer) |
| Y | Retract intake + indexer idle |
| LB (hold) | Indexer `HARD_CHURN` (un-jam); release → idle |
| RB | Launcher `HAMMERTIME` (safe stow) |
| Back / Start | Hopper deployed / retracted |

**Operator (port 1)**

| Button | Action |
|--------|--------|
| A / B / X / Y (hold) | Preset: tower / hub / hub / forward; release → `LOW_SPEED` |
| RB | Intake `ANGLED` (hold) / `INTAKING` (release) |
| LB | Indexer `FORCE` (hold) / `CHURN` (release) |
| Back / Start | Flywheel multiplier −0.01 / +0.01 |

---

## Autos

Registered in `RebuiltRobot.buildAutos`, selectable from SmartDashboard and the web UI.

**Simple routines** (no path following): **Do Nothing**, **Shoot Preload** (PREP → settle → the
coupling feeds the preload), **Drive Out + Shoot Preload**. These call the same launcher/indexer
building blocks the source's `NamedCommandsReg` registers.

**Path-following routines** (ported from the source PathPlanner autos to BLine), each the verbatim
step sequence of the matching source `.auto` file (path follows + launcher/indexer state requests
+ waits), assembled in
[RebuiltAutoRoutines.java](../src/main/java/frc/robot/rebuilt/RebuiltAutoRoutines.java):

| Family | Routines |
|---|---|
| Orbit | Left, Right, Left 1 Swipe, Right 1 Swipe, Right 2 Swipe (no HP), Churn Right 2 Swipe (no HP) |
| Follow | Left Bump Depot, Left Trench, Right Bump HP, Right Trench HP |
| Left | 2056 Double HP, 5010 Double, 3 Shuttle HPC |
| Right | 2056 Double HP, 5010 Double, 5010 Double (Old / Optimized / Short), 3 Shuttle HPC |
| Other | Delay Trench Neutral Bump HP, Disrupt, Quals 110, Quals 73 |

- **Paths** come from the source PathPlanner files, copied into
  `src/main/deploy/pathplanner/paths/` (46 `.path` files) and converted to BLine at load time by
  `PathPlannerToBLine` (Bézier-sampled — see [docs/auto.md](auto.md#importing-pathplanner-paths-pathplannertobline)).
- **Intake collection happens via the paths' embedded `intakeIntake` event markers** (plus the
  `intakeIntake` named step some autos run before driving), not extra auto logic —
  `RebuiltAutoRoutines.registerEvents()` binds those markers (and `launcherLow` / `launcherPrep` /
  `indexerChurn`) to the ported subsystem commands through `FollowPath.registerEventTrigger`.
  Firing the preload/collected Fuel is handled by the same always-running coupling loop as teleop,
  so a `launcherPrep` step spins up, aims, and (in sim) scores with no auto-specific firing logic.
  `Disrupt` also uses `waitUntilIntaking` (blocks ≤ 3 s until the intake reaches INTAKING).
- The first path of each auto re-anchors odometry to its start; continuation paths (which are
  linked end-to-start) do not. For autos that open with `wait`/`intake` steps before the first
  path, the odometry re-anchor therefore happens when that first path runs. All paths are Blue-side
  and alliance-mirror via BLine's `withDefaultShouldFlip()`.
- **Following is tuned for these tight, curved paths.** `RebuiltAutoRoutines` converts at
  `samplesPerSegment = 4` and installs its own BLine global constraints (`0.45 m` handoff, `0.65×`
  cruise) instead of the library defaults. Dense Bézier sampling made BLine thread every vertex and
  loop/overshoot the sharp corners (a 5 m path weaving ~22 m); sparse sampling + a bigger handoff
  rounds the corners cleanly. See [docs/auto.md](auto.md#importing-pathplanner-paths-pathplannertobline)
  and the `tunedFollowingDoesNotLoop` / end-to-end regression tests.

Pinned by `OrbitAutoSimPhysicsTest` (Layer 3: a converted path drives the physics robot to its end
and its event marker fires) and `PathPlannerToBLineTest` (Layer 1: converter structure + every
deployed `.path` converts). The full PathPlanner characterization/SysId suite and the
experimental/duplicate competition autos were not ported.

---

## Vision — 4 cameras (`RebuiltRobotProfile.createVision`)

Three PhotonVision AprilTag cameras plus the QuestNav headset. **The AprilTag cameras are
pitched 30° UP** (the source robot mounts them 30° down). Robot frame, x forward / y left / z up,
metres:

| Camera | Backend | x, y, z | roll/pitch/yaw |
|--------|---------|---------|----------------|
| rear-prometheus | PhotonVision | −0.2961, −0.2212, 0.2368 | 0 / **+30** / 175 |
| right-raikou | PhotonVision | −0.2775, −0.3025, 0.2432 | 0 / **+30** / −81 |
| left-bagel | PhotonVision | −0.2778, 0.2975, 0.2368 | 0 / **+30** / 70 |
| QuestNav | QuestNav pose source | −0.1720, −0.2525, 0.2253 | 0 / 0 / 180 |

In simulation the PhotonVision cameras run camera sim; QuestNav is a no-op without hardware.

`RebuiltRobotProfile.getAprilTagFieldLayout()` declares the **AndyMark** variant of the 2026
field (the one the team competes on). The container publishes it to the shared `AprilTags`
holder before building vision, so pose estimation **and** `FieldConstants` geometry both use it
— see [docs/vision.md](vision.md) and [docs/robot-profiles.md](robot-profiles.md).

---

## Drivetrain (`RebuiltRobotProfile`, from `robot.json`)

Swerve, 22 in track / 22 in wheelbase, **0.1002156588 m wheel diameter**, 5.93 m/s physical max
speed, 1.0 drive gear ratio, Pigeon2 gyro. `createDrive()` builds IronMaple physics in
simulation; REAL mode is a hardware-wiring TODO (like `ExampleRobotProfile` — `SwerveFactory`
throws for `TALON_FX` in REAL by design).

---

## Rebuilt-specific sim gotchas

1. **Intake deploy/retract complete on a 0.6 s settle timer** (`HOPPER_SETTLE_SECONDS`), not on
   exact hopper-angle arrival. The real robot detects the deployed hard stop by stall current,
   which can't be faithfully simulated — the timer is the robust sim/fallback equivalent.
2. **Arm mechanisms that start at a non-zero angle and sweep across vertical (90°) stall.** The
   hopper travels 120° → 0°; the soft LQR/MotionMagic can't push the arm "over the top" against
   gravity, and the sim mis-reports position at a non-zero start (≈2× the angle). `ExampleProfiledArm`
   (start 0°) converges fine. This is why the intake leans on the settle timer rather than the
   exact-angle transition. The mechanism still visibly moves.
3. **Mechanism control style.** Hopper/hood/turret are PROFILED-PID (onboard MotionMagic, needs
   `kV`); flywheel is PROFILED velocity (`kV ≈ 2.1962`). For sim convergence, PROFILED arms need
   `kV` set or they barely move (see `docs/mechanisms.md`).
4. **Test isolation.** Constructing the full robot per test on the singleton `CommandScheduler`
   accumulates button bindings; a button left pressed makes the next test's freshly-bound
   `onTrue` initialise already-pressed (no rising edge → never fires). `RebuiltRobotSmokeTest`
   clears the button loop **and releases all controller buttons** in teardown. Full-robot
   PROFILED mechanisms move in tests with the `feedEnable` + synchronous `SimHooks.stepTiming`
   pump; LQR ones move with `stepTimingAsync` (see the test pumps).

---

## Run it

```powershell
.\gradlew.bat test                          # full regression (incl. the 3 rebuilt suites)
.\gradlew.bat simulateJava -PwebUI          # interactive web UI (drive, A=intake, X=fire, ...)
```

**Web UI verification** (the project's gotcha-11 standard — drive the real flow and poll
`/api/state`): enable, `POST /api/drive` to move, press **A** (intake deploys → `intakeExtended`),
press **X** (`heldFuel` drops, `scoredFuel` rises as Fuel scores in the hub). Close the sim
window before re-running `.\gradlew.bat test` — a live sim JVM holds `build/jni/release/wpiHal.dll`
and the native-extract step will fail with a file-lock error.
