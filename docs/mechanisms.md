# Mechanisms — TalonFX-native, LQR-first

Declarative, simulation-ready mechanism subsystems built directly on Phoenix 6
TalonFX — no third-party mechanism library. Teams fill in a `Settings` object and get
a subsystem with **state-space LQR or profiled-PID control**, AdvantageKit replay, a
physics sim, a Mechanism2d view, SysId, and live NetworkTables tuning.

> This branch replaces the YAMS-based implementation: same Settings/commands/examples
> API, but TalonFX-only and self-contained. The closed loop runs synchronously in
> `periodic()` — no Notifier threads, no async test pumps, no published-jar bugs.

## Architecture

```
Settings (public fields — robot-specific numbers ONLY, incl. controlStyle)
   │
   ▼
Subsystem (org.frc5010.common.mechanisms)             Controller (ControlStyle.LQR default)
 ├── SingleDofMechanism (abstract) ─ shared engine: goal state machine, profile+LQR or
 │    MotionMagic dispatch, live tuning, enable/disable transitions, disconnect Alert
 ├── Elevator   ─ ELEVATOR plant (meters) + trapezoid profile + kG FF + current-spike homing
 ├── Arm        ─ ARM plant (radians) + trapezoid profile + kG·cos(θ) FF
 ├── Pivot      ─ ARM plant, no gravity FF — turrets, hoods, wrists
 ├── Flywheel   ─ velocity plant (plant-inversion FF built in)
 │     └── ...or ControlStyle.PROFILED_PID on any of the four above:
 │         onboard MotionMagic / VelocityVoltage at 1 kHz with Slot0 kP/kI/kD/kS/kV/kG
 │         (gains in mechanism rotations)
 ├── DoubleJointedArm      ─ onboard MotionMagic per joint (LQR doesn't model coupled joints)
 └── DifferentialMechanism ─ onboard MotionMagic per motor (tilt = avg, twist = diff)
   │
   ▼
MechanismIO (@AutoLog inputs — the AdvantageKit replay bubble, mechanism rotations)
 ├── MechanismIOTalonFX      (REAL — Phoenix 6, SensorToMechanismRatio = gearing)
 ├── MechanismIOTalonFXSim   (SIM — same Phoenix code; sim state fed by a WPILib
 │                            ElevatorSim / SingleJointedArmSim / FlywheelSim / DCMotorSim
 │                            via the MechanismSim adapter, like the swerve's
 │                            ModuleIOTalonFXSim)
 └── new MechanismIO() {}    (REPLAY — no-op; inputs come from the log)
```

IO selection happens automatically from `RobotMode.get()` — which means **RobotMode
must be set before constructing a mechanism** (Robot.java does this; tests call
`RobotMode.set(Mode.SIM)` in setup).

**Common vs robot-specific:** all control logic, plant construction, and tuning
plumbing live in `org.frc5010.common.mechanisms`. Team code only fills in a
`Settings` object — see the examples in `src/main/java/org/frc5010/examples/mechanisms/`,
all Kraken X60 on TalonFX:

| LQR style (CAN 21–28, 35) | Profiled-PID style (CAN 31–34) |
|---|---|
| `ExampleElevator` | `ExampleProfiledElevator` |
| `ExampleArm` | `ExampleProfiledArm` |
| `ExampleTurret` | `ExampleProfiledTurret` |
| `ExampleShooter` | `ExampleProfiledShooter` |
| `ExampleCharacterizedElevator` (kV/kA plant) | |
| `ExampleDoubleJointedArm` (MotionMagic — only style) | |
| `ExampleDifferentialWrist` (MotionMagic — only style) | |

## Choosing a control style

Both styles share the same settings, commands, telemetry, profile limits, and tests —
switching is `s.controlStyle = ControlStyle.PROFILED_PID;` plus gains.

- **LQR** (default): WPILib `LinearSystemLoop` (LQR gain + Kalman filter +
  plant-inversion feedforward) computed in `periodic()` at 20 ms, fed by a trapezoid
  profile, output as a `VoltageOut` request. Tune physical tolerances, not gains — but
  the plant model (mass/MOI/gearing) must be accurate, or characterize it (below).
- **PROFILED_PID**: TalonFX onboard MotionMagic (position) / VelocityVoltage
  (flywheels) at 1 kHz with Slot0 kP/kI/kD/kS/kV/kG and the right GravityType
  (Elevator_Static / Arm_Cosine). Gains are in **mechanism rotations** (kP = volts per
  rotation of error), even for elevators. Simpler to reason about and tolerant of
  model error — but hand-tuned. For flywheels kV does most of the work
  (≈ 12 V ÷ free speed in rot/s).

## Adding a mechanism (short version — see `/new-mechanism` for the playbook)

```java
public class MyElevator extends Elevator {
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
`setDutyCycle(...)`, `sysId()` (limit-guarded quasistatic+dynamic), and
`Elevator.homeCommand()`; triggers: `isAtHeight` / `isAtAngle` / `isAtSpeed`;
`getSettings()` for start points; `close()` frees the CAN IDs for tests.

Non-command goal setters — `Arm.track(Angle)` / `Pivot.track(Angle)` /
`Flywheel.track(AngularVelocity)` (and `SingleDofMechanism.commandGoalNative`) — write the goal
directly from another subsystem's `periodic()` for a **continuously moving setpoint** (e.g. a
turret tracking a target), avoiding a fresh command schedule each loop. Call every cycle (the
goal latches); `periodic()` acts on whatever goal was last written. Unlike the `goTo*` commands
these acquire no subsystem requirement, so the owning subsystem keeps control of the mechanism.

**Real-robot hardware options** (all in Settings):
- `followerCanId`/`followerOpposed` — second TalonFX on the same gearbox (set
  `motorModel = DCMotor.getKrakenX60(2)` so the plant/sim include both motors).
  `followerVisualOffset` draws it as an offset mirror in the 3D view (single-DOF only;
  see the 3D-visualization section below).
- Arm/Pivot `cancoderId`/`cancoderOffset` — absolute CANcoder mounted 1:1 on the
  joint, fused onboard (position correct at power-on, no seeding). Best paired with
  PROFILED_PID: onboard MotionMagic consumes the fused sensor at 1 kHz
  (`ExampleProfiledTurret` demonstrates it). The RIO-side LQR adds a sensor hop of
  latency on top of the fusion — prefer the rotor sensor or onboard control there.
- `Elevator.homeCommand()` — drives gently into the bottom hard stop with soft limits
  temporarily disabled, detects a debounced stator-current spike, and re-seeds the
  sensor to `minHeight`. `homingCurrentThreshold` must sit well below
  `statorCurrentLimit` (the Talon's limiter caps stall current — a threshold at the
  limit never triggers; in sim the observable ceiling is ~0.75 × the limit).
- `enableFoc` (default **true**) — all control requests run with FOC commutation
  (~15% more torque, smoother low-speed control). Requires Phoenix Pro on the device;
  unlicensed devices fall back to non-FOC and raise an UnlicensedFeatureInUse fault —
  non-Pro teams set it false. This is FOC *commutation* on the voltage-based requests
  (same gains/units as before); torque-current *control* (gains in amps) is a possible
  future option.
- `clearGoalOnDisable` — drop the goal when disabled so the mechanism stays put on
  re-enable instead of driving back to a stale target (default false = resume).
- Every motor gets a WPILib `Alert` ("<name> TalonFX disconnected") driven by the
  `connected` input.
- **Visualization** — every mechanism publishes its geometry to the single isometric
  robot view (SmartDashboard → **RobotMechanisms3D**, and the `-PwebUI` panel /
  AdvantageScope), positioned by `settings.visualPose3d` (see the next section).
  Mechanism names must be unique (they already must be for tuning tables).

## 3D visualization (isometric web view + AdvantageScope)

Robots are 3D — an elevator at the back-left and a turret spinning in the horizontal
plane can't honestly share one side-view plane. Each mechanism therefore also carries
`settings.visualPose3d`, a full **mount pose** in the robot frame (x forward, y left,
z up, meters from robot center at floor level):

- The **translation** is where the mechanism sits on the robot.
- The **rotation** re-aims its working plane (its local X → `planeX`, local Z →
  `planeUp`). It can face any direction:
  - **identity** (default) → robot **X-Z** side-view plane (arm swings fore/aft) — the
    Mechanism2d convention;
  - `MechanismVisuals3d.YAW_PLANE` → robot **X-Y** (flat): a `Pivot`'s angle becomes a
    yaw about the vertical axis, i.e. a turret (see `ExampleTurret`);
  - `MechanismVisuals3d.ROLL_PLANE` → robot **Y-Z**: the mechanism swings side-to-side
    (a side-mounted deploy). Any other `Rotation3d` works too — these three are just
    the common cases.

**Coupled mechanisms.** When one mechanism rides another — a small arm on an elevator
carriage, a flywheel on the arm tip — set the child's `settings.visualParent` to the
parent's `attachmentPose` method reference (`Elevator`/`Arm`/`Pivot`/`Flywheel` each
expose one: the carriage, the swinging tip, the wheel centre). With a parent set, the
child's `visualPose3d` becomes an **offset from the parent's live endpoint** instead of
an absolute mount, so the child tracks the parent every cycle (raising the elevator
lifts the whole arm + flywheel assembly). Chains work to any depth. The example robot
wires `ExampleElevator → ExampleArm → ExampleShooter` as a three-link demo (see
`ExampleRobot.configureDemoMechanisms`).

`settings.visualParentOffset` (a `Transform3d`, default identity) is a **structural
linkage offset** applied to the parent's endpoint *before* the child's `visualPose3d` —
the bracket/standoff that carries the child off the parent (e.g. the shooter sitting 8 cm
past the arm tip in the demo). Keeping it separate from `visualPose3d` lets the same
child pose read identically whether the mechanism is standalone or coupled.

**Follower motors** (single-DOF only: `Elevator`/`Arm`/`Pivot`). A follower
(`followerCanId`) is locked to the lead shaft, so it has no motion of its own — but you
can draw it as an **offset mirror of the mechanism**: the same geometry redrawn at
`settings.followerVisualOffset` (a `Translation3d` in the mount's local frame: x = plane
horizontal, y = plane normal, z = plane vertical). Use it to show the far side of an
elevator or a duplicated arm on the same shaft. The mirror tracks the live state every
cycle and is drawn only when a follower is configured. `ExampleElevator` carries a
follower on CAN 36 mirrored 0.5 m to the +Y side as a live example.

Every cycle each mechanism publishes its current 3D line segments (current state in
its type color, goal ghost in white) into the `MechanismVisuals3d` registry. A
flywheel instead renders as a **speedometer dial**: the needle points straight down at
zero and sweeps up as it spins — CCW for positive speed, CW for negative — normalized
to the wheel's free speed, so sign and magnitude read at a glance. Three renderers
consume the registry:

1. **Web UI isometric panel** (`-PwebUI`) — the bottom-right overlay on the field
   page draws the chassis box, the swerve wheels (steered live, line length growing
   with drive speed), a cyan gyro-heading compass on the floor, and all mechanism
   segments; drag horizontally to orbit the view, and click the title to collapse it
   (collapsing stops the poll/draw entirely, handy on narrow screens). Backed by
   `GET /api/mechanisms3d`. The chassis box and wheels are sized from the drivetrain's
   bumper dimensions and module layout automatically — no configuration needed. (The
   drivetrain also publishes a `SwerveDrive` Mechanism2d to SmartDashboard for Glass /
   AdvantageScope — see [docs/simulation.md](simulation.md).)
2. **AdvantageScope 3D** — each publish also logs `Pose3d[]` under
   **Mechanisms3d/\<name\>** (one pose per segment: position at the segment start,
   X-axis along the segment), ready to attach as articulated components on the 3D
   field view.
3. **Glass / SmartDashboard iso canvas** — the same scene is drawn as a fixed 30°
   isometric projection on a `Mechanism2d` published as **SmartDashboard →
   RobotMechanisms3D**, so the 3D layout is visible in the plain simulator without the
   web UI or AdvantageScope (no orbiting — just a static iso angle, z straight up). This
   is the **default unified robot view**: equivalent to the web panel, it draws the
   chassis box, the swerve wheels (live steer, length growing with speed), the gyro
   compass, *and* every mechanism's segments. The drivetrain feeds its part each cycle
   (`AkitSwerveDrive.periodic()` → `MechanismVisuals3d.setRobotScene(...)`); each
   mechanism feeds its own via `publish`. On by default;
   `MechanismVisuals3d.setGlassIsoViewEnabled(false)` (before anything publishes) skips
   it. This is the only robot `Mechanism2d` the library publishes — it supersedes the
   former per-mechanism side-view overlay and the separate swerve drivetrain widget.

`close()` removes the mechanism from the registry; tests that publish must call
`MechanismVisuals3d.resetForTesting()` in teardown (see `MechanismVisuals3dTest`).

## LQR tuning

LQR is tuned with *physical tolerances*, not abstract gains:

| Weight | Meaning | Default |
|---|---|---|
| `qelmsPosition` | position error you tolerate (meters or degrees). Smaller = more aggressive | 2 in / 1.5° |
| `qelmsVelocity` | velocity error you tolerate (m/s, deg/s, or RPM for flywheels) | 0.5 m/s / 20°/s |
| `relms` | control effort you allow (volts). Smaller = gentler | 12 V |

All three are live-tunable under `/Tuning/<name>/lqr_*`; on change the regulator is
rebuilt and re-seeded at the current state. In PROFILED_PID style the tunables are
`/Tuning/<name>/pid_kP|kI|kD` (re-applied onboard via `setPidGains`). See
`/tune-mechanism` for the full workflow.

LQR covers the three single-DOF plants (elevator / arm-or-pivot / flywheel).
DoubleJointedArm and DifferentialMechanism are coupled multi-motor systems those
plants don't model, so they use onboard MotionMagic with `TunableGains`.

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
settings and `MechanismLqr` builds the plant with WPILib's
`LinearSystemId.identifyPositionSystem` (or `identifyVelocitySystem` for flywheels)
instead of the mass-based model. See `ExampleCharacterizedElevator` (CAN 35) for a
fully-commented walkthrough; the functional test
`characterizedPlantElevatorConverges` proves the path works end to end.

**Procedure:**
1. Run the wrapper's `sysId()` command on the real mechanism (tethered, clear travel).
2. Open the log in the WPILib SysId tool. Units: **meters** for elevators,
   **rotations** for arms/pivots/flywheels (the settings expect those units; the
   subsystem converts to the plant's radians internally).
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

1. **Set `RobotMode` before constructing any mechanism.** IO selection
   (REAL/SIM/REPLAY) reads `RobotMode.get()`, which throws if unset — by design, to
   catch ordering bugs at startup. Tests: `RobotMode.set(Mode.SIM)` in setup,
   `RobotMode.resetForTesting()` in teardown.

2. **Profile cruise velocity must be physically achievable** (below
   free speed ÷ gearing × circumference). The profile doesn't know about saturation;
   if it outruns the mechanism the controller chases an unreachable reference and
   overshoots badly. Same for arms/pivots in rotational units.

3. **kG feedforward is required for elevators/arms.** The linearized LQR plants have
   no gravity term and LQR has no integrator, so uncompensated gravity = steady-state
   error. In LQR style kG is added on the RIO (constant for elevators, ×cos(θ) for
   arms); in PROFILED_PID style it runs onboard via the Slot0 GravityType.

4. **No kV feedforward in LQR style** — the `LinearSystemLoop` provides plant
   inversion; adding kV would double-apply. (PROFILED_PID flywheels *need* kV.)

5. **WPILib position plants have two outputs but only position is measured** —
   `MechanismLqr` builds the Kalman filter on `plant.slice(0)`. If you construct
   loops by hand, do the same or the native DARE solver reads garbage
   ("R was not symmetric").

6. **Tests: feed the Phoenix sim, in two ways.** (a) The enable watchdog: TalonFX
   outputs silently neutral (duty 0, no error) if fresh DS packets stop for ~100 ms
   real time — call `DriverStationSim.notifyNewData()` + `Unmanaged.feedEnable(...)`
   every cycle. (b) The device thread: the simulated TalonFX processes control
   requests on a real-time thread, so a paused-clock loop running flat out starves it —
   sleep a few ms per cycle (`MechanismsFunctionalTest.runScheduledFor`). Plain
   synchronous `stepOneCycle()` is otherwise fine — there are no Notifier threads in
   this design.

7. **Homing threshold vs. current limit.** `homingCurrentThreshold` ≥
   `statorCurrentLimit` can never trigger — the Talon's limiter caps the stall
   current below it (and Phoenix's motor model differs slightly from WPILib's, so in
   sim the ceiling reads ~0.75 × the limit). Default 25 A against a 40 A limit.

8. **Fused CANcoder + RIO-side LQR don't mix well in sim.** The fusion adds a sensor
   hop of latency that destabilizes an aggressive 20 ms LQR. Use the CANcoder with
   PROFILED_PID (onboard MotionMagic consumes it natively at 1 kHz) or keep the rotor
   sensor for LQR mechanisms.

9. **Mechanism rotations are the IO unit.** `SensorToMechanismRatio` = gearing, so
   soft limits, MotionMagic constraints, and onboard gains are all in mechanism
   rotations (drum rotations for elevators). Subsystems convert to meters/radians at
   the boundary; getters return WPILib units.

## AdvantageKit integration

The replay bubble is the IO layer, exactly like the swerve drive:

- `MechanismIO.MechanismIOInputs` (`@AutoLog`, `double` fields per repo convention —
  never `Measure<>`): connected, positionRot, velocityRotPerSec, appliedVolts,
  statorCurrentAmps. Dual-motor mechanisms log one inputs set per motor
  (`<name>/Lower`, `<name>/Upper` or `/Left`, `/Right`).
- `periodic()` runs `io.updateInputs(inputs)` → `Logger.processInputs(name, inputs)`
  before control, so every cycle's state lands in the `.wpilog` in all modes.
- **Getters and triggers are replay-safe**: `getHeight()`, `getAngle()`, `getSpeed()`,
  `getTilt()`/`getTwist()`, `isAt...` all read the inputs object. In REPLAY the no-op
  IO leaves inputs to the log, and the RIO-side LQR recomputes its outputs
  deterministically from the replayed inputs.
- Goals and commanded targets are recorded as outputs (`<name>/GoalMeters`,
  `<name>/CommandedHeightMeters`, ...), so commanded-vs-actual is directly plottable.

After changing what the mechanisms log, run `/validate-replay`.

## Try them in the sim

`ExampleRobot` instantiates every example mechanism in simulation (sim only — the CAN
IDs don't exist on a real robot) and binds the **X button** to drive them all to a
mid-travel point in parallel (elevators → 0.75 m, arms/turrets → 90°, shooters →
3000 RPM, DJA → 90°/0°, wrist → 45°/30°); releasing X returns everything to its
configured start point (read from each mechanism's `getSettings()`, flywheels spin
down to 0). Run `./gradlew simulateJava`, enable, press X, and watch the combined robot
view under SmartDashboard → **RobotMechanisms3D** (the isometric robot view — chassis,
wheels, gyro, and every mechanism at its `visualPose3d`). Tests that construct
`RobotContainer` must call `SwerveRobotContainer.closeMechanisms()` in teardown.

With `-PwebUI` the same run also shows the **Mechanisms 3D** isometric panel on the
field page (the examples set distinct `visualPose3d`s — the turrets use `YAW_PLANE`,
so X visibly swings them in the horizontal plane while the elevators climb). The
`ExampleArm` and `ExampleShooter` are coupled onto the `ExampleElevator` carriage there
(see "Coupled mechanisms" above), so pressing X lifts the elevator and the arm +
flywheel ride up with it — a live demo of parent-child mounting.

## Functional tests

`src/test/java/org/frc5010/examples/mechanisms/MechanismsFunctionalTest.java` builds each
example subsystem for real (TalonFX IO + Phoenix sim + physics sim + closed loop),
schedules its public command, and asserts the mechanism reaches the commanded state —
covering both control styles, the characterized plant, a live retune over NT, and the
AdvantageKit inputs path. They run in the normal `./gradlew test` suite.

## Real-robot bring-up

1. Copy an example, set real CAN IDs / gearing / masses / limits.
2. Verify in sim (`./gradlew test`, or `simulateJava` and watch the RobotMechanisms3D
   iso view — set `visualPose3d` to match where the mechanism sits on your robot).
3. On the robot: run `sysId()` to characterize kG, kV, and kA; set `kG` and (for LQR
   style) `characterizedKv`/`characterizedKa` so the plant matches the real mechanism
   (see "Characterized plants" above).
4. Tune qelms/relms live over NT (`/tune-mechanism`), then bake the final values into
   the settings.
