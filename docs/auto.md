# Autonomous routines with BLine

FRC5010Claude integrates [BLine-Lib](https://github.com/edanliahovetsky/BLine-Lib) (Team 2638)
as its path-following framework for autonomous and driver-triggered drive-to-pose commands.

BLine uses **polyline** paths (straight segments between waypoints) rather than Bézier or
time-parameterised trajectories. Following is position-based: at each cycle, BLine computes
a `ChassisSpeeds` that closes the remaining distance via PID, applies cross-track correction,
and rate-limits acceleration. Paths can be defined in code, loaded from JSON authored with
BLine-GUI, or built at runtime.

---

## Where things live

| What | Where |
|---|---|
| Vendordep | [vendordeps/BLine-Lib.json](../vendordeps/BLine-Lib.json) |
| Common-library wrapper (game-agnostic) | [BLineSwerveAuto.java](../src/main/java/org/frc5010/common/drive/swerve/auto/BLineSwerveAuto.java) |
| Example auto routines (game-specific) | [AutoRoutines.java](../src/main/java/frc/robot/AutoRoutines.java) |
| Teleop drive-to-pose commands (game-specific) | [TeleopRoutines.java](../src/main/java/frc/robot/TeleopRoutines.java) |
| Auto chooser + Y-button binding | [RealRobot.java](../src/main/java/frc/robot/RealRobot.java) |
| Global default constraints (deployed) | [src/main/deploy/autos/config.json](../src/main/deploy/autos/config.json) |
| JSON path example (deployed) | [src/main/deploy/autos/paths/ExampleScore.json](../src/main/deploy/autos/paths/ExampleScore.json) |
| Layer 3 integration test | [BLineFollowPathSimPhysicsTest.java](../src/test/java/org/frc5010/common/subsystem/BLineFollowPathSimPhysicsTest.java) |

---

## Wiring `BLineSwerveAuto`

`BLineSwerveAuto.builder(drive)` returns a `FollowPath.Builder` pre-wired with:
- Pose supplier → `drive::getPose`
- Robot-relative `ChassisSpeeds` supplier → `drive::getChassisSpeeds`
- Robot-relative `ChassisSpeeds` consumer → `drive::runVelocity`
- Translation PID (5.0, 0, 0), rotation PID (3.0, 0, 0), cross-track PID (2.0, 0, 0) — BLine quick-start gains
- `.withDefaultShouldFlip()` — paths authored Blue-side are mirrored on Red automatically
- `.withPoseReset(drive::setPose)` — odometry re-anchors to the path's start pose at command init

It also installs `Path.DefaultGlobalConstraints` derived from the drive's `getMaxLinearSpeed()`
and `getMaxAngularSpeed()`. The default acceleration limit is set to **2 × max linear speed**;
lower values leave the rate limiter unable to decelerate from cruise within the end-tolerance
window, causing overshoot on short paths. **Teams should tune the PIDs and constraints to
their robot's measured response.**

---

## Auto-mode examples

`RealRobot` constructs a `SendableChooser<Command>` exposed as the `Auto Mode` widget on
SmartDashboard / Driver Station:

| Chooser option | Behaviour |
|---|---|
| `None` | `Commands.none()` — default |
| `BLine: Example Score (JSON)` | Loads `ExampleScore.json` from `deploy/autos/paths/`, drives the polyline |
| `BLine: Example Score (code)` | Same trajectory as above, defined inline in `AutoRoutines.exampleScoreInCode` |
| `BLine: Pickup + Score` | Drives out, intakes Fuel, drives back into the scoring zone, fires at the Hub. **Sim-only** — registered only when `DemoIntake` is present. |

The `-PvisualTest` Gradle flag still wins: `RealRobot.getAutonomousCommand()` calls
`super.getAutonomousCommand()` first, which returns `SwerveVisualTest` under the flag.

### Composing BLine with mechanism commands

The `pickupAndScore` example in [AutoRoutines.java](../src/main/java/frc/robot/AutoRoutines.java)
shows the recommended composition pattern:

```java
return Commands.sequence(
    Commands.runOnce(() -> drive.setPose(start), drive),
    Commands.waitSeconds(0.05),                            // setPose needs one cycle to read back
    Commands.parallel(b.build(outbound), intake.extendCommand()),
    Commands.waitUntil(() -> intake.getHeldFuel() > 0).withTimeout(2.0),
    intake.retractCommand(),
    b.build(returnPath),
    intake.fireCommand(),
    Commands.runOnce(drive::stop, drive));
```

- `Commands.parallel(path, extend)` lets the intake state machine run while the path drives.
- `waitUntil` gated by `withTimeout` keeps the auto from hanging if no Fuel is on the path.
- All poses are Blue-side; BLine's `.withDefaultShouldFlip()` mirrors for Red.
- The `Commands.waitSeconds(0.05)` after `setPose` is gotcha #6 in CLAUDE.md — the pose
  estimator needs one extra cycle before subsequent reads reflect the new value.
- **Route around field obstacles.** The Blue Hub is a physical body on the field centerline at
  `(4.5974, 4.0345)`; a straight path through it drives the robot into it and stalls. The
  outbound/return paths therefore use `TranslationTarget` via-points through the clear lane below
  the Hub (`y≈2.5`) rather than a straight line. BLine v0.9.1 has no runtime pathfinder, so
  obstacle avoidance is hand-authored via-points (same pattern as `driveToVia` below). This is
  regression-tested by `AutoRoutinesSimPhysicsTest.pickupAndScore_realRoutine_roundsHubReachesPickupAndReturns`,
  which runs the full routine under physics and asserts the robot rounds the Hub to the pickup.

---

## Driver button: drive-to-pose

`RealRobot.configureBindings()` binds the **Y** button to
`TeleopRoutines.driveToHub(drive)`. The implementation lives in
[TeleopRoutines.java](../src/main/java/frc/robot/TeleopRoutines.java) (game-specific —
2026 Reef coordinates), and routes the robot to a fixed Reef-approach pose using two
hand-authored via-points to skirt the Reef centre:

```java
public static Command driveToHub(AkitSwerveDrive drive) {
  Pose2d hubApproach = new Pose2d(3.6, 4.03, Rotation2d.fromDegrees(0));
  return BLineSwerveAuto.driveToVia(
      drive,
      hubApproach,
      List.of(new Translation2d(2.5, 5.5), new Translation2d(3.1, 4.8)));
}
```

The underlying `BLineSwerveAuto.driveToVia` (game-agnostic, lives in the common library)
uses `Commands.defer` so the start pose is captured at button-press time (not at binding
registration). It also overrides `withPoseReset` with a no-op so the live odometry isn't
clobbered with a stale snapshot.

**Obstacle avoidance is hand-authored.** BLine v0.9.1 does not expose a runtime Theta\* /
pathfinder API — only `FollowPath`, `Path`, `BLineCommands`, `BLineField`, `FlippingUtil`,
`ChassisRateLimiter`, and `JsonUtils` are public. If a future BLine release ships a
pathfinder, swap the hard-coded via-point list for the pathfinder's output without
changing call sites.

---

## JSON path schema

BLine reads paths from `deploy/autos/paths/<name>.json` (resolved on robot via
`Filesystem.getDeployDirectory()`). Minimum structure:

```json
{
  "path_elements": [
    { "type": "waypoint",
      "translation_target": { "x_meters": 1.5, "y_meters": 2.0 },
      "rotation_target": { "rotation_radians": 0.0, "t_ratio": 0.0, "profiled_rotation": true } },
    { "type": "translation", "x_meters": 2.25, "y_meters": 2.0 },
    { "type": "waypoint",
      "translation_target": { "x_meters": 3.0, "y_meters": 2.0 },
      "rotation_target": { "rotation_radians": 0.0, "t_ratio": 1.0, "profiled_rotation": true } }
  ]
}
```

Global constraint defaults live in `deploy/autos/config.json` and use the
`default_*` key prefix (e.g. `default_max_velocity_meters_per_sec`).

For interactive authoring use [BLine-GUI](https://github.com/edanliahovetsky/BLine-GUI).

---

## Tuning

The BLineSwerveAuto defaults are tuned for the IronMaple physics sim — they pass the
Layer 3 test, but they are NOT calibrated for any specific real robot. Before deploying
to hardware:

1. Measure your robot's true max acceleration (e.g. via `/calibrate-drive`'s SysId step).
2. Override the global constraints with `Path.setDefaultGlobalConstraints(...)` at robot init
   using the measured value (or build a custom `BLineSwerveAuto`-style helper).
3. Tune the translation / rotation / cross-track PIDs against your robot — high `kP` plus
   low acceleration limits causes oscillation near the end of paths; low `kP` plus high
   acceleration causes overshoot.

The Layer 3 test ([BLineFollowPathSimPhysicsTest](../src/test/java/org/frc5010/common/subsystem/BLineFollowPathSimPhysicsTest.java))
is a smoke test of the integration plumbing, not a tuning regression — it asserts the robot
reaches within 0.30 m of the goal, which is generous on purpose.
