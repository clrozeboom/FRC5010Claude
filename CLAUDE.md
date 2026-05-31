# FRC5010Claude — Claude Code Project Briefing

## What this project is
WPILib 2026.2.1 FRC robot **swerve drive library**. Teams configure one `SwerveConstants` record, call `SwerveFactory.build()`, and get a fully wired `AkitSwerveDrive` subsystem that works in REAL, SIM, and REPLAY modes with AdvantageKit.

**Build system:** Gradle + GradleRIO 2026.2.1 · **Java 17** · `.\gradlew.bat test` (Windows PowerShell — never `./gradlew` via Bash; WSL cannot reach `C:\workspace`)

---

## Architecture in one diagram

```
SwerveConstants (immutable record, Builder — all fields typed WPILib units)
        │
        ▼
SwerveFactory.build()               ← SIM: IronMaple physics (SwerveDriveSimulation)
SwerveFactory.buildWithoutPhysics() ← SIM: WPILib DCMotorSim (lighter, for unit tests)
        │
        ▼
AkitSwerveDrive (SubsystemBase)
 ├── GyroIO  ──► GyroIOSim (kinematics integration, buildWithoutPhysics)
 │               GyroIOSimPhysics (reads GyroSimulation, build)
 │               GyroIONavX / GyroIOPigeon2 (REAL)
 └── Module[4]
      └── ModuleIO ──► ModuleIOSim (DCMotorSim, buildWithoutPhysics)
                       ModuleIOSimPhysics (IronMaple GenericMotorController, build)
                       ModuleIOTalonFXReal / ModuleIOSparkTalon (REAL)

RobotProfile (abstract) ──► SimRobotProfile (library CI/dev)
                             RealRobotProfile (frc.robot — team's robot)
        │
        ▼
SwerveRobotContainer (abstract) ← keyboard drive, alliance pose, visual-test auto
        │
        ▼
frc.robot.RobotContainer (concrete — extends SwerveRobotContainer)
```

**Critical distinction — `instanceof GyroIOSim` in `AkitSwerveDrive.periodic()`:**
- `buildWithoutPhysics()` uses `GyroIOSim` → the kinematics-fallback branch fires.
- `build()` uses `GyroIOSimPhysics` (a separate class) → that branch does NOT fire. Heading comes from the physics engine's `GyroSimulation`.

---

## Test pyramid (59/59 passing as of 2026-05-26)

| Layer | Class | Factory method | IO impl | Tests |
|-------|-------|----------------|---------|-------|
| 1 — unit | `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest` | — | — | 33 |
| 2 — subsystem sim | `AkitSwerveDriveTest`, `VisionSubsystemTest` | `buildWithoutPhysics()` / stub IO | `ModuleIOSim` / `VisionIO` stub | 13 |
| 3 — physics integration | `AkitSwerveDriveSimPhysicsTest`, `VisionSimIntegrationTest` | `build()` | `ModuleIOSimPhysics` + `VisionIOSim` | 10 |
| 4 — visual / interactive | `RobotContainer` visual-test sequence (6 steps incl. vision correction) | `build()` | `ModuleIOSimPhysics` + `VisionIOSim` | visual |

Layers 1–3 extend `SimTestBase` (deterministic FPGA clock via `SimHooks`).
Layer 4 runs as a full robot program via `./gradlew simulateJava`; it is **never** in CI.

---

## Per-cycle call order — Layer 3 tests (IronMaple)

```java
drive.runVelocity(speeds);   // 1. queue voltage commands to physics controllers
drive.simulationPeriodic();  // 2. advance dyn4j world: 5 sub-ticks × 4 ms = 20 ms
drive.periodic();            // 3. read updated module caches → pose estimator
// If vision is present (Layer 3 vision tests):
vision.periodic();           // 4. update VisionIOSim → call addVisionMeasurement
stepOneCycle();              // 5. advance FPGA clock 20 ms
```

**Wrong order = stale data.** `periodic()` reads IronMaple module position caches. Those caches are only refreshed by `simulationPeriodic()` sub-ticks. If you call `periodic()` first, it reads the zero-filled initial caches and no motion appears.

`vision.periodic()` must come *after* `drive.simulationPeriodic()` and `drive.periodic()` so `VisionIOSim` calls `visionSim.update()` with the freshly-updated physics pose.

Layer 2 tests (`buildWithoutPhysics`) don't need `simulationPeriodic()` — `ModuleIOSim.updateInputs()` calls `driveSim.update(0.02)` internally.

## Per-cycle call order — Layer 4 robot program

`CommandScheduler.run()` (called from `Robot.robotPeriodic()`) handles everything automatically:

```
robotPeriodic()
  └─ CommandScheduler.run()
       ├─ drive.periodic()           // read sensors → odometry → Field2d
       ├─ drive.simulationPeriodic() // advance IronMaple physics (sim only)
       └─ command.execute()          // runVelocityFieldRelative() or visual-test step
```

There is a 1-cycle lag between commanding a velocity and seeing pose displacement — this is normal for a real-time system. **Do not add a separate `drive.simulationPeriodic()` call in `Robot.simulationPeriodic()`**; the scheduler already calls it and a double-call would advance the physics engine twice per loop.

---

## SimulatedArena singleton — test isolation

`SimulatedArena` is a static singleton. Every `SwerveFactory.build()` call registers a new `SwerveDriveSimulation` body into the current arena. Without cleanup, each test accumulates stale bodies.

**Required teardown pattern in every Layer 3 `@AfterEach`:**
```java
SimulatedArena.getInstance().shutDown();         // removes all dyn4j bodies
java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
f.setAccessible(true);
f.set(null, null);                               // null the singleton → next test gets fresh Arena2026Rebuilt
```

---

## WPILib units conventions

All `SwerveConstants` fields use typed WPILib unit measures. **Never** pass raw doubles to the builder; always pass a typed measure so the caller controls units.

```java
new SwerveConstants.Builder()
    .trackWidth(Inches.of(22.75))
    .wheelBase(Inches.of(22.75))
    .wheelRadius(Inches.of(2.0))
    .maxLinearSpeed(MetersPerSecond.of(4.5))
    .maxAngularSpeed(RadiansPerSecond.of(2 * Math.PI))
    .robotMass(Pounds.of(125))        // or Kilograms.of(45)
    .bumperLength(Inches.of(30))      // or Meters.of(0.76)
    .bumperWidth(Inches.of(30))
    .odometryFrequency(Hertz.of(250)) // 250 Hz for CANivore, 100 Hz default
    .build();
```

Public fields on `SwerveConstants` are typed: `Distance trackWidth`, `LinearVelocity maxLinearSpeed`, `Mass robotMass`, `Frequency odometryFrequency`, etc. Consumers call `.in(unit)` at the point of use.

**Do NOT convert `@AutoLog` inner-class fields to `Measure<>`.**
AdvantageKit's logging serializer requires primitive `double` fields in `@AutoLog`-annotated classes (`ModuleIOInputs`, `GyroIOInputs`). These must stay `double`.

### `AkitSwerveDrive` speed accessors

```java
LinearVelocity  v = drive.getMaxLinearSpeed();   // returns constants.maxLinearSpeed directly
AngularVelocity w = drive.getMaxAngularSpeed();  // returns constants.maxAngularSpeed directly
// To extract a raw double: v.in(MetersPerSecond), w.in(RadiansPerSecond)
```

### `RobotProfile` / `SwerveRobotContainer` field length

```java
Distance len = profile.getFieldLength();            // default: Meters.of(16.540988) — Arena2026Rebuilt
Distance len = container.getFieldLength();          // same; delegates to profile when one was provided
```

---

## Robot profiles and simulation scenarios

Three scenarios, two profile classes:

| Scenario | Profile | How to run |
|----------|---------|------------|
| CI / library dev | `SimRobotProfile` | `.\gradlew.bat test -PtestSim` or automated test agents |
| VSCode "Simulate Robot Code" | `RealRobotProfile` | `.\gradlew simulateJava` (default) |
| Real hardware | `RealRobotProfile` | Deploy to RoboRIO |

`RobotContainer.selectProfile()` picks the profile:
```java
if (RobotBase.isReal()) return new RealRobotProfile();
if (Boolean.getBoolean("testSim")) return new SimRobotProfile();
return new RealRobotProfile();  // default for VSCode sim
```

`RealRobotProfile.createDrive()` branches on `RobotBase.isReal()`:
- **REAL**: wire hardware IO (`GyroIOPigeon2`, `ModuleIOTalonFXReal`, …) — must be done manually; factory throws for `TALON_FX` in REAL mode by design.
- **SIM**: call `SwerveFactory.build(CONSTANTS, BLUE_START)` — IronMaple uses the real robot's mass/geometry for accurate physics even in simulation.

`RobotProfile.createVision(drive)` returns `null` by default. Override in the team's profile to wire cameras — `SwerveRobotContainer` calls it automatically after `createDrive()` and stores the result in the `protected Vision vision` field. `SimRobotProfile` inherits the default null (lightweight CI profile, no cameras).

See `/new-robot-profile` for the step-by-step wiring guide.

### Gradle simulation flags

```powershell
.\gradlew.bat simulateJava                  # default — RealRobotProfile + IronMaple
.\gradlew.bat simulateJava -PtestSim        # SimRobotProfile (lightweight, no real CAN IDs)
.\gradlew.bat simulateJava -PvisualTest     # RealRobotProfile + automated visual-test sequence
```

Both flags are forwarded to the JVM as system properties via `tasks.withType(JavaExec)` in `build.gradle`.

---

## Key gotchas (hard-won debugging lessons)

### 1. `physicsMotionRequiresSimulationPeriodic` is the Layer 3 contract test
`SwerveModuleSimulation` pre-fills its position caches with `SIMULATION_SUB_TICKS_IN_1_PERIOD` (= 5) copies of the initial zero position at construction. Without `simulationPeriodic()`, those 5 zeros are re-read each cycle. Pose stays at origin even when `runVelocity()` is called. This is the fundamental difference between Layer 2 and Layer 3.

### 2. DCMotorSim coasts after `setPose()`
`ModuleIOSim` uses WPILib `DCMotorSim` which has its own internal velocity state. Calling `drive.setPose(Pose2d.kZero)` re-anchors the odometry estimator but does **not** stop the DCMotorSim. The motor coasts for v₀·τ ≈ 1 m/s × 0.1 s ≈ 0.1 m. Use tolerance `< 0.15 m` and 50 coast cycles, not 5.

### 3. Strafe threshold is lower than forward
Modules start facing forward (0°). A forward command works immediately. A strafe command requires modules to rotate 90° first, consuming several of the 50 test cycles. Strafe threshold: `> 0.05 m`; forward threshold: `> 0.1 m`.

### 4. Initial heading has physics noise
After one sub-tick, `initialPoseIsAtOrigin` sees a heading of ~1.5e-6 rad (sub-micro-radian numerical noise from dyn4j). Use tolerance `1e-4`, not `1e-6`.

### 5. Running tests — platform-dependent invocation
Use the wrapper that matches the host:

- **Windows local** (`C:\workspace\FRC5010Claude`): `.\gradlew.bat test` via PowerShell. WSL has no access to `C:\workspace`, so `./gradlew` via Bash fails (`/mnt/c` not mounted).
- **Linux** (Codespace, devcontainer, claude.ai/code web sandbox, CI): `./gradlew test`. The `gradlew` script is already executable in-repo.

This codebase is otherwise platform-agnostic — same JDK 17, same vendordeps, same test results. Slash commands in `.claude/commands/` are written for the Windows local workflow; on Linux, translate `.\gradlew.bat` → `./gradlew`.

### 6. `setPose()` needs one extra cycle before measuring — Layer 4 sequence
`AkitSwerveDrive.setPose()` re-anchors the pose estimator immediately, but the Field2d widget and subsequent `getPose()` calls won't reflect the new value until `periodic()` runs again (next scheduler tick). Always insert `Commands.waitSeconds(0.05)` after a `setPose()` call inside a command sequence before asserting position, or you'll compare against the old pose.

### 7. DriverStation must be enabled for IronMaple motors to move — Layer 4
`AkitSwerveDrive.periodic()` calls `module.stop()` for every module when `DriverStation.isDisabled()`. In `simulateJava` mode, the robot starts disabled. The visual-test sequence auto-enables via `DriverStationSim` in `simulationInit()` when `-PvisualTest` is set. For interactive use, click **Enable** in the Glass Driver Station panel.

### 8. REAL mode factory throws by design
`SwerveFactory.build()` and `buildWithoutPhysics()` throw `UnsupportedOperationException` for `TALON_FX`/`SPARK_TALON` in REAL mode. Teams must instantiate `ModuleIOTalonFXReal`/`ModuleIOSparkTalon` directly with their CTRE TunerX `SwerveModuleConstants`. This is intentional — the factory can't construct motor specs without full TunerX gear/gain configuration.

### 9. `@AutoLog` fields must stay `double` — never `Measure<>`
AdvantageKit's annotation processor generates logging code that expects primitive `double` fields inside `@AutoLog`-annotated inner classes. Converting `GyroIOInputs` or `ModuleIOInputs` fields to `Measure<Distance>` etc. will cause a compile error or silent logging failure. Always extract the raw value with `.in(unit)` *before* assigning to an inputs struct field.

### 10. Enabling the robot from sim code requires `setDsAttached(true)` AND a disable-tolerant caller
Two non-obvious WPILib rules govern the web interface's Enable button (`WebDriveController` + `SwerveRobotContainer`):

1. **`DriverStation.isEnabled()` is `controlWord.getEnabled() && controlWord.getDSAttached()`** (verified in the 2026.2.1 bytecode). Calling `DriverStationSim.setEnabled(true)` alone is *not* enough — without `DriverStationSim.setDsAttached(true)` the robot stays disabled and `AkitSwerveDrive.periodic()` stops every module (drive signals log once at 0, `n=1`). `Robot.simulationInit()` already pairs the two for `-PvisualTest`; `WebDriveController.applyPendingControl()` must do the same.
2. **Default commands do not run while the robot is disabled.** The code that *enables* the robot therefore cannot live in the drive default command — it would never run while disabled (catch-22). `SwerveRobotContainer` schedules `applyPendingControl()` on a separate `Commands.run(...).ignoringDisable(true)` command so the enable click is processed even from the disabled state.

Symptom if either is missing: the web Enable button appears to toggle but the robot never actually enables and never moves.

---

## Key file locations

| What | Where |
|------|-------|
| Swerve config record | `src/main/java/org/frc5010/common/drive/swerve/SwerveConstants.java` |
| Factory (build/buildWithoutPhysics) | `src/main/java/org/frc5010/common/drive/swerve/SwerveFactory.java` |
| Subsystem (periodic, simulationPeriodic) | `src/main/java/org/frc5010/common/drive/swerve/akit/AkitSwerveDrive.java` |
| Base robot container (keyboard drive, auto, alliance pose) | `src/main/java/org/frc5010/common/drive/swerve/SwerveRobotContainer.java` |
| Visual auto test sequence | `src/main/java/org/frc5010/common/drive/swerve/SwerveVisualTest.java` |
| Robot profile interface | `src/main/java/org/frc5010/common/drive/swerve/RobotProfile.java` |
| Sim robot profile (CI / library dev) | `src/main/java/org/frc5010/common/drive/swerve/SimRobotProfile.java` |
| Real robot profile placeholder | `src/main/java/frc/robot/RealRobotProfile.java` |
| Top-level robot container | `src/main/java/frc/robot/RobotContainer.java` |
| Physics module IO | `src/main/java/org/frc5010/common/drive/swerve/akit/ModuleIOSimPhysics.java` |
| Physics gyro IO | `src/main/java/org/frc5010/common/drive/swerve/akit/GyroIOSimPhysics.java` |
| DCMotorSim module IO | `src/main/java/org/frc5010/common/drive/swerve/akit/ModuleIOSim.java` |
| Log summary utility (agent-readable) | `src/main/java/org/frc5010/common/util/LogSummary.java` |
| Odometry timestamp helper | `src/main/java/org/frc5010/common/drive/swerve/akit/util/PhoenixUtil.java` |
| Sim test base class | `src/test/java/org/frc5010/common/util/SimTestBase.java` |
| Layer 2 tests | `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveTest.java` |
| Layer 3 tests (drive) | `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveSimPhysicsTest.java` |
| Layer 3 tests (vision) | `src/test/java/org/frc5010/common/subsystem/VisionSimIntegrationTest.java` |
| Layer 4 robot program | `src/main/java/frc/robot/Robot.java`, `RobotContainer.java` |
| IronMaple sources (read-only reference) | `yagsl_src_tmp/swervelib/simulation/ironmaple/` |
| Vision subsystem | `src/main/java/org/frc5010/common/vision/Vision.java` |
| Vision IO interface + @AutoLog inputs | `src/main/java/org/frc5010/common/vision/VisionIO.java` |
| Vision factory (REAL/SIM/REPLAY wiring) | `src/main/java/org/frc5010/common/vision/VisionFactory.java` |
| Camera configuration (Builder pattern) | `src/main/java/org/frc5010/common/vision/CameraConfig.java` |
| PhotonVision IO (REAL) | `src/main/java/org/frc5010/common/vision/VisionIOPhoton.java` |
| Limelight IO via YALL (REAL) | `src/main/java/org/frc5010/common/vision/VisionIOLimelight.java` |
| Vision sim IO (extends VisionIOPhoton) | `src/main/java/org/frc5010/common/vision/VisionIOSim.java` |
| Vision Layer 2 tests | `src/test/java/org/frc5010/common/subsystem/VisionSubsystemTest.java` |

---

## Log analysis

Every `simulateJava` run writes a `.wpilog` to the `logs/` directory (configured in `Robot.java`). Replay runs write `<original>_sim.wpilog` alongside the source log.

**Agent-readable summary:**
```powershell
.\gradlew.bat logSummary                              # most recent log in logs/
.\gradlew.bat logSummary -PlogFile=logs/foo.wpilog    # specific file
```
Output: entry list, min/max per signal, anomaly flags (loop overruns > 25 ms, gyro disconnect, motor current > 60 A).

**Replay (re-run code against a recorded log):**
```powershell
.\gradlew.bat replayWatch    # opens file picker; output written to <original>_sim.wpilog
```

**Key signal paths in the log** (from `@AutoLogOutput` and `@AutoLog`-generated fields):
- `RealOutputs/Drive/Pose` — `double[]` [x, y, headingRad]
- `RealOutputs/Drive/Module{0-3}DriveVelocityRadPerSec` — `double`
- `RealOutputs/Drive/Module{0-3}DriveCurrentAmps` — `double`
- `RealOutputs/Drive/Module{0-3}TurnPosition` — `double`
- `RealOutputs/Drive/GyroConnected` — `boolean`
- `RealOutputs/Drive/GyroYawPositionRad` — `double`

Use `.\gradlew.bat logSummary` to discover the actual entry paths in any given log before searching for a specific signal.

See `/diagnose-log` slash command for the full agent workflow.

---

## CI / devcontainer

- **CI:** `.github/workflows/ci.yml` — `./gradlew test` on every push/PR to `main`
- **Codespaces:** `.devcontainer/` — Java 17 bookworm + xvfb; `postCreateCommand` pre-warms Gradle; forwards ports 5810 (NT4), 5800, 1735
- **Sim sharing:** `xvfb-run ./gradlew simulateJava` in Codespace → VS Code auto-forwards port 5810 → AdvantageScope connects live

### Setting up Claude Code on the web

To use this repo in a [claude.ai/code](https://claude.ai/code) web session:

1. **Install the Claude Code GitHub App** — go to [github.com/apps/claude](https://github.com/apps/claude), click **Install**, and grant it read/write access to this repository. Without write access the agent cannot push commits or create branches.
2. **Allow the required domains** — when creating a new environment, set the **network policy** to allow the domains listed below so the Gradle build and vendordep downloads succeed. See the [environment configuration docs](https://code.claude.com/docs/en/claude-code-on-the-web) for how to set the allowed-domains list.

### Trusted domains for Claude Code on the web

When creating a new environment at [claude.ai/code](https://claude.ai/code), set the **network policy** to allow these domains so the Gradle build and vendordep downloads succeed. See [environment configuration docs](https://code.claude.com/docs/en/claude-code-on-the-web) for how to set the allowed-domains list.

| Domain | Purpose |
|--------|---------|
| `services.gradle.org` | Gradle wrapper distribution (`gradle-8.11-bin.zip`) |
| `plugins.gradle.org` | Gradle Plugin Portal — GradleRIO plugin |
| `frcmaven.wpi.edu` | WPILib Maven — WPILib libraries + AdvantageKit |
| `repo1.maven.org` | Maven Central — JUnit, YAGSL transitive deps |
| `maven.ctr-electronics.com` | CTRE Phoenix 6 |
| `maven.revrobotics.com` | REV Robotics |
| `maven.reduxrobotics.com` | Redux Robotics |
| `docs.home.thethriftybot.com` | ThriftyBot library |
| `maven.photonvision.org` | PhotonVision |
| `yet-another-software-suite.github.io` | YAGSL |
| `3015rangerrobotics.github.io` | PathPlannerLib |
| `pypi.org` | Python packages for the `frc-docs` MCP server (`uvx first-agentic-csa`) |
| `files.pythonhosted.org` | Python package downloads for `frc-docs` MCP server |

---

## Contribution rules

**Before committing any change to the common library (`src/main/java/org/frc5010/common/...`):**

1. **Run the full test suite** — `.\gradlew.bat test` — all 59 tests must pass. Never weaken an assertion to force a pass; fix the root cause.
2. Update any affected slash command in `.claude/commands/` (e.g. `new-sim-test`, `new-robot-profile`, `diagnose-log`, `validate-replay`).
3. Update the relevant `docs/` page (`configuration`, `architecture`, `testing`, `simulation`, or `robot-profiles`).
4. Update `CLAUDE.md` if a gotcha, file location, or architecture section is no longer accurate.
5. If a new reusable pattern was introduced, consider whether it warrants a new slash command or docs page.
6. **For non-trivial logging changes** — any change to `@AutoLog` fields, `Robot.java` data receivers, or `LogSummary.java` — validate replay fidelity:
   ```powershell
   # 1. Produce a live log (Glass opens, auto-closes when test completes)
   .\gradlew.bat simulateJava -PvisualTest -PvisualTestExit
   # 2. Replay it headlessly; exits automatically when autonomous completes
   .\gradlew.bat simulateJava -Plog=logs/<your-log>.wpilog -PvisualTest -PreplayExit
   # 3. Check the replay log for anomalies vs the live log
   .\gradlew.bat replayValidate
   ```
   See `/validate-replay` for the full workflow and how to interpret the output.

The code, the tests, the docs, and the agent skills must stay in sync — stale guidance causes the next contributor to repeat solved problems.

---

## Vision architecture

### Overview

The vision subsystem follows the same AdvantageKit IO pattern as the drive subsystem.

```
CameraConfig[] (one per camera, Builder pattern)
        │
        ▼
VisionFactory.build(consumer, poseSupplier, headingSupplier, configs)
        │
        ├─ REAL  → VisionIOPhoton  (PhotonVision, multi-tag PnP)
        │          VisionIOLimelight (YALL, MegaTag 1 + MegaTag 2)
        ├─ SIM   → VisionIOSim    (extends VisionIOPhoton, PhotonCameraSim)
        │          (Limelight → no-op; no PhotonVision sim equivalent)
        └─ REPLAY→ no-op VisionIO (AKit replays logged inputs automatically)
                │
                ▼
         Vision (SubsystemBase)
          ├─ filters bad observations (ambiguity, Z error, field boundaries)
          ├─ scales std devs: distance²/tagCount × stdDevFactor
          │    MegaTag 2: ½ linear, 1e6× angular (heading locked)
          └─ calls consumer (drive::addVisionMeasurement) for accepted poses
```

### Key design decisions

- **`@AutoLog` parallel arrays** — `VisionIOInputs` uses parallel primitive/struct arrays (`double[]`, `Pose3d[]`, `int[]`) instead of a `PoseObservation[]` record. AdvantageKit's annotation processor only serializes WPILib struct types; custom records cause the field to be typed `Object[]` and break all accessors in `Vision.periodic()`.
- **`VisionIO.updateInputs` is `default`** — allows `new VisionIO() {}` no-op for REPLAY/Limelight-in-SIM without subclassing. Consequence: `VisionIO` is NOT a `@FunctionalInterface`; use anonymous inner classes (not lambdas) in tests.
- **`AprilTagFields.kDefaultField`** — always `k2026RebuiltWelded`. Using `kDefaultField` means the factory tracks future season defaults automatically.
- **`poseSupplier` must be the TRUE physics position** — `VisionIOSim` uses this supplier to place the simulated camera. Always pass `() -> drive.getSimulatedPose().orElse(drive.getPose())`, NOT `drive::getPose`. If you use the estimator pose and then inject an estimator error (e.g. push-correction test), the camera sim will be moved to the wrong position and stop detecting tags — breaking the very correction you're testing.
- **MegaTag 1 via NT queue** — `megatag1Subscriber.readQueue()` drains every frame since the last cycle so no poses are dropped between 20 ms loops.
- **Orientation via YALL `withRobotOrientation`** — `limelight.getSettings().withRobotOrientation(new Orientation3d(rot3d, zero))` sets the NT key `robot_orientation_set` and flushes; the Limelight uses this to lock its heading for MegaTag 2.

### Usage example

```java
Vision vision = VisionFactory.build(
    drive::addVisionMeasurement,
    () -> drive.getSimulatedPose().orElse(drive.getPose()),  // TRUE physics position, not estimator
    drive::getRotation,
    new CameraConfig[] {
        new CameraConfig.Builder("photon_front")
            .robotToCamera(FRONT_CAM_TRANSFORM)
            .backend(CameraConfig.Backend.PHOTON)
            .build(),
        new CameraConfig.Builder("limelight")
            .robotToCamera(REAR_CAM_TRANSFORM)
            .backend(CameraConfig.Backend.LIMELIGHT)
            .stdDevFactor(0.8)   // trust this camera more
            .build()
    });
```

See `/new-vision-camera` for the step-by-step wiring guide.

---

## MCP servers available

- **`frc-docs`** (`first-agentic-csa` via `uvx`, configured in `.mcp.json`) — natural-language search across WPILib, REV Robotics, CTRE Phoenix, Redux Robotics, and PhotonVision documentation. Prefer this over `WebFetch` for vendor-API questions ("how do I configure a SparkMax", "what's the TalonFX position-PID signature") — it's a single search across all five doc sites and respects the project's language context. Requires `uv` on PATH locally (Codespace/web sandbox installs it via `.devcontainer/Dockerfile`).

---

## Slash commands available

- `/new-sim-test` — step-by-step playbook for adding a Layer 2 or Layer 3 sim test (includes team-specific test location)
- `/new-robot-profile` — step-by-step guide for wiring a real robot's hardware IO into `RealRobotProfile`
- `/diagnose-log` — agent workflow for reading `.wpilog` files, interpreting anomaly flags, replay, and performance comparison
- `/new-vision-camera` — step-by-step guide for adding a PhotonVision or Limelight camera to the Vision subsystem
- `/new-game-field` — build a 2D web field + custom IronMaple arena (barriers + game pieces) from a new season's game manual, for when IronMaple hasn't shipped the season arena yet
- `/validate-replay` — validate replay fidelity after non-trivial logging changes (produce log → replay → check anomalies)

---

## TODO (future sessions)

- **Web field: render live game pieces.** Pull game-piece poses from
  `SimulatedArena.getInstance()` (the dynamic objects, not obstacles) and draw them
  on the web field view. They are dynamic (move / get scored) so must be polled each
  frame via `/api/state` (or a new `/api/gamepieces` endpoint), not hard-coded.
