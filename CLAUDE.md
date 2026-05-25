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

## Test pyramid (48/48 passing as of 2026-05-25)

| Layer | Class | Factory method | IO impl | Tests |
|-------|-------|----------------|---------|-------|
| 1 — unit | `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest` | — | — | 33 |
| 2 — subsystem sim | `AkitSwerveDriveTest` | `buildWithoutPhysics()` | `ModuleIOSim` | 8 |
| 3 — physics integration | `AkitSwerveDriveSimPhysicsTest` | `build()` | `ModuleIOSimPhysics` | 7 |
| 4 — visual / interactive | `RobotContainer` visual-test sequence | `build()` | `ModuleIOSimPhysics` | visual |

Layers 1–3 extend `SimTestBase` (deterministic FPGA clock via `SimHooks`).
Layer 4 runs as a full robot program via `./gradlew simulateJava`; it is **never** in CI.

---

## Per-cycle call order — Layer 3 tests (IronMaple)

```java
drive.runVelocity(speeds);   // 1. queue voltage commands to physics controllers
drive.simulationPeriodic();  // 2. advance dyn4j world: 5 sub-ticks × 4 ms = 20 ms
drive.periodic();            // 3. read updated module caches → pose estimator
stepOneCycle();              // 4. advance FPGA clock 20 ms
```

**Wrong order = stale data.** `periodic()` reads IronMaple module position caches. Those caches are only refreshed by `simulationPeriodic()` sub-ticks. If you call `periodic()` first, it reads the zero-filled initial caches and no motion appears.

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

### 5. Running tests on Windows
There is **no WSL access** to `C:\workspace`. Always use `gradlew.bat` via PowerShell:
```powershell
cd C:\workspace\FRC5010Claude
.\gradlew.bat test
```
`./gradlew test` via Bash will fail (`/mnt/c` not mounted).

### 6. `setPose()` needs one extra cycle before measuring — Layer 4 sequence
`AkitSwerveDrive.setPose()` re-anchors the pose estimator immediately, but the Field2d widget and subsequent `getPose()` calls won't reflect the new value until `periodic()` runs again (next scheduler tick). Always insert `Commands.waitSeconds(0.05)` after a `setPose()` call inside a command sequence before asserting position, or you'll compare against the old pose.

### 7. DriverStation must be enabled for IronMaple motors to move — Layer 4
`AkitSwerveDrive.periodic()` calls `module.stop()` for every module when `DriverStation.isDisabled()`. In `simulateJava` mode, the robot starts disabled. The visual-test sequence auto-enables via `DriverStationSim` in `simulationInit()` when `-PvisualTest` is set. For interactive use, click **Enable** in the Glass Driver Station panel.

### 8. REAL mode factory throws by design
`SwerveFactory.build()` and `buildWithoutPhysics()` throw `UnsupportedOperationException` for `TALON_FX`/`SPARK_TALON` in REAL mode. Teams must instantiate `ModuleIOTalonFXReal`/`ModuleIOSparkTalon` directly with their CTRE TunerX `SwerveModuleConstants`. This is intentional — the factory can't construct motor specs without full TunerX gear/gain configuration.

### 9. `@AutoLog` fields must stay `double` — never `Measure<>`
AdvantageKit's annotation processor generates logging code that expects primitive `double` fields inside `@AutoLog`-annotated inner classes. Converting `GyroIOInputs` or `ModuleIOInputs` fields to `Measure<Distance>` etc. will cause a compile error or silent logging failure. Always extract the raw value with `.in(unit)` *before* assigning to an inputs struct field.

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
| Layer 3 tests | `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveSimPhysicsTest.java` |
| Layer 4 robot program | `src/main/java/frc/robot/Robot.java`, `RobotContainer.java` |
| IronMaple sources (read-only reference) | `yagsl_src_tmp/swervelib/simulation/ironmaple/` |

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

---

## Contribution rules

**Before committing any change to the common library (`src/main/java/org/frc5010/common/...`):**

1. **Run the full test suite** — `.\gradlew.bat test` — all 48 tests must pass. Never weaken an assertion to force a pass; fix the root cause.
2. Update any affected slash command in `.claude/commands/` (e.g. `new-sim-test`, `new-robot-profile`, `diagnose-log`).
3. Update the relevant `docs/` page (`configuration`, `architecture`, `testing`, `simulation`, or `robot-profiles`).
4. Update `CLAUDE.md` if a gotcha, file location, or architecture section is no longer accurate.
5. If a new reusable pattern was introduced, consider whether it warrants a new slash command or docs page.

The code, the tests, the docs, and the agent skills must stay in sync — stale guidance causes the next contributor to repeat solved problems.

---

## Slash commands available

- `/new-sim-test` — step-by-step playbook for adding a Layer 2 or Layer 3 sim test (includes team-specific test location)
- `/new-robot-profile` — step-by-step guide for wiring a real robot's hardware IO into `RealRobotProfile`
- `/diagnose-log` — agent workflow for reading `.wpilog` files, interpreting anomaly flags, replay, and performance comparison
