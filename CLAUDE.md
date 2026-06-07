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
 ├── WebControl (optional singleton — HTTP server port 5800; -PwebUI only)
 └── frc.robot.RealRobot (concrete — extends SwerveRobotContainer, owns DemoIntake)
      └── frc.robot.RobotContainer (thin shell — delegates getAutonomousCommand / resetToAllianceStart)

SimRobotState (abstract SubsystemBase) ──► frc.robot.DemoIntake (2026 Fuel intake + ballistic firing)
```

**Critical distinction — `instanceof GyroIOSim` in `AkitSwerveDrive.periodic()`:**
- `buildWithoutPhysics()` uses `GyroIOSim` → the kinematics-fallback branch fires.
- `build()` uses `GyroIOSimPhysics` (a separate class) → that branch does NOT fire. Heading comes from the physics engine's `GyroSimulation`.

---

## Test pyramid

| Layer | Short name | Class | Factory method | IO impl |
|-------|-----------|-------|----------------|---------|
| 1 | **unit** | `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest`, `JoystickAxisTest`, `SelectProfileTest` | — | — |
| 2 | **subsystem** | `AkitSwerveDriveTest`, `VisionSubsystemTest`, `RobotContainerSmokeTest` | `buildWithoutPhysics()` / stub IO | `ModuleIOSim` / `VisionIO` stub |
| 3 | **physics** | `AkitSwerveDriveSimPhysicsTest`, `VisionSimIntegrationTest`, `DriveCalibrationSimPhysicsTest` | `build()` | `ModuleIOSimPhysics` + `VisionIOSim` |
| 4 | **functional** | `WebUIFunctionalTest` — HTTP-driven via `WebDriveController` | `build()` | `ModuleIOSimPhysics` |
| 5 | **visual** | `RobotContainer` visual-test sequence (6 steps incl. vision correction) | `build()` | `ModuleIOSimPhysics` + `VisionIOSim` |

Layers 1–3 extend `SimTestBase` (deterministic FPGA clock via `SimHooks`).
Layer 4 spawns the full robot program as a subprocess (`./gradlew functionalTest`) — occasional, not in CI.
Layer 5 runs as a full robot program via `./gradlew simulateJava`; it is **never** in CI.

Full Layer-by-layer detail, per-cycle call order, and `SimulatedArena` teardown pattern: [docs/testing.md](docs/testing.md).

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
- **Windows local** (`C:\workspace\FRC5010Claude`): `.\gradlew.bat test` via PowerShell. WSL has no access to `C:\workspace`, so `./gradlew` via Bash fails.
- **Linux** (Codespace, devcontainer, claude.ai/code web sandbox, CI): `./gradlew test`.

Slash commands in `.claude/commands/` are written for the Windows local workflow; on Linux, translate `.\gradlew.bat` → `./gradlew`.

### 6. `setPose()` needs one extra cycle before measuring — Layer 4 sequence
`AkitSwerveDrive.setPose()` re-anchors the pose estimator immediately, but the Field2d widget and subsequent `getPose()` calls won't reflect the new value until `periodic()` runs again (next scheduler tick). Always insert `Commands.waitSeconds(0.05)` after a `setPose()` call inside a command sequence before asserting position, or you'll compare against the old pose.

### 7. DriverStation must be enabled for IronMaple motors to move — Layer 4
`AkitSwerveDrive.periodic()` calls `module.stop()` for every module when `DriverStation.isDisabled()`. In `simulateJava` mode, the robot starts disabled. The visual-test sequence auto-enables via `DriverStationSim` in `simulationInit()` when `-PvisualTest` is set. For interactive use, click **Enable** in the Glass Driver Station panel (or the web UI's Enable button under `-PwebUI`).

### 8. REAL mode factory throws by design
`SwerveFactory.build()` and `buildWithoutPhysics()` throw `UnsupportedOperationException` for `TALON_FX`/`SPARK_TALON` in REAL mode. Teams must instantiate `ModuleIOTalonFXReal`/`ModuleIOSparkTalon` directly with their CTRE TunerX `SwerveModuleConstants`. This is intentional — the factory can't construct motor specs without full TunerX gear/gain configuration.

### 9. `@AutoLog` fields must stay `double` — never `Measure<>`
AdvantageKit's annotation processor generates logging code that expects primitive `double` fields inside `@AutoLog`-annotated inner classes. Converting `GyroIOInputs` or `ModuleIOInputs` fields to `Measure<Distance>` etc. will cause a compile error or silent logging failure. Always extract the raw value with `.in(unit)` *before* assigning to an inputs struct field.

### 10. Enabling the robot from sim code requires `setDsAttached(true)` AND a disable-tolerant caller
Two non-obvious WPILib rules govern the web interface's Enable button (`WebDriveController` + `SwerveRobotContainer`):

1. **`DriverStation.isEnabled()` is `controlWord.getEnabled() && controlWord.getDSAttached()`** (verified in the 2026.2.1 bytecode). Calling `DriverStationSim.setEnabled(true)` alone is *not* enough — without `DriverStationSim.setDsAttached(true)` the robot stays disabled and `AkitSwerveDrive.periodic()` stops every module (drive signals log once at 0, `n=1`). `Robot.simulationInit()` already pairs the two for `-PvisualTest`; `WebDriveController.applyPendingControl()` must do the same.
2. **Default commands do not run while the robot is disabled.** The code that *enables* the robot therefore cannot live in the drive default command — it would never run while disabled (catch-22). `SwerveRobotContainer` schedules `applyPendingControl()` on a separate `Commands.run(...).ignoringDisable(true)` command so the enable click is processed even from the disabled state.

Symptom if either is missing: the web Enable button appears to toggle but the robot never actually enables and never moves.

### 11. Verify web-UI / mechanism / game-piece changes by RUNNING the sim — tests alone miss them
Several real bugs passed the whole test suite and only surfaced when the sim was actually driven. Before claiming such a change works, run `./gradlew simulateJava -PwebUI`, drive the real flow, and **poll `/api/state` over time** (`heldFuel`, `scoredFuel`, `x`/`y`):
- **`WebUIFunctionalTest` exercises the backend HTTP API, not the browser JS or the visual telemetry.** A frozen Field2d, a non-firing button handler, or wrong `/api/state` numbers will not fail it. (Example: `poseBuf` was only written inside the drive *default* command, so the web field froze for the entire duration of any auto — every test still passed.)
- **Web telemetry (`poseBuf`, demo-state suppliers) must update in ALL robot states.** Anything that reads the drive subsystem only via the default command goes stale whenever an auto/other command owns `drive`. Put per-cycle web snapshots in the always-running `applyPendingControl()` (the `WebControlApply` command requires no subsystems and `ignoringDisable(true)`).
- **Game-piece autos must be routed against the ACTUAL spawn positions in `GamePieceSpawner` (center grid x 7.43–9.11), not assumptions.** pickupAndScore originally stopped at x=6.0 and collected nothing because the Fuel grid starts at x=7.43. When asserting "it collects," require collection *beyond* any preload (`maxHeld > preload`), or the `DemoIntake` 8-piece preload masks a robot that grabbed nothing.

---

## Key file locations

| What | Where |
|------|-------|
| Swerve config record | `src/main/java/org/frc5010/common/drive/swerve/SwerveConstants.java` |
| Factory (build/buildWithoutPhysics) | `src/main/java/org/frc5010/common/drive/swerve/SwerveFactory.java` |
| Subsystem (periodic, simulationPeriodic) | `src/main/java/org/frc5010/common/drive/swerve/akit/AkitSwerveDrive.java` |
| Base robot container (keyboard drive, auto, alliance pose) | `src/main/java/org/frc5010/common/profiles/SwerveRobotContainer.java` |
| Visual auto test sequence | `src/main/java/org/frc5010/common/sim/SwerveVisualTest.java` |
| Joystick axis transform pipeline | `src/main/java/org/frc5010/common/input/JoystickAxis.java` |
| 2-D drive vector (combines two JoystickAxis) | `src/main/java/org/frc5010/common/input/DriveVector.java` |
| Generic configurable controller (port-based) | `src/main/java/org/frc5010/common/input/ConfigurableController.java` |
| Xbox-specific named accessors | `src/main/java/org/frc5010/common/input/XboxConfigurableController.java` |
| Robot profile interface | `src/main/java/org/frc5010/common/profiles/RobotProfile.java` |
| Sim robot profile (CI / library dev) | `src/main/java/org/frc5010/common/profiles/SimRobotProfile.java` |
| Real robot profile placeholder | `src/main/java/frc/robot/RealRobotProfile.java` |
| Top-level robot container | `src/main/java/frc/robot/RobotContainer.java` |
| Browser-based web UI controller (sim only, `-PwebUI`) | `src/main/java/org/frc5010/common/sim/WebDriveController.java` |
| Demo intake (team-code example) | `src/main/java/frc/robot/DemoIntake.java` |
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
| Calibration result record | `src/main/java/org/frc5010/common/drive/swerve/calibration/CalibrationResult.java` |
| Calibration data-collection routine | `src/main/java/org/frc5010/common/drive/swerve/calibration/MotorCalibrationRoutine.java` |
| BLine path-follower wrapper (game-agnostic) | `src/main/java/org/frc5010/common/drive/swerve/auto/BLineSwerveAuto.java` |
| Auto routines (game-specific BLine examples) | `src/main/java/frc/robot/AutoRoutines.java` |
| Teleop drive-to-pose commands (game-specific) | `src/main/java/frc/robot/TeleopRoutines.java` |
| Deployed BLine paths + config | `src/main/deploy/autos/` |
| BLine sim test | `src/test/java/org/frc5010/common/subsystem/BLineFollowPathSimPhysicsTest.java` |

---

## Deeper docs

| Topic | File |
|---|---|
| `SwerveConstants` field reference, units conventions, speed accessors | [docs/configuration.md](docs/configuration.md) |
| `RobotProfile` pattern, REAL/SIM branching, field length, vision wiring | [docs/robot-profiles.md](docs/robot-profiles.md) |
| Simulation scenarios, Gradle flags (`-PtestSim` / `-PvisualTest` / `-PwebUI`), AdvantageScope | [docs/simulation.md](docs/simulation.md) |
| Test pyramid in depth, per-cycle call order, `SimulatedArena` teardown, log analysis | [docs/testing.md](docs/testing.md) |
| Vision architecture (IO pattern, design decisions, usage example) | [docs/vision.md](docs/vision.md) |
| Motor calibration workflow (sim ramp → SysId → apply gains) | [docs/calibration.md](docs/calibration.md) |
| BLine path-following — auto chooser, JSON + code-defined paths, drive-to-pose button | [docs/auto.md](docs/auto.md) |
| High-level architecture overview | [docs/architecture.md](docs/architecture.md) |
| Local / Codespaces / claude.ai/code / CI environments + trusted-domain list | [docs/environment.md](docs/environment.md) |
| Student-facing setup walkthrough (fork → deploy) | [docs/student-setup.md](docs/student-setup.md) |
| GitHub Codespaces setup and usage (build, sim, web UI, AdvantageScope, log replay) | [docs/codespaces.md](docs/codespaces.md) |
| Contribution rules (test first, docs in sync, replay validation) | [CONTRIBUTING.md](CONTRIBUTING.md) |

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
- `/calibrate-drive` — agent-guided step-by-step motor calibration (sim ramp → SysId → apply gains to TunerX or DriveConstants)

---

## TODO (future sessions)

- **Web field: render live game pieces.** Pull game-piece poses from
  `SimulatedArena.getInstance()` (the dynamic objects, not obstacles) and draw them
  on the web field view. They are dynamic (move / get scored) so must be polled each
  frame via `/api/state` (or a new `/api/gamepieces` endpoint), not hard-coded.
