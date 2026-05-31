# Testing

The project uses a four-layer test pyramid. Layers 1–3 are automated and run in CI; Layer 4 is visual and run manually.

---

## Test pyramid

| Layer | What | Factory | IO impl |
|-------|------|---------|---------|
| 1 — unit | `SwerveConstantsTest`, `SwerveFactoryModeTest`, `TunableGainsTest`, `JoystickAxisTest` | — | — |
| 2 — subsystem sim | `AkitSwerveDriveTest`, `VisionSubsystemTest` | `buildWithoutPhysics()` | `ModuleIOSim` (DCMotorSim) |
| 3 — physics integration | `AkitSwerveDriveSimPhysicsTest`, `VisionSimIntegrationTest` | `build()` | `ModuleIOSimPhysics` (IronMaple) |
| 4 — visual / interactive | `RobotContainer` visual-test sequence | `build()` | `ModuleIOSimPhysics` |

The CI badge at the top of the README shows the current pass/fail status.

---

## Running tests

```powershell
# Windows PowerShell (from the project root)
.\gradlew.bat test

# macOS / Linux / Codespaces
./gradlew test

# Force a re-run even if Gradle thinks nothing changed
.\gradlew.bat cleanTest test   # Windows
./gradlew cleanTest test        # macOS / Linux
```

HTML report: `build/reports/tests/test/index.html`

VS Code shortcut: **Ctrl+Shift+P → Tasks: Run Test Task → Run Unit Tests**

---

## Layer 1 — Unit tests

Test `SwerveConstants` builder validation, `SwerveFactory` mode-selection logic, and `TunableGains`. No simulation infrastructure needed — these are plain JUnit 5 tests.

Representative assertions:
```java
// Builder accepts typed units
SwerveConstants c = new SwerveConstants.Builder()
    .trackWidth(Inches.of(22.75))
    .robotMass(Pounds.of(125))
    .build();
assertEquals(22.75, c.trackWidth.in(Inches), 1e-6);

// Factory throws in REAL mode for hardware types
RobotMode.set(Mode.REAL);
assertThrows(UnsupportedOperationException.class, () -> SwerveFactory.build(TALON_CONSTANTS));
```

---

## Layer 2 — Subsystem sim tests

`buildWithoutPhysics()` uses WPILib `DCMotorSim` — no dyn4j, no IronMaple overhead. All Layer 2 tests extend `SimTestBase`, which initialises HAL, pauses the FPGA clock, and cleans up `CommandScheduler` between tests.

### Test skeleton

```java
@Test
void myBehavior() {
    enableTeleop();
    for (int i = 0; i < 50; i++) {          // 50 × 20 ms = 1 simulated second
        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
        drive.periodic();                     // reads DCMotorSim → odometry
        stepOneCycle();                       // advance FPGA clock 20 ms
    }
    Pose2d pose = drive.getPose();
    // assert on pose.getX(), pose.getY(), pose.getRotation()
}
```

No `drive.simulationPeriodic()` — `ModuleIOSim` calls `driveSim.update(0.02)` internally during `periodic()`.

### Thresholds (Layer 2)

| Motion | Expected after 50 cycles (1 s at 1 m/s or π rad/s) |
|--------|-----------------------------------------------------|
| Forward 1 m/s | X > 0.1 m |
| Strafe 1 m/s | Y > 0.1 m |
| Rotate π rad/s | \|heading\| > 0.1 rad |
| After `setPose()` coast | < 0.15 m from target (use 50 coast cycles, not 5) |

`setPose()` re-anchors odometry but does not stop the DCMotorSim — the motor coasts for v₀·τ ≈ 0.1 m.

---

## Layer 3 — Physics integration tests

`build()` uses IronMaple (dyn4j). Every Layer 3 test must follow the strict per-cycle call order:

```java
drive.runVelocity(speeds);    // 1. queue voltage commands
drive.simulationPeriodic();   // 2. advance dyn4j: 5 sub-ticks × 4 ms = 20 ms
drive.periodic();             // 3. read updated module caches → odometry
stepOneCycle();               // 4. advance FPGA clock 20 ms
```

The `step()` helper in `AkitSwerveDriveSimPhysicsTest` wraps steps 2–4.

**Wrong order = stale data.** `periodic()` reads IronMaple module position caches. Those caches are only filled by `simulationPeriodic()` sub-ticks. Calling `periodic()` first reads the initial zero-filled caches and no motion appears.

### Teardown — `SimulatedArena` singleton

`SimulatedArena` is a static singleton. Every `build()` call registers a physics body into it. Without cleanup, bodies accumulate across tests.

```java
@AfterEach
public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
    f.setAccessible(true);
    f.set(null, null);     // null → next test gets a fresh Arena2026Rebuilt
    RobotMode.resetForTesting();
    super.simTeardown();
}
```

### Thresholds (Layer 3)

| Motion | Expected after 50 cycles (1 s at 1 m/s or π rad/s) |
|--------|-----------------------------------------------------|
| Forward 1 m/s | X > 0.1 m |
| Strafe 1 m/s | Y > 0.05 m (modules must rotate 90° first — costs ~10 cycles) |
| Rotate π rad/s | \|heading\| > 0.05 rad |
| Initial pose heading | tolerance 1e-4 rad (not 1e-6 — dyn4j has sub-micro-radian noise) |

Strafe threshold is lower than forward because modules start facing forward (0°) and must steer to 90° before contributing lateral motion.

---

## Layer 4 — Visual test

Runs as a full robot program with the automated `SwerveVisualTest` command sequence:

1. Drive forward 1 m (1 s at 1 m/s)
2. Strafe left 0.5 m
3. Rotate 90°
4. Drive in the alliance-correct direction
5. Approach the field boundary and stop

```powershell
.\gradlew.bat simulateJava -PvisualTest
```

Watch the robot in AdvantageScope or the Glass **Field2d** widget. The sequence never runs in CI — it is a manual sanity check.

---

## Regression workflow (agent checklist)

After any change to `src/main/java/org/frc5010/common/`:

```powershell
.\gradlew.bat test
```

All tests must pass before committing. If a test fails, diagnose the failure in the HTML report (`build/reports/tests/test/index.html`) before proceeding. Never disable or weaken an assertion to make a test pass — fix the root cause.

To force a full re-run even if Gradle thinks nothing changed:

```powershell
.\gradlew.bat cleanTest test
```

---

## Team-specific tests

Teams extending this library can add their own test classes alongside the library tests. The recommended location is `src/test/java/frc/robot/` — Gradle picks up all test classes under `src/test/java/` automatically.

Team tests use the same infrastructure as library tests:

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.*;

class MyRobotTest extends SimTestBase {

  // Use your real robot's constants so the test matches real behavior
  private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
      .moduleType(ModuleType.SIM)   // SIM regardless — hardware IO not available in tests
      .gyroType(GyroType.SIM)
      .trackWidth(Inches.of(22.75)) // same values as RealRobotProfile
      .wheelBase(Inches.of(22.75))
      .wheelRadius(Inches.of(2.0))
      .build();

  private AkitSwerveDrive drive;

  @BeforeEach @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS);
  }

  @AfterEach @Override
  public void simTeardown() {
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  @Test
  void myRobotSpecificBehavior() {
    enableTeleop();
    // ... test something specific to your robot
  }
}
```

Use `buildWithoutPhysics()` for speed; use `build()` (+ the `SimulatedArena` teardown pattern from Layer 3) when you need IronMaple physics accuracy. See the `/new-sim-test` slash command for full skeletons.

---

## Log analysis

Every sim run writes a `.wpilog` file to `logs/` (set up in `Robot.java`'s `Logger` configuration). Replay runs write to `logs/<original>_sim.wpilog`.

### Quick summary (agent-readable)

```powershell
# Analyze the most recent log
.\gradlew.bat logSummary

# Analyze a specific log
.\gradlew.bat logSummary -PlogFile=logs/FRC_20260525_143022.wpilog
```

Output sections:
- **All Entries** — every logged signal with its type; use this to discover what's available
- **Numeric Statistics** — min/max/count for every `double` signal
- **Anomaly Flags** — loop overruns (> 25 ms), gyro disconnects, motor current spikes (> 60 A)

### Replay mode (re-run code against a recorded log)

```powershell
.\gradlew.bat replayWatch   # opens a file picker; select a .wpilog
```

The robot code re-runs against the log at full speed. Output is written to `<original>_sim.wpilog`. Open both in AdvantageScope to compare logged vs. re-computed state — useful for validating a code fix against a previously recorded failure.

### Key AdvantageKit signal paths

| Signal | Path in log |
|--------|------------|
| Robot pose (x, y, heading) | `RealOutputs/Drive/Pose` (`double[]`) |
| Chassis speeds (commanded) | `RealOutputs/Drive/ChassisSpeeds/...` |
| Module drive velocity | `RealOutputs/Drive/Module0DriveVelocityRadPerSec` … `Module3` |
| Module drive current | `RealOutputs/Drive/Module0DriveCurrentAmps` … `Module3` |
| Module steer angle | `RealOutputs/Drive/Module0TurnPosition` … `Module3` |
| Gyro connected | `RealOutputs/Drive/GyroConnected` |
| Gyro yaw | `RealOutputs/Drive/GyroYawPositionRad` |

Signal paths use the `@AutoLogOutput` key attribute set in `AkitSwerveDrive` and the `@AutoLog`-generated field names from `ModuleIOInputs` / `GyroIOInputs`.

### Adding a new test

See the `/new-sim-test` slash command in Claude Code for a step-by-step playbook with copy-paste skeletons for both Layer 2 and Layer 3.
