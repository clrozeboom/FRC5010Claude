# /new-sim-test — Add a simulation test to FRC5010Claude

Use this playbook when adding a new Layer 2 (DCMotorSim) or Layer 3 (IronMaple physics) test to `AkitSwerveDrive`.

---

## Step 0 — Decide which layer

| Question | Answer → Layer |
|----------|---------------|
| Testing a specific subsystem behavior (pose, velocity, disabled safety)? | **Layer 2** |
| Testing that IronMaple physics advances correctly, or physics-specific behavior? | **Layer 3** |
| Unit-testing `SwerveConstants`, `SwerveFactory` mode selection, or `TunableGains`? | **Layer 1** (no SimTestBase needed) |

---

## Layer 2 — DCMotorSim test skeleton

File: `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveTest.java`
(add the `@Test` method to the existing class)

```java
@Test
void myNewBehavior() {
    // Optional: enable teleop if the robot needs to be enabled
    enableTeleop();

    for (int i = 0; i < 50; i++) {           // 50 × 20 ms = 1 simulated second
        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
        drive.periodic();                      // reads DCMotorSim state → odometry
        stepOneCycle();                        // advance FPGA clock 20 ms
    }

    Pose2d pose = drive.getPose();
    // assert something about pose.getX(), pose.getY(), pose.getRotation()
}
```

**No** `drive.simulationPeriodic()` — `ModuleIOSim` updates itself inside `periodic()`.

### Threshold guidance for Layer 2
- Forward 1 m/s for 1 s → X > **0.1 m** (DCMotorSim spins up in a few cycles)
- Strafe 1 m/s for 1 s → Y > **0.1 m** (modules start at correct angle in `buildWithoutPhysics`)
- Rotation π rad/s for 1 s → |heading| > **0.1 rad**
- After `setPose()`, always run **50** coast-down cycles (not 5): DCMotorSim keeps internal velocity; coast distance ≈ 0.1 m; use tolerance **< 0.15 m**

---

## Layer 3 — IronMaple physics test skeleton

File: `src/test/java/org/frc5010/common/subsystem/AkitSwerveDriveSimPhysicsTest.java`
(add the `@Test` method to the existing class)

```java
@Test
void myNewPhysicsBehavior() {
    enableTeleop();

    for (int i = 0; i < 50; i++) {
        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
        step();   // simulationPeriodic → periodic → stepOneCycle
    }

    // assert on drive.getPose(), drive.getRotation(), etc.
}
```

The `step()` helper is already defined in `AkitSwerveDriveSimPhysicsTest`:
```java
private void step() {
    drive.simulationPeriodic();  // advance dyn4j (5 × 4 ms sub-ticks)
    drive.periodic();            // read updated caches → odometry
    stepOneCycle();              // tick FPGA 20 ms
}
```

### Threshold guidance for Layer 3
- Forward 1 m/s for 1 s → X > **0.1 m**
- Strafe 1 m/s for 1 s → Y > **0.05 m** (modules must rotate 90° first, costs ~10 cycles)
- Rotation π rad/s for 1 s → |heading| > **0.05 rad**
- Initial pose heading tolerance: **1e-4** (not 1e-6 — dyn4j has sub-micro-radian noise)

---

## Creating a brand-new test class

Only needed if you're adding a completely new subsystem. Copy this shell:

```java
package org.frc5010.common.subsystem;

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
// Layer 3 only:
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

class MyNewTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private AkitSwerveDrive drive;

  @BeforeEach @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS);  // or build() for Layer 3
  }

  @AfterEach @Override
  public void simTeardown() {
    // ---- Layer 3 only: reset SimulatedArena singleton ----
    // SimulatedArena.getInstance().shutDown();
    // try {
    //   java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
    //   f.setAccessible(true);
    //   f.set(null, null);
    // } catch (ReflectiveOperationException e) {
    //   throw new RuntimeException("Failed to reset SimulatedArena", e);
    // }
    // ---- end Layer 3 block ----
    RobotMode.resetForTesting();
    super.simTeardown();  // cancels commands, unregisters subsystems, resumes timing
  }

  @Test
  void exampleTest() {
    drive.periodic();
    assertEquals(0.0, drive.getPose().getX(), 1e-6);
  }
}
```

---

## Checklist before running

- [ ] `@BeforeEach` calls `super.simSetup()` first, `@AfterEach` calls `super.simTeardown()` last
- [ ] `RobotMode.set(Mode.SIM)` before constructing drive
- [ ] `RobotMode.resetForTesting()` in teardown
- [ ] Layer 3 only: `SimulatedArena` singleton reset in teardown (see template above)
- [ ] Layer 3 only: call order is `runVelocity → simulationPeriodic → periodic → stepOneCycle`
- [ ] Disabled tests: no `enableTeleop()` call; `DriverStation.isDisabled()` causes `periodic()` to call `module.stop()`
- [ ] Thresholds match layer (see guidance above)

---

## Team-specific tests (in frc.robot package)

Tests for robot-specific behavior (your game piece logic, auto routines, etc.) belong in
`src/test/java/frc/robot/`. They use exactly the same `SimTestBase` infrastructure:

```java
package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.*;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.*;

class MyRobotTest extends SimTestBase {

  // Match your real robot's geometry so the sim reflects actual behavior
  private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
      .moduleType(ModuleType.SIM)
      .gyroType(GyroType.SIM)
      .trackWidth(Inches.of(22.75))
      .wheelBase(Inches.of(22.75))
      .wheelRadius(Inches.of(2.0))
      .build();

  private AkitSwerveDrive drive;

  @BeforeEach @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS); // or build() for Layer 3
  }

  @AfterEach @Override
  public void simTeardown() {
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  @Test
  void myTeamBehavior() {
    enableTeleop();
    // ...
  }
}
```

For Layer 3 (IronMaple physics), add the `SimulatedArena` teardown — see the Layer 3 skeleton above.

---

## Regression gate — run before every commit

```powershell
# Windows PowerShell (always use this — WSL cannot reach C:\workspace)
cd C:\workspace\FRC5010Claude
.\gradlew.bat test

# Or the VS Code task: Ctrl+Shift+P → "Tasks: Run Test Task" → "Run Unit Tests"
```

All tests must pass. Fix root causes — never weaken assertions. If Gradle skips tests because nothing changed, use `.\gradlew.bat cleanTest test`.

Test report: `build/reports/tests/test/index.html`
