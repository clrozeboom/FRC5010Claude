package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.TimeUnit;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * High-level functional tests for the mechanism examples: each test builds the real
 * example subsystem (TalonFX IO + Phoenix sim state + WPILib physics sim + LQR/PID
 * closed loop), schedules its public command, and asserts the mechanism actually
 * reaches the commanded state.
 *
 * <p>These validate the full chain — settings → TalonFX config → controller →
 * sim-state physics — not individual classes. The closed loop runs synchronously in
 * {@code periodic()}, so plain deterministic {@code stepOneCycle()} stepping works
 * (no Notifier threads). The Phoenix enable watchdog still applies: feed
 * {@code DriverStationSim.notifyNewData()} + {@code Unmanaged.feedEnable(...)} every
 * cycle or TalonFX outputs silently neutral after ~100 ms of real time.
 *
 * <p>Each test closes its subsystem so CAN IDs are released for the next test.
 */
@Timeout(value = 3, unit = TimeUnit.MINUTES)
public class MechanismsFunctionalTest extends SimTestBase {

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM); // mechanism IO selection requires an explicit mode
  }

  @AfterEach
  @Override
  public void simTeardown() {
    // Fresh shared canvas per test — re-constructing a same-named mechanism would
    // otherwise collide with ligaments left on the old canvas.
    org.frc5010.common.mechanisms.MechanismVisuals.resetForTesting();
    org.frc5010.common.mechanisms.MechanismVisuals3d.resetForTesting();
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  /** Runs the scheduler + sim for the given sim-time duration. */
  private void runScheduledFor(double seconds) {
    int cycles = (int) Math.round(seconds / LOOP_PERIOD_SECS);
    for (int i = 0; i < cycles; i++) {
      CommandScheduler.getInstance().run();
      // Keep DS packets fresh and feed the Phoenix enable watchdog directly: TalonFX
      // outputs silently neutral (duty cycle 0) if either starves for ~100 ms real time.
      DriverStationSim.notifyNewData();
      // Feed the Phoenix enable watchdog only while actually enabled — feeding it
      // when disabled would override the DS-disable neutral and keep the last
      // control request running.
      if (edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
        Unmanaged.feedEnable(200);
      }
      try {
        Thread.sleep(10); // the simulated TalonFX processes controls on a real-time thread
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new RuntimeException(e);
      }
      stepOneCycle();
    }
  }

  private void scheduleAndRun(Command command, double seconds) {
    enableTeleop();
    CommandScheduler.getInstance().schedule(command);
    runScheduledFor(seconds);
  }

  /**
   * Asserts the value converges to {@code target} ± {@code tolerance}: checks after the
   * initial run, then keeps simulating in 0.5 s slices (up to {@code extraSeconds}) so a
   * real-time scheduling hiccup that shifts the settle phase doesn't flake the test.
   */
  private void assertConverges(java.util.function.DoubleSupplier value, double target,
      double tolerance, double extraSeconds, String message) {
    double t = 0;
    while (Math.abs(value.getAsDouble() - target) > tolerance && t < extraSeconds) {
      runScheduledFor(0.5);
      t += 0.5;
    }
    assertEquals(target, value.getAsDouble(), tolerance, message);
  }

  @Test
  public void elevatorReachesCommandedHeight() {
    ExampleElevator elevator = new ExampleElevator();
    try {
      scheduleAndRun(elevator.goToHeight(Meters.of(0.8)), 4.0);
      assertTrue(Math.abs(elevator.getHeight().in(Meters) - 0.1) > 0.3,
          "elevator should have moved from its starting height");
      assertConverges(() -> elevator.getHeight().in(Meters), 0.8, 0.05, 4.0,
          "elevator should settle at the commanded height");
    } finally {
      elevator.close();
    }
  }

  @Test
  public void elevatorRetunesLiveAndStillConverges() {
    ExampleElevator elevator = new ExampleElevator();
    try {
      // Soften the controller mid-flight via the NT tuning entry (less allowed effort).
      NetworkTableInstance.getDefault()
          .getTable("/Tuning/ExampleElevator")
          .getEntry("lqr_relms")
          .setDouble(6.0);
      scheduleAndRun(elevator.goToHeight(Meters.of(0.6)), 4.0);
      assertConverges(() -> elevator.getHeight().in(Meters), 0.6, 0.05, 4.0,
          "elevator should still converge after a live LQR retune");
    } finally {
      elevator.close();
    }
  }

  @Test
  public void advantageKitInputsTrackMechanismState() {
    ExampleElevator elevator = new ExampleElevator();
    try {
      // Sample mid-travel (the move takes ~0.7 s) so velocity is meaningfully nonzero.
      scheduleAndRun(elevator.goToHeight(Meters.of(0.5)), 0.5);
      // The public getters read from the @AutoLog IO inputs (the replay bubble) —
      // verify they reflect real motion, not defaults.
      assertTrue(elevator.getHeight().in(Meters) > 0.15,
          "inputs should reflect actual motion, not stay at defaults");
      assertTrue(elevator.getVelocity().in(MetersPerSecond) > 0.05,
          "velocity input should be populated while climbing");
    } finally {
      elevator.close();
    }
  }

  @Test
  public void characterizedPlantElevatorConverges() {
    // Same elevator as ExampleElevator, but the LQR plant comes from measured kV/kA
    // (SysId) instead of carriage mass — proving mass need not be known directly.
    ExampleCharacterizedElevator elevator = new ExampleCharacterizedElevator();
    try {
      scheduleAndRun(elevator.goToHeight(Meters.of(0.8)), 4.0);
      assertConverges(() -> elevator.getHeight().in(Meters), 0.8, 0.05, 4.0,
          "characterized-plant (kV/kA) elevator should settle at the commanded height");
    } finally {
      elevator.close();
    }
  }

  @Test
  public void armReachesCommandedAngle() {
    ExampleArm arm = new ExampleArm();
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(90)), 4.0);
      assertConverges(() -> arm.getAngle().in(Degrees), 90, 5, 4.0,
          "arm should settle at the commanded angle");
    } finally {
      arm.close();
    }
  }

  @Test
  public void turretReachesCommandedAngle() {
    ExampleTurret turret = new ExampleTurret();
    try {
      scheduleAndRun(turret.goToAngle(Degrees.of(90)), 4.0);
      assertConverges(() -> turret.getAngle().in(Degrees), 90, 5, 4.0,
          "turret should settle at the commanded angle");
    } finally {
      turret.close();
    }
  }

  @Test
  public void shooterReachesCommandedSpeed() {
    ExampleShooter shooter = new ExampleShooter();
    try {
      scheduleAndRun(shooter.goToSpeed(RPM.of(3000)), 4.0);
      assertConverges(() -> shooter.getSpeed().in(RPM), 3000, 150, 4.0,
          "shooter should settle at the commanded speed");
    } finally {
      shooter.close();
    }
  }

  @Test
  public void elevatorHomingZerosAtBottom() {
    ExampleElevator elevator = new ExampleElevator();
    try {
      enableTeleop();
      // Start at 0.1 m; homing drives into the bottom hard stop (soft limits
      // temporarily disabled), detects the debounced current spike, and re-seeds the
      // sensor to minHeight.
      var home = elevator.homeCommand();
      CommandScheduler.getInstance().schedule(home);
      double t = 0;
      while (CommandScheduler.getInstance().isScheduled(home) && t < 6.0) {
        runScheduledFor(0.25);
        t += 0.25;
      }
      assertTrue(t < 6.0, "homing should detect the hard-stop current spike and finish");
      assertEquals(0.0, elevator.getHeight().in(Meters), 0.03,
          "after homing, the sensor should read the configured minimum height");
    } finally {
      elevator.close();
    }
  }

  @Test
  public void turretWithFusedCancoderConverges() {
    // ExampleProfiledTurret runs on a fused CANcoder (absolute, 1:1 on the mechanism);
    // onboard MotionMagic consumes the fused sensor natively.
    ExampleProfiledTurret turret = new ExampleProfiledTurret();
    try {
      scheduleAndRun(turret.goToAngle(Degrees.of(-45)), 3.0);
      assertConverges(() -> turret.getAngle().in(Degrees), -45, 5, 4.0,
          "fused-CANcoder turret should settle at the commanded angle");
    } finally {
      turret.close();
    }
  }

  @Test
  public void clearGoalOnDisableDropsTheGoal() {
    // Same elevator as ExampleElevator but with clearGoalOnDisable = true: after a
    // disable/enable cycle the mechanism must NOT resume driving to the stale goal.
    var s = new org.frc5010.common.mechanisms.Elevator.Settings();
    s.name = "ClearGoalElevator";
    s.canId = 36;
    s.startingHeight = Meters.of(0.1);
    s.kG = edu.wpi.first.units.Units.Volts.of(0.19);
    s.clearGoalOnDisable = true;
    var elevator = new org.frc5010.common.mechanisms.Elevator(s);
    try {
      scheduleAndRun(elevator.goToHeight(Meters.of(0.8)), 1.0); // climbing
      assertTrue(elevator.getHeight().in(Meters) > 0.2,
          "elevator should be climbing before the disable");

      // Disable and let the carriage coast to rest (brake mode), then measure.
      disable();
      runScheduledFor(0.5);
      double afterDisable = elevator.getHeight().in(Meters);

      enableTeleop();
      runScheduledFor(1.5);

      assertTrue(elevator.getHeight().in(Meters) <= afterDisable + 0.02,
          "with clearGoalOnDisable, re-enabling must not resume driving to the stale goal");
    } finally {
      elevator.close();
    }
  }

  // --- PROFILED_PID style: same mechanisms, trapezoid profile + onboard control ---

  @Test
  public void profiledElevatorReachesCommandedHeight() {
    ExampleProfiledElevator elevator = new ExampleProfiledElevator();
    try {
      scheduleAndRun(elevator.goToHeight(Meters.of(0.8)), 4.0);
      assertConverges(() -> elevator.getHeight().in(Meters), 0.8, 0.05, 4.0,
          "profiled-PID elevator should settle at the commanded height");
    } finally {
      elevator.close();
    }
  }

  @Test
  public void profiledArmReachesCommandedAngle() {
    ExampleProfiledArm arm = new ExampleProfiledArm();
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(90)), 4.0);
      assertConverges(() -> arm.getAngle().in(Degrees), 90, 5, 4.0,
          "profiled-PID arm should settle at the commanded angle");
    } finally {
      arm.close();
    }
  }

  @Test
  public void profiledTurretReachesCommandedAngle() {
    ExampleProfiledTurret turret = new ExampleProfiledTurret();
    try {
      scheduleAndRun(turret.goToAngle(Degrees.of(90)), 4.0);
      assertConverges(() -> turret.getAngle().in(Degrees), 90, 5, 4.0,
          "profiled-PID turret should settle at the commanded angle");
    } finally {
      turret.close();
    }
  }

  @Test
  public void profiledShooterReachesCommandedSpeed() {
    ExampleProfiledShooter shooter = new ExampleProfiledShooter();
    try {
      scheduleAndRun(shooter.goToSpeed(RPM.of(3000)), 4.0);
      assertConverges(() -> shooter.getSpeed().in(RPM), 3000, 150, 4.0,
          "profiled-PID shooter should settle at the commanded speed");
    } finally {
      shooter.close();
    }
  }

  @Test
  public void doubleJointedArmReachesJointAngles() {
    ExampleDoubleJointedArm arm = new ExampleDoubleJointedArm();
    try {
      scheduleAndRun(arm.goToAngles(Degrees.of(90), Degrees.of(0)), 5.0);
      assertConverges(() -> arm.getLowerAngle().in(Degrees), 90, 10, 4.0,
          "shoulder should settle at the commanded angle");
      assertConverges(() -> arm.getUpperAngle().in(Degrees), 0, 10, 4.0,
          "elbow should settle at the commanded angle");
    } finally {
      arm.close();
    }
  }

  @Test
  public void differentialWristReachesTiltAndTwist() {
    ExampleDifferentialWrist wrist = new ExampleDifferentialWrist();
    try {
      scheduleAndRun(wrist.goToAngles(Degrees.of(45), Degrees.of(30)), 5.0);
      assertConverges(() -> wrist.getTilt().in(Degrees), 45, 10, 4.0,
          "wrist tilt should settle at the commanded angle");
      assertConverges(() -> wrist.getTwist().in(Degrees), 30, 10, 4.0,
          "wrist twist should settle at the commanded angle");
    } finally {
      wrist.close();
    }
  }
}
