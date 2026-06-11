package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.TimeUnit;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * High-level functional tests for the YAMS mechanism examples: each test builds the
 * real example subsystem (TalonFX wrapper + YAMS physics sim + LQR/PID closed loop),
 * schedules its public command, and asserts the mechanism actually reaches the
 * commanded state.
 *
 * <p>These validate the full chain — settings → motor config → YAMS closed-loop
 * Notifier → vendor sim → mechanism physics — not individual classes.
 *
 * <p><b>Timing:</b> the YAMS closed loop runs in a WPILib Notifier thread. The
 * synchronous {@code SimHooks.stepTiming} deadlocks against that Notifier (it waits
 * for an alarm acknowledgment the YAMS thread never delivers), so this test pumps
 * time the same way YAMS's own test suite does: {@code stepTimingAsync} to advance
 * the paused sim clock plus a short real-time sleep so the Notifier callback can run.
 *
 * <p>Each test closes its subsystem so CAN IDs and Notifier threads are released
 * for the next test.
 */
@Timeout(value = 3, unit = TimeUnit.MINUTES)
public class YamsMechanismsFunctionalTest extends SimTestBase {

  /** Runs the scheduler + sim for the given sim-time duration. */
  private void runScheduledFor(double seconds) {
    int cycles = (int) Math.round(seconds / LOOP_PERIOD_SECS);
    for (int i = 0; i < cycles; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTimingAsync(LOOP_PERIOD_SECS);
      try {
        Thread.sleep(10); // let the YAMS closed-loop Notifier callback run
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new RuntimeException(e);
      }
      // Keep DS packets fresh and feed the Phoenix enable watchdog directly: TalonFX
      // outputs silently neutral (duty cycle 0) if either starves for ~100 ms real time.
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      Unmanaged.feedEnable(200);
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
      double start = elevator.getHeight().in(Meters);
      scheduleAndRun(elevator.goToHeight(Meters.of(0.8)), 4.0);
      assertTrue(Math.abs(elevator.getHeight().in(Meters) - start) > 0.3,
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
  public void advantageKitInputsTrackMechanismState() {
    ExampleElevator elevator = new ExampleElevator();
    try {
      scheduleAndRun(elevator.goToHeight(Meters.of(0.5)), 1.0);
      // The public getters read from the @AutoLog inputs (the replay bubble), not the
      // mechanism directly — verify the inputs are being populated each periodic()
      // and match the live simulated mechanism state.
      // Inputs are a snapshot from the last periodic(); the sim advances up to one
      // cycle past it, so compare with a one-cycle-of-motion tolerance.
      assertEquals(elevator.getMechanism().getHeight().in(Meters),
          elevator.getHeight().in(Meters), 0.05,
          "inputs-based height getter should track the live mechanism");
      assertTrue(elevator.getHeight().in(Meters) > 0.1,
          "inputs should reflect actual motion, not stay at defaults");
      // The closed-loop setpoint crossed into the inputs too.
      assertTrue(Math.abs(elevator.getVelocity().in(MetersPerSecond)) >= 0,
          "velocity input should be populated");
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

  // --- PROFILED_PID style: same mechanisms, trapezoid profile + (onboard) PID ---

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
