package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.TimeUnit;
import org.frc5010.common.mechanisms.Arm;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Range-of-motion exploration for the single-jointed {@link Arm}, focused on the back
 * half of the arc — angles past the top (90°, arm straight up, where the gravity
 * feedforward {@code kG·cos(θ)} crosses zero and then flips sign). The {@code Example*}
 * arms sweep −30°→210°, so 90°→210° is the "over the top" region where gravity reverses
 * relative to the direction of travel; the existing functional tests only ever command
 * 90°, so that region was previously unexercised.
 *
 * <p>Each test builds the real example arm (TalonFX IO + Phoenix sim state + WPILib
 * {@link edu.wpi.first.wpilibj.simulation.SingleJointedArmSim} + LQR/PID loop), commands
 * an angle, and asserts the arm actually settles there. Same Phoenix enable-watchdog
 * pumping as {@link MechanismsFunctionalTest}.
 */
@Timeout(value = 5, unit = TimeUnit.MINUTES)
public class ArmRangeOfMotionTest extends SimTestBase {

  /** Settle tolerance, degrees — matches MechanismsFunctionalTest's arm tests. */
  private static final double TOLERANCE_DEG = 5.0;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM); // mechanism IO selection requires an explicit mode
  }

  @AfterEach
  @Override
  public void simTeardown() {
    org.frc5010.common.mechanisms.MechanismVisuals3d.resetForTesting();
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  /** Runs the scheduler + sim for the given sim-time duration (pumps the Phoenix enable watchdog). */
  private void runScheduledFor(double seconds) {
    int cycles = (int) Math.round(seconds / LOOP_PERIOD_SECS);
    for (int i = 0; i < cycles; i++) {
      CommandScheduler.getInstance().run();
      DriverStationSim.notifyNewData();
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
   * Asserts the angle converges to {@code targetDeg} ± tolerance, simulating in 0.5 s
   * slices (up to {@code extraSeconds}) so a real-time scheduling hiccup doesn't flake it.
   */
  private void assertConverges(Arm arm, double targetDeg, double extraSeconds, String message) {
    double t = 0;
    while (Math.abs(arm.getAngle().in(Degrees) - targetDeg) > TOLERANCE_DEG && t < extraSeconds) {
      runScheduledFor(0.5);
      t += 0.5;
    }
    assertEquals(targetDeg, arm.getAngle().in(Degrees), TOLERANCE_DEG, message);
  }

  // --- LQR arm: settle at angles across the whole arc, including past the top ---

  @ParameterizedTest(name = "LQR arm settles at {0}°")
  @ValueSource(doubles = {-30, 0, 45, 90, 135, 180, 210})
  public void lqrArmSettlesAcrossFullRange(double targetDeg) {
    ExampleArm arm = new ExampleArm();
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(targetDeg)), 4.0);
      assertConverges(arm, targetDeg, 4.0,
          "LQR arm should settle at " + targetDeg + "° (top of arc = 90°)");
    } finally {
      arm.close();
    }
  }

  // --- PROFILED_PID arm: same sweep, onboard MotionMagic + Arm_Cosine gravity comp ---

  @ParameterizedTest(name = "Profiled arm settles at {0}°")
  @ValueSource(doubles = {-30, 0, 45, 90, 135, 180, 210})
  public void profiledArmSettlesAcrossFullRange(double targetDeg) {
    ExampleProfiledArm arm = new ExampleProfiledArm();
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(targetDeg)), 4.0);
      assertConverges(arm, targetDeg, 4.0,
          "PROFILED_PID arm should settle at " + targetDeg + "° (top of arc = 90°)");
    } finally {
      arm.close();
    }
  }

  // --- Continuous sweep that crosses the top of the arc in both directions ---

  @Test
  public void lqrArmSweepsUpOverTheTopAndBackDown() {
    ExampleArm arm = new ExampleArm();
    try {
      // Up and over the top: 0° → 90° (vertical) → 210° (well past, gravity reversed).
      scheduleAndRun(arm.goToAngle(Degrees.of(210)), 4.0);
      assertConverges(arm, 210, 4.0, "LQR arm should reach the upper limit over the top");

      // Back down across the top the other way: 210° → 90° → −30°.
      CommandScheduler.getInstance().schedule(arm.goToAngle(Degrees.of(-30)));
      runScheduledFor(4.0);
      assertConverges(arm, -30, 4.0, "LQR arm should return to the lower limit back over the top");
    } finally {
      arm.close();
    }
  }

  @Test
  public void profiledArmSweepsUpOverTheTopAndBackDown() {
    ExampleProfiledArm arm = new ExampleProfiledArm();
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(210)), 4.0);
      assertConverges(arm, 210, 4.0, "Profiled arm should reach the upper limit over the top");

      CommandScheduler.getInstance().schedule(arm.goToAngle(Degrees.of(-30)));
      runScheduledFor(4.0);
      assertConverges(arm, -30, 4.0, "Profiled arm should return to the lower limit back over the top");
    } finally {
      arm.close();
    }
  }

  // --- Does the arm overshoot the soft limit while driving over the top? ---

  @Test
  public void lqrArmDoesNotBlowPastTheUpperSoftLimitGoingOverTheTop() {
    ExampleArm arm = new ExampleArm();
    try {
      enableTeleop();
      CommandScheduler.getInstance().schedule(arm.goToAngle(Degrees.of(210)));
      double maxSeen = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < 400; i++) { // 8 s, sampling the peak every cycle
        runScheduledFor(LOOP_PERIOD_SECS);
        maxSeen = Math.max(maxSeen, arm.getAngle().in(Degrees));
      }
      // 210° upper soft limit; allow a small controller/physics overshoot margin.
      assertTrue(maxSeen <= 215,
          "LQR arm overshot the 210° upper limit going over the top: peaked at " + maxSeen + "°");
      assertConverges(arm, 210, 2.0, "LQR arm should still settle at the upper limit");
    } finally {
      arm.close();
    }
  }
}
