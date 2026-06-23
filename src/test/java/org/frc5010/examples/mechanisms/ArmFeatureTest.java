package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.TimeUnit;
import org.frc5010.common.mechanisms.Arm;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Feature-coverage exploration for {@link Arm} in simulation, beyond the basic
 * "go to 90°" functional test. Each test builds a real {@link Arm} (its own CAN IDs)
 * with one feature turned on — soft limits, a fused CANcoder, a follower motor, the
 * {@code sysId()} routine, open-loop duty cycle, {@code clearGoalOnDisable}, a
 * characterized (kV/kA) plant, and the {@code isAtAngle} trigger — and asserts the
 * full TalonFX-sim chain behaves. The aim is to surface issues that only appear when
 * these less-travelled paths run, not to re-verify the happy path.
 */
@Timeout(value = 8, unit = TimeUnit.MINUTES)
public class ArmFeatureTest extends SimTestBase {

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    org.frc5010.common.mechanisms.MechanismVisuals3d.resetForTesting();
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  /** A 0.6 m / 4 kg / 50:1 Kraken arm (same physical arm as ExampleArm) on a given CAN ID. */
  private static Arm.Settings baseArm(String name, int canId) {
    var s = new Arm.Settings();
    s.name = name;
    s.canId = canId;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 5};
    s.length = Meters.of(0.6);
    s.mass = Kilograms.of(4.0);
    s.minAngle = Degrees.of(-30);
    s.maxAngle = Degrees.of(210);
    s.startingAngle = Degrees.of(0);
    s.kG = Volts.of(0.40);
    s.statorCurrentLimit = Amps.of(40);
    return s;
  }

  private void runScheduledFor(double seconds) {
    int cycles = (int) Math.round(seconds / LOOP_PERIOD_SECS);
    for (int i = 0; i < cycles; i++) {
      CommandScheduler.getInstance().run();
      DriverStationSim.notifyNewData();
      if (edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
        Unmanaged.feedEnable(200);
      }
      try {
        Thread.sleep(10);
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

  private void assertConverges(Arm arm, double targetDeg, double tolDeg, double extraSeconds,
      String message) {
    double t = 0;
    while (Math.abs(arm.getAngle().in(Degrees) - targetDeg) > tolDeg && t < extraSeconds) {
      runScheduledFor(0.5);
      t += 0.5;
    }
    assertEquals(targetDeg, arm.getAngle().in(Degrees), tolDeg, message);
  }

  // --- Soft limits: commanding beyond maxAngle must not drive past it ---

  @Test
  public void softLimitHoldsAtUpperBoundWhenCommandedBeyond() {
    Arm arm = new Arm(baseArm("SoftLimitArm", 41));
    try {
      enableTeleop();
      CommandScheduler.getInstance().schedule(arm.goToAngle(Degrees.of(260))); // 50° past max
      double maxSeen = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < 350; i++) {
        runScheduledFor(LOOP_PERIOD_SECS);
        maxSeen = Math.max(maxSeen, arm.getAngle().in(Degrees));
      }
      assertTrue(maxSeen <= 215,
          "soft limit should hold the arm at ~210° even when commanded to 260°, saw " + maxSeen);
      assertConverges(arm, 210, 6, 2.0,
          "arm should rest against the 210° soft limit, not oscillate or fall away");
    } finally {
      arm.close();
    }
  }

  // --- Fused CANcoder: 1:1 absolute encoder, travel that crosses ±180° ---

  @Test
  public void fusedCancoderArmConvergesAcrossThe180Boundary() {
    var s = baseArm("CancoderArm", 42);
    s.cancoderId = 43;
    s.cancoderOffset = Degrees.of(0);
    Arm arm = new Arm(s);
    try {
      // Start should read ~0° from the absolute sensor with no seeding.
      enableTeleop();
      runScheduledFor(0.1);
      assertEquals(0, arm.getAngle().in(Degrees), 3,
          "fused CANcoder should report the starting angle without seeding");
      // 200° is past the 180° (0.5-rotation) absolute-encoder discontinuity.
      scheduleAndRun(arm.goToAngle(Degrees.of(200)), 4.0);
      assertConverges(arm, 200, 6, 4.0,
          "fused-CANcoder arm should settle at 200° (past the ±180° absolute boundary)");
    } finally {
      arm.close();
    }
  }

  // --- Follower motor on the same gearbox ---

  @Test
  public void followerArmConverges() {
    var s = baseArm("FollowerArm", 44);
    s.followerCanId = 45;
    s.followerOpposed = false;
    Arm arm = new Arm(s);
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(120)), 4.0);
      assertConverges(arm, 120, 6, 4.0,
          "arm with a follower TalonFX should settle at the commanded angle");
    } finally {
      arm.close();
    }
  }

  // --- Characterized (measured kV/kA) plant instead of mass/length ---

  @Test
  public void characterizedPlantArmConverges() {
    var s = baseArm("CharacterizedArm", 46);
    s.characterizedKv = 6.0;  // V per mechanism rot/s (12 V ÷ 2 rot/s free speed)
    s.characterizedKa = 0.10; // V per mechanism rot/s² (≈ R·J/(G·kT)·2π for this arm)
    Arm arm = new Arm(s);
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(135)), 4.0);
      assertConverges(arm, 135, 6, 4.0,
          "characterized-plant arm should settle at the commanded angle over the top");
    } finally {
      arm.close();
    }
  }

  // --- Open-loop duty cycle, both directions ---

  @Test
  public void dutyCycleDrivesArmBothDirections() {
    Arm arm = new Arm(baseArm("DutyCycleArm", 47));
    try {
      // Positive duty cycle should raise the arm off horizontal against gravity.
      scheduleAndRun(arm.setDutyCycle(0.5), 1.5);
      double raised = arm.getAngle().in(Degrees);
      assertTrue(raised > 10, "positive duty cycle should raise the arm, saw " + raised + "°");
      CommandScheduler.getInstance().cancelAll();

      // Negative duty cycle should lower it back down.
      scheduleAndRun(arm.setDutyCycle(-0.5), 1.5);
      double lowered = arm.getAngle().in(Degrees);
      assertTrue(lowered < raised - 10,
          "negative duty cycle should lower the arm from " + raised + "°, saw " + lowered + "°");
    } finally {
      arm.close();
    }
  }

  // --- clearGoalOnDisable: must not resume a stale goal after a disable/enable cycle ---

  @Test
  public void clearGoalOnDisableDropsTheGoal() {
    var s = baseArm("ClearGoalArm", 48);
    s.clearGoalOnDisable = true;
    Arm arm = new Arm(s);
    try {
      scheduleAndRun(arm.goToAngle(Degrees.of(150)), 0.6); // mid-flight
      double climbing = arm.getAngle().in(Degrees);
      assertTrue(climbing > 20, "arm should be climbing before the disable, saw " + climbing + "°");

      disable();
      runScheduledFor(0.5);
      double afterDisable = arm.getAngle().in(Degrees);

      enableTeleop();
      runScheduledFor(1.5);
      // With the goal cleared, re-enabling must not resume driving toward 150°.
      assertTrue(arm.getAngle().in(Degrees) <= afterDisable + 8,
          "with clearGoalOnDisable, re-enabling must not resume the stale 150° goal");
    } finally {
      arm.close();
    }
  }

  // --- isAtAngle trigger reflects real state ---

  @Test
  public void isAtAngleTriggerTracksArrival() {
    Arm arm = new Arm(baseArm("IsAtAngleArm", 49));
    try {
      var atTarget = arm.isAtAngle(Degrees.of(90), Degrees.of(4));
      enableTeleop();
      runScheduledFor(0.1);
      assertFalse(atTarget.getAsBoolean(), "trigger should be false before the arm moves");
      scheduleAndRun(arm.goToAngle(Degrees.of(90)), 4.0);
      assertConverges(arm, 90, 4, 4.0, "arm should reach 90° for the trigger check");
      assertTrue(atTarget.getAsBoolean(), "isAtAngle should be true once the arm arrives");
    } finally {
      arm.close();
    }
  }

  // --- sysId routine: runs all four segments to the travel limits and finishes ---

  @Test
  public void sysIdRoutineRunsAndFinishes() {
    Arm arm = new Arm(baseArm("SysIdArm", 50));
    try {
      enableTeleop();
      var sysId = arm.sysId();
      CommandScheduler.getInstance().schedule(sysId);
      double t = 0;
      while (CommandScheduler.getInstance().isScheduled(sysId) && t < 90.0) {
        runScheduledFor(0.5);
        t += 0.5;
      }
      assertFalse(CommandScheduler.getInstance().isScheduled(sysId),
          "sysId routine should reach the travel limits and finish, not hang");
    } finally {
      arm.close();
    }
  }
}
