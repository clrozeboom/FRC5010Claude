package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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

/**
 * Probes the fused-CANcoder ±180° absolute-discontinuity question for the {@link Arm}.
 *
 * <p>The two tests separate the two cases that are easy to conflate:
 *
 * <ol>
 *   <li><b>Crossing the boundary while running</b> (this is safe). The arm is correctly
 *       initialized at 120°, drifts under gravity over the top and across 180° while
 *       disabled, then is commanded back. The CANcoder's {@code AbsolutePosition} wraps
 *       at the discontinuity, but the fused mechanism feedback tracks the continuous
 *       {@code Position} signal (extended by the rotor), so the angle stays continuous
 *       and the arm returns exactly — no 360° glitch, enabled or disabled.</li>
 *   <li><b>Booting parked past the boundary</b> (this is the real-hardware hazard the
 *       {@code AbsoluteSensorDiscontinuityPoint} fix targets). On a real robot the Talon
 *       seeds its fused position from the wrapping {@code AbsolutePosition}, so an arm
 *       powered on at 200° would seed ~−160°. The simulator does <em>not</em> reproduce
 *       this: {@link org.frc5010.common.mechanisms.MechanismIOTalonFXSim} seeds the
 *       continuous {@code Position} directly via {@code setRawPosition}, so it reads the
 *       true angle regardless. The second test pins that sim behaviour so the limitation
 *       is explicit — the fix is correct on hardware but is not sim-verifiable as the
 *       sim IO is written today.</li>
 * </ol>
 */
@Timeout(value = 5, unit = TimeUnit.MINUTES)
public class ArmCancoderDriftTest extends SimTestBase {

  private static final int MOTOR_ID = 51;
  private static final int CANCODER_ID = 52;

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

  private static Arm.Settings cancoderArm(String name, double startDeg) {
    var s = new Arm.Settings();
    s.name = name;
    s.canId = MOTOR_ID;
    s.cancoderId = CANCODER_ID;
    s.cancoderOffset = Degrees.of(0);
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 5};
    s.length = Meters.of(0.6);
    s.mass = Kilograms.of(4.0);
    s.minAngle = Degrees.of(-30);
    s.maxAngle = Degrees.of(210);
    s.startingAngle = Degrees.of(startDeg);
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

  private void assertConverges(Arm arm, double targetDeg, double tolDeg, double extraSeconds,
      String message) {
    double t = 0;
    while (Math.abs(arm.getAngle().in(Degrees) - targetDeg) > tolDeg && t < extraSeconds) {
      runScheduledFor(0.5);
      t += 0.5;
    }
    assertEquals(targetDeg, arm.getAngle().in(Degrees), tolDeg, message);
  }

  /** Case 1: drift across ±180° while disabled, then return — must stay continuous. */
  @Test
  public void driftOverTheTopWhileDisabledThenReturn() {
    Arm arm = new Arm(cancoderArm("DriftArm", 120));
    try {
      enableTeleop();
      runScheduledFor(0.2);
      assertEquals(120, arm.getAngle().in(Degrees), 4,
          "arm should initialize at 120° from the CANcoder");

      // Disabled, the top-heavy arm drifts under gravity up over 180° toward the 210° stop.
      disable();
      double peak = Double.NEGATIVE_INFINITY;
      double maxJump = 0;
      double prev = arm.getAngle().in(Degrees);
      for (int i = 0; i < 24; i++) { // 6 s of drift, sampled every 0.25 s
        runScheduledFor(0.25);
        double fused = arm.getAngle().in(Degrees);
        peak = Math.max(peak, fused);
        maxJump = Math.max(maxJump, Math.abs(fused - prev));
        prev = fused;
      }
      assertTrue(peak > 185,
          "precondition: disabled arm should drift up past 180°, peaked at " + peak + "°");
      // The fused feedback must stay continuous across the boundary — no ~360° glitch.
      assertTrue(maxJump < 90,
          "fused position glitched by " + maxJump + "° crossing the ±180° boundary");

      // Re-enable and command back to 120° — it must return exactly, not be ~360° off.
      enableTeleop();
      CommandScheduler.getInstance().schedule(arm.goToAngle(Degrees.of(120)));
      runScheduledFor(4.0);
      assertConverges(arm, 120, 6, 4.0,
          "arm should return to 120° after drifting over the top while disabled");
    } finally {
      arm.close();
    }
  }

  /**
   * Case 2: verifies the discontinuity fix. The arm auto-places the CANcoder's absolute
   * wrap opposite the travel midpoint (range [−90°, 270°) for −30°..210°), so the
   * absolute reading no longer wraps anywhere inside the travel. Parking at 210° (the
   * upper stop) now reads ≈ +0.583 rot instead of the −0.417 rot the Phoenix default
   * (±180°) would produce — which is exactly what keeps real-robot power-on seeding
   * unambiguous. The sim models the absolute signal faithfully (we watched it wrap in
   * case 1), so this sign is a genuine check of the fix, not just of the fused position.
   */
  @Test
  public void discontinuityPlacedOutsideTravelSoAbsoluteDoesNotWrap() {
    Arm arm = new Arm(cancoderArm("BootParkedArm", 210)); // parked at the upper stop
    CANcoder probe = new CANcoder(CANCODER_ID); // read-only handle to the same sim device
    try {
      enableTeleop();
      runScheduledFor(0.2);
      double fused = arm.getAngle().in(Degrees);
      double abs = probe.getAbsolutePosition().getValueAsDouble();
      // With the wrap moved out of the way, the absolute reading at 210° stays positive
      // (≈ 0.583 rot). Under the ±180° default it would have wrapped to ≈ −0.417 rot.
      assertTrue(abs > 0.5,
          "discontinuity should be placed so 210° does not wrap; abs was " + abs + " rot");
      assertEquals(210.0 / 360.0, abs, 0.02,
          "absolute reading should equal the true mechanism angle, not a wrapped value");
      assertEquals(210, fused, 6, "fused position should read the true angle");
    } finally {
      probe.close();
      arm.close();
    }
  }
}
