package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.mechanisms.Arm;

/**
 * The intake hopper arm — a {@link Arm} that also exposes the homing signals the intake's
 * deploy/zeroing logic needs (the real hopper has no absolute encoder, so it homes against
 * the deployed hard stop at 0°).
 *
 * <p>Adds: stator-current and velocity getters for stall detection, an open-loop
 * "drive into the hard stop" duty command, and an encoder-reset (zero) helper. The
 * {@link Arm}'s protected {@code inputs}/{@code positionNative()}/{@code velocityNative()}
 * are visible here as a subclass.
 */
public class HopperArm extends Arm {

  public HopperArm(Settings settings) {
    super(settings);
  }

  /** Hopper stator current (amps) — high while pressed into the hard stop. */
  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  /** Hopper angular velocity, degrees/second. */
  public double getVelocityDegPerSec() {
    return Math.toDegrees(velocityNative());
  }

  /** Hopper angle, degrees (0° = deployed hard stop, 120° = retracted). */
  public double getAngleDegrees() {
    return getAngle().in(Degrees);
  }

  /**
   * Re-seeds the encoder to {@code angle} (homing). Declares the hopper's current physical
   * position — used after driving into the deployed hard stop, and by the periodic
   * auto-rezero.
   */
  public void resetEncoder(Angle angle) {
    // Arm native = radians; the IO boundary is mechanism rotations (1 rotation = 360°),
    // so the target angle in rotations is exactly the sensor position to seed.
    io.setSensorPosition(angle.in(edu.wpi.first.units.Units.Rotations));
  }

  /**
   * Open-loop command: drive the hopper down at {@code duty} (negative = toward the
   * deployed hard stop) until {@code untilHardStop} reports the stop, with a timeout.
   * Marks the output as externally driven so the mechanism's profile stands down.
   */
  public Command driveDownUntilHardStop(double duty, java.util.function.BooleanSupplier untilHardStop) {
    return setDutyCycle(duty)
        .until(untilHardStop)
        .withName(getName() + " HomeDown");
  }

  /**
   * Whether the hopper is at a hard stop now: low movement velocity AND stall current
   * above {@code stallCurrentThreshold} amps.
   */
  public boolean atHardStop(double movingVelocityThresholdDegPerSec, double stallCurrentThreshold) {
    return Math.abs(getVelocityDegPerSec()) < movingVelocityThresholdDegPerSec
        && getStatorCurrentAmps() > stallCurrentThreshold;
  }

  /** Whether the hopper is moving faster than {@code thresholdDegPerSec}. */
  public boolean isMoving(double thresholdDegPerSec) {
    return Math.abs(getVelocityDegPerSec()) > thresholdDegPerSec;
  }

  /** Convenience: an idle command that holds the current goal (used as a default). */
  public Command holdCommand() {
    return Commands.idle(this).withName(getName() + " Hold");
  }

  /** The hopper's configured upper (retracted) limit. */
  public Angle retractedAngle() {
    return getSettings().maxAngle;
  }

  /** The hopper profile cruise velocity, deg/s — for reference by callers. */
  public double cruiseVelDegPerSec() {
    return getSettings().maxVelocity.in(DegreesPerSecond);
  }
}
