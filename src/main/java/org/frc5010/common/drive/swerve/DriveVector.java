package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.DoubleSupplier;

/**
 * Combines two joystick axes into a 2-D drive vector with optional unit-circle normalization.
 *
 * <p>Without normalization the X and Y components are returned as-is. With
 * {@link #unitCircle()}, the combined vector is clamped so its magnitude never exceeds 1.
 * This prevents diagonal inputs from producing speeds greater than the robot's maximum —
 * important for swerve drive where unscaled diagonal inputs would reach magnitude √2.
 *
 * <p>Example — swerve translation axes with unit-circle normalization, then scaled to
 * physical speed:
 * <pre>{@code
 * DriveVector translate = DriveVector.of(
 *     controller.axis(1).negate().deadzone(0.05),   // forward (X)
 *     controller.axis(0).negate().deadzone(0.05)    // strafe  (Y)
 * ).unitCircle();
 *
 * // Inside the drive command lambda:
 * Translation2d xy = translate.get();
 * double vx = xy.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond);
 * double vy = xy.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond);
 * }</pre>
 */
public class DriveVector {

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private boolean normalize;

  private DriveVector(DoubleSupplier x, DoubleSupplier y) {
    this.xSupplier = x;
    this.ySupplier = y;
  }

  /**
   * Creates a drive vector from two axis suppliers.
   *
   * @param x the first axis (maps to {@link Translation2d#getX()})
   * @param y the second axis (maps to {@link Translation2d#getY()})
   */
  public static DriveVector of(DoubleSupplier x, DoubleSupplier y) {
    return new DriveVector(x, y);
  }

  /**
   * Enable unit-circle normalization: if the combined magnitude exceeds 1.0 both components
   * are scaled down proportionally so the magnitude equals exactly 1.0.
   * Magnitudes already ≤ 1.0 are left unchanged.
   */
  public DriveVector unitCircle() {
    this.normalize = true;
    return this;
  }

  /**
   * Evaluate both axes and return the (possibly normalized) vector.
   * The returned {@link Translation2d} has X = first axis, Y = second axis.
   */
  public Translation2d get() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    if (normalize) {
      double mag = Math.hypot(x, y);
      if (mag > 1.0) {
        x /= mag;
        y /= mag;
      }
    }
    return new Translation2d(x, y);
  }
}
