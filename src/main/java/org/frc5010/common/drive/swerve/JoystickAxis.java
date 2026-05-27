package org.frc5010.common.drive.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/**
 * A single joystick axis with a chainable transform pipeline.
 *
 * <p>Transforms are applied in the order they are stacked. Example:
 * <pre>{@code
 * JoystickAxis forward = JoystickAxis.of(() -> joystick.getRawAxis(1))
 *     .deadzone(0.05)
 *     .power(2.0)
 *     .scale(drive.getMaxLinearSpeed().in(MetersPerSecond))
 *     .negate();
 * }</pre>
 */
public class JoystickAxis implements DoubleSupplier {

  private final DoubleSupplier source;
  private DoubleUnaryOperator chain;

  private JoystickAxis(DoubleSupplier source) {
    this.source = source;
    this.chain = v -> v;
  }

  /** Creates an axis backed by the given supplier, with no transforms applied yet. */
  public static JoystickAxis of(DoubleSupplier source) {
    return new JoystickAxis(source);
  }

  /**
   * Apply a deadzone: values whose absolute value is {@code ≤ threshold} snap to 0.
   * The remaining range is rescaled so that full deflection still maps to ±1.
   */
  public JoystickAxis deadzone(double threshold) {
    return append(v -> {
      if (Math.abs(v) <= threshold) return 0.0;
      return Math.copySign((Math.abs(v) - threshold) / (1.0 - threshold), v);
    });
  }

  /**
   * Raise the absolute value to {@code exponent}, preserving the original sign.
   * Useful for response curves — e.g. {@code power(2)} gives a squared (gentle-start) curve.
   */
  public JoystickAxis power(double exponent) {
    return append(v -> Math.copySign(Math.pow(Math.abs(v), exponent), v));
  }

  /** Multiply the output by {@code factor}. Use to map a normalised [-1, 1] axis to physical units. */
  public JoystickAxis scale(double factor) {
    return append(v -> v * factor);
  }

  /** Clamp the output to {@code [-max, max]}. */
  public JoystickAxis limit(double max) {
    return append(v -> Math.min(max, Math.max(-max, v)));
  }

  /** Flip the sign of the output. */
  public JoystickAxis negate() {
    return append(v -> -v);
  }

  private JoystickAxis append(DoubleUnaryOperator next) {
    chain = chain.andThen(next);
    return this;
  }

  @Override
  public double getAsDouble() {
    return chain.applyAsDouble(source.getAsDouble());
  }
}
