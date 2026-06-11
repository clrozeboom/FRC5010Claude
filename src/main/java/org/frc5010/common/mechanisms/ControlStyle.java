package org.frc5010.common.mechanisms;

/**
 * Closed-loop control style for the single-DOF YAMS mechanism wrappers
 * ({@link YamsElevator}, {@link YamsArm}, {@link YamsPivot}, {@link YamsFlywheel}).
 *
 * <p>Selected via {@code settings.controlStyle}; both styles use the same settings,
 * commands, telemetry, and motion profile limits, so switching is a one-line change.
 */
public enum ControlStyle {
  /**
   * State-space LQR + Kalman filter, running on the RIO in the YAMS closed-loop
   * Notifier. Tuned with physical tolerances (qelms/relms) under
   * {@code /Tuning/<name>/lqr_*}; gains are computed from the plant model, so the
   * mechanism's mass/MOI/gearing settings must be accurate.
   */
  LQR,
  /**
   * Profiled PID — trapezoidal motion profile + kP/kI/kD with feedforward (kS/kV/kG).
   * On TalonFX this runs onboard (MotionMagic for position, VelocityVoltage for
   * flywheels) with gains in <b>mechanism rotations</b> (kP = volts per rotation of
   * error). Tuned under {@code /Tuning/<name>/pid_*}. Simpler to reason about and
   * tolerant of model error, at the cost of hand tuning.
   */
  PROFILED_PID
}
