package org.frc5010.common.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for a single-motor mechanism (elevator, arm, pivot, flywheel —
 * or one joint/side of a multi-motor mechanism).
 *
 * <p>This is the AdvantageKit replay bubble: everything read from hardware flows
 * through {@link MechanismIOInputs}; everything sent to hardware goes through the
 * methods below. Units at the IO boundary are <b>mechanism rotations</b> (after
 * gearing) — subsystems convert to meters/radians as needed.
 *
 * <p>Implementations: {@link MechanismIOTalonFX} (REAL), {@link MechanismIOTalonFXSim}
 * (SIM — same Phoenix code, sim state fed by a WPILib physics sim), and the no-op
 * default methods themselves (REPLAY — inputs come from the log).
 */
public interface MechanismIO {

  /** Sensor readings crossing the replay bubble. Mechanism rotations, after gearing. */
  @AutoLog
  class MechanismIOInputs {
    /** True when the last status-signal refresh succeeded. */
    public boolean connected = false;
    /** Mechanism position, rotations. */
    public double positionRot = 0.0;
    /** Mechanism velocity, rotations per second. */
    public double velocityRotPerSec = 0.0;
    /** Applied motor output voltage. */
    public double appliedVolts = 0.0;
    /** Stator current, amps. */
    public double statorCurrentAmps = 0.0;
  }

  /** Updates the loggable inputs from hardware. */
  default void updateInputs(MechanismIOInputs inputs) {}

  /** Applies a raw output voltage (used by the RIO-side LQR, homing, and SysId). */
  default void setVoltage(double volts) {}

  /** Applies an open-loop duty cycle (−1..1), battery-compensated by the controller. */
  default void setDutyCycle(double dutyCycle) {}

  /**
   * Runs the onboard MotionMagic position controller to the given mechanism position
   * (PROFILED_PID style; profile and gains were configured at construction).
   *
   * @param positionRot target, mechanism rotations
   */
  default void runPosition(double positionRot) {}

  /**
   * Runs the onboard velocity controller (PROFILED_PID style flywheels).
   *
   * @param velocityRotPerSec target, mechanism rotations per second
   */
  default void runVelocity(double velocityRotPerSec) {}

  /** Neutral output. */
  default void stop() {}

  /**
   * Re-applies closed-loop gains (live tuning, PROFILED_PID style).
   *
   * @param kP volts per mechanism rotation (or rot/s for velocity) of error
   * @param kI integral gain
   * @param kD derivative gain
   */
  default void setPidGains(double kP, double kI, double kD) {}

  /**
   * Re-seeds the sensor (homing): declares the mechanism's current physical position.
   *
   * @param positionRot the true mechanism position, rotations
   */
  default void setSensorPosition(double positionRot) {}

  /**
   * Enables/disables the configured soft limits. Homing routines disable them while
   * driving into the hard stop (a mis-seeded encoder makes soft limits meaningless).
   *
   * @param enabled true to enforce the configured soft limits
   */
  default void setSoftLimitsEnabled(boolean enabled) {}

  /** Frees the hardware handles (CAN IDs) for unit tests. */
  default void close() {}
}
