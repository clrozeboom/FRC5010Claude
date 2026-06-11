package org.frc5010.common.mechanisms;

/**
 * Adapter between a WPILib physics sim ({@code ElevatorSim}, {@code SingleJointedArmSim},
 * {@code FlywheelSim}, {@code DCMotorSim}) and {@link MechanismIOTalonFXSim}, normalizing
 * the differing unit conventions to mechanism rotations.
 */
public interface MechanismSim {

  /** Applies the motor voltage (from the TalonFX sim state) to the physics model. */
  void setInputVoltage(double volts);

  /** Advances the physics model. */
  void update(double dtSeconds);

  /** Mechanism position in rotations (e.g. elevator meters ÷ drum circumference). */
  double getPositionRot();

  /** Mechanism velocity in rotations per second. */
  double getVelocityRotPerSec();
}
