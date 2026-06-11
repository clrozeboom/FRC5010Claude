package org.frc5010.common.mechanisms;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Simulation implementation of {@link MechanismIO}: identical Phoenix code to
 * {@link MechanismIOTalonFX}, with the TalonFX sim state fed from a WPILib physics
 * sim each cycle (the CTRE-recommended pattern, same as the swerve's
 * {@code ModuleIOTalonFXSim}).
 *
 * <p>Per {@code updateInputs} call: read the motor voltage the (simulated) Talon is
 * applying — including its onboard MotionMagic output — push it into the physics
 * model, step the model 20 ms, then write the resulting rotor position/velocity back
 * into the sim state so the next status-signal refresh sees it.
 */
public class MechanismIOTalonFXSim extends MechanismIOTalonFX {

  private final MechanismSim sim;

  /**
   * @param config hardware configuration (shared with the REAL path)
   * @param sim    physics model adapter, in mechanism rotations
   */
  public MechanismIOTalonFXSim(Config config, MechanismSim sim) {
    super(config);
    this.sim = sim;
    // Seed the sim state so the first refresh matches the configured start position.
    talon.getSimState().setRawRotorPosition(config.startingPositionRot * config.gearing);
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    var simState = talon.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(0.02);
    simState.setRawRotorPosition(sim.getPositionRot() * config.gearing);
    simState.setRotorVelocity(sim.getVelocityRotPerSec() * config.gearing);

    super.updateInputs(inputs);
  }
}
