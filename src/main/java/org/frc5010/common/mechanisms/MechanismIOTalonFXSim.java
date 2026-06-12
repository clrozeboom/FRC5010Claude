package org.frc5010.common.mechanisms;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Simulation implementation of {@link MechanismIO}: identical Phoenix code to
 * {@link MechanismIOTalonFX}, with the TalonFX (and CANcoder, when configured) sim
 * state fed from a WPILib physics sim each cycle — the CTRE-recommended pattern, same
 * as the swerve's {@code ModuleIOTalonFXSim}.
 *
 * <p>Per {@code updateInputs} call: read the motor voltage the (simulated) Talon is
 * applying — including its onboard MotionMagic output — push it into the physics
 * model, step the model 20 ms, then write the resulting rotor/sensor positions back
 * into the sim state so the next status-signal refresh sees them. Stator current is
 * substituted from the physics model (Phoenix doesn't model it in sim), which makes
 * current-spike homing testable.
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
    if (cancoder != null) {
      cancoder.getSimState().setRawPosition(config.startingPositionRot + config.cancoderOffsetRot);
    }
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    var simState = talon.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(0.02);
    simState.setRawRotorPosition(sim.getPositionRot() * config.gearing);
    simState.setRotorVelocity(sim.getVelocityRotPerSec() * config.gearing);
    if (cancoder != null) {
      // The CANcoder reads the mechanism directly (1:1); raw = mechanism + offset
      // so the fused, offset-corrected value matches the physics model.
      var cancoderSim = cancoder.getSimState();
      cancoderSim.setRawPosition(sim.getPositionRot() + config.cancoderOffsetRot);
      cancoderSim.setVelocity(sim.getVelocityRotPerSec());
    }

    super.updateInputs(inputs);
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }
}
