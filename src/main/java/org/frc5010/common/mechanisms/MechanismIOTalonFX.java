package org.frc5010.common.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * TalonFX implementation of {@link MechanismIO}.
 *
 * <p>Configures the Talon so its feedback is in <b>mechanism rotations</b>
 * ({@code SensorToMechanismRatio} = gearing), letting the onboard MotionMagic /
 * VelocityVoltage controllers and the soft limits all work in mechanism units.
 * Feedforward gains (kS/kV/kG with the configured {@link GravityTypeValue}) run
 * onboard at 1 kHz in PROFILED_PID style; the LQR style drives {@link #setVoltage}
 * from the RIO instead.
 */
public class MechanismIOTalonFX implements MechanismIO {

  /** Hardware configuration distilled from a mechanism's Settings. */
  public static class Config {
    /** CAN ID of the TalonFX. */
    public int canId;
    /** Rotor-to-mechanism gear reduction (e.g. 12.0 for 12:1). */
    public double gearing = 1.0;
    /** Invert motor output. */
    public boolean inverted = false;
    /** Brake (true) or coast (false) when neutral. */
    public boolean brakeMode = true;
    /** Stator current limit, amps. */
    public double statorCurrentLimitAmps = 40;
    /** Enable + set soft limits, mechanism rotations. NaN disables. */
    public double softLimitLowRot = Double.NaN;
    /** Upper soft limit, mechanism rotations. NaN disables. */
    public double softLimitHighRot = Double.NaN;
    /** Mechanism position at power-on, rotations (seeds the relative encoder). */
    public double startingPositionRot = 0;
    /** MotionMagic cruise velocity, mechanism rot/s (PROFILED_PID style). */
    public double motionMagicCruiseRotPerSec = 1;
    /** MotionMagic acceleration, mechanism rot/s² (PROFILED_PID style). */
    public double motionMagicAccelRotPerSecSq = 2;
    /** Onboard proportional gain, volts per mechanism rotation (or rot/s) of error. */
    public double kP = 0;
    /** Onboard integral gain. */
    public double kI = 0;
    /** Onboard derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts. */
    public double kS = 0;
    /** Velocity feedforward, volts per mechanism rot/s. */
    public double kV = 0;
    /** Gravity feedforward, volts (interpreted per {@link #gravityType}). */
    public double kG = 0;
    /** How the onboard controller applies kG. */
    public GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;
  }

  protected final TalonFX talon;
  protected final Config config;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final TalonFXConfiguration talonConfig;

  /**
   * Creates and configures the TalonFX.
   *
   * @param config hardware configuration
   */
  public MechanismIOTalonFX(Config config) {
    this.config = config;
    talon = new TalonFX(config.canId);

    talonConfig = new TalonFXConfiguration();
    talonConfig.Feedback.SensorToMechanismRatio = config.gearing;
    talonConfig.MotorOutput.NeutralMode =
        config.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talonConfig.MotorOutput.Inverted =
        config.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimitAmps;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    if (!Double.isNaN(config.softLimitLowRot)) {
      talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.softLimitLowRot;
    }
    if (!Double.isNaN(config.softLimitHighRot)) {
      talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.softLimitHighRot;
    }
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicCruiseRotPerSec;
    talonConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAccelRotPerSecSq;
    talonConfig.Slot0.kP = config.kP;
    talonConfig.Slot0.kI = config.kI;
    talonConfig.Slot0.kD = config.kD;
    talonConfig.Slot0.kS = config.kS;
    talonConfig.Slot0.kV = config.kV;
    talonConfig.Slot0.kG = config.kG;
    talonConfig.Slot0.GravityType = config.gravityType;
    talon.getConfigurator().apply(talonConfig);
    talon.setPosition(config.startingPositionRot);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    statorCurrent = talon.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, statorCurrent);
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorCurrent);
    inputs.connected = status.isOK();
    inputs.positionRot = position.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runPosition(double positionRot) {
    talon.setControl(positionRequest.withPosition(positionRot));
  }

  @Override
  public void runVelocity(double velocityRotPerSec) {
    talon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    talon.setControl(neutralRequest);
  }

  @Override
  public void setPidGains(double kP, double kI, double kD) {
    talonConfig.Slot0.kP = kP;
    talonConfig.Slot0.kI = kI;
    talonConfig.Slot0.kD = kD;
    talon.getConfigurator().apply(talonConfig.Slot0);
  }

  @Override
  public void close() {
    talon.close();
  }
}
