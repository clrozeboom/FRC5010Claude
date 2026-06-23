package org.frc5010.common.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * TalonFX implementation of {@link MechanismIO}.
 *
 * <p>Configures the Talon so its feedback is in <b>mechanism rotations</b>
 * ({@code SensorToMechanismRatio} = gearing, or a fused CANcoder when
 * {@link Config#cancoderId} is set), letting the onboard MotionMagic /
 * VelocityVoltage controllers and the soft limits all work in mechanism units.
 * Feedforward gains (kS/kV/kG with the configured {@link GravityTypeValue}) run
 * onboard at 1 kHz in PROFILED_PID style; the LQR style drives {@link #setVoltage}
 * from the RIO instead.
 *
 * <p>Optionally drives a second TalonFX as a {@link Follower}
 * ({@link Config#followerCanId}) — common for two-motor elevator gearboxes.
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
    /** Lower soft limit, mechanism rotations. NaN disables. */
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
    /**
     * CAN ID of a fused CANcoder mounted 1:1 on the mechanism; −1 = use the rotor
     * sensor. When set, position is absolute — the starting position is read from the
     * CANcoder instead of {@link #startingPositionRot}.
     */
    public int cancoderId = -1;
    /** CANcoder magnet offset, rotations (reading at the mechanism's zero). */
    public double cancoderOffsetRot = 0;
    /**
     * Fused-CANcoder absolute discontinuity point, rotations: the absolute position
     * reported by a 1:1 CANcoder wraps over the range {@code [point − 1, point)}. The
     * Phoenix default (0.5 → ±180°) is wrong for any mechanism whose travel spans more
     * than ±180° from the CANcoder zero, because at power-on the Talon seeds its fused
     * position from this wrapping absolute reading and would be ~360° off. Set this so
     * the wrap falls <em>outside</em> the mechanism's travel (e.g. opposite the travel
     * midpoint). Only used when {@link #cancoderId} is set.
     */
    public double absoluteSensorDiscontinuityRot = 0.5;

    /**
     * Run all control requests with FOC commutation (~15% more torque, smoother
     * low-speed control). Requires Phoenix Pro licensing on the device; unlicensed
     * devices fall back to non-FOC and raise an UnlicensedFeatureInUse fault.
     * Default true — set false for non-Pro teams.
     */
    public boolean enableFoc = true;
    /** CAN ID of a follower TalonFX (two-motor gearbox); −1 = none. */
    public int followerCanId = -1;
    /** True if the follower is mounted opposing the lead motor. */
    public boolean followerOpposed = false;
  }

  protected final TalonFX talon;
  protected final CANcoder cancoder; // null when not configured
  protected final TalonFX follower; // null when not configured
  protected final Config config;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;

  private final VoltageOut voltageRequest;
  private final DutyCycleOut dutyCycleRequest;
  private final MotionMagicVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final NeutralOut neutralRequest = new NeutralOut();

  private final TalonFXConfiguration talonConfig;

  /**
   * Creates and configures the TalonFX (and CANcoder / follower when configured).
   *
   * @param config hardware configuration
   */
  public MechanismIOTalonFX(Config config) {
    this.config = config;
    talon = new TalonFX(config.canId);
    voltageRequest = new VoltageOut(0).withEnableFOC(config.enableFoc);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(config.enableFoc);
    positionRequest = new MotionMagicVoltage(0).withEnableFOC(config.enableFoc);
    velocityRequest = new VelocityVoltage(0).withEnableFOC(config.enableFoc);

    talonConfig = new TalonFXConfiguration();
    if (config.cancoderId >= 0) {
      cancoder = new CANcoder(config.cancoderId);
      var cancoderConfig = new CANcoderConfiguration();
      cancoderConfig.MagnetSensor.MagnetOffset = -config.cancoderOffsetRot;
      // Place the absolute wrap outside the travel so power-on seeding is unambiguous
      // even for arms/pivots that swing past ±180° from the CANcoder zero.
      cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
          config.absoluteSensorDiscontinuityRot;
      cancoder.getConfigurator().apply(cancoderConfig);
      talonConfig.Feedback.FeedbackRemoteSensorID = config.cancoderId;
      talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      talonConfig.Feedback.RotorToSensorRatio = config.gearing;
      talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    } else {
      cancoder = null;
      talonConfig.Feedback.SensorToMechanismRatio = config.gearing;
    }
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
    if (cancoder == null) {
      seedStartingPosition();
    }

    if (config.followerCanId >= 0) {
      follower = new TalonFX(config.followerCanId);
      var followerConfig = new TalonFXConfiguration();
      followerConfig.MotorOutput.NeutralMode = talonConfig.MotorOutput.NeutralMode;
      followerConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimitAmps;
      followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      follower.getConfigurator().apply(followerConfig);
      follower.setControl(new Follower(config.canId,
          config.followerOpposed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    } else {
      follower = null;
    }

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
  public void setDutyCycle(double dutyCycle) {
    talon.setControl(dutyCycleRequest.withOutput(dutyCycle));
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
  public void setSensorPosition(double positionRot) {
    talon.setPosition(positionRot);
  }

  /**
   * Seeds the mechanism's starting position at construction (rotor-sensor mode only — a fused
   * CANcoder reads absolute and needs no seed). The REAL path seeds the Talon's mechanism
   * position directly; the SIM subclass overrides this to seed the simulated <em>raw rotor</em>
   * instead, so the physics-driven rotor stays the single source of truth.
   *
   * <p>Seeding both — the Talon mechanism offset (here) <em>and</em> the simulated raw rotor —
   * double-counts the start angle: a 120° start would read 240° (the symptom this split fixes).
   * Only {@code config}, {@code talon}, and {@code cancoder} are touched, all set before this is
   * invoked from the constructor.
   */
  protected void seedStartingPosition() {
    talon.setPosition(config.startingPositionRot);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        enabled && !Double.isNaN(config.softLimitLowRot);
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        enabled && !Double.isNaN(config.softLimitHighRot);
    talon.getConfigurator().apply(talonConfig.SoftwareLimitSwitch);
  }

  @Override
  public void close() {
    talon.close();
    if (cancoder != null) {
      cancoder.close();
    }
    if (follower != null) {
      follower.close();
    }
  }
}
