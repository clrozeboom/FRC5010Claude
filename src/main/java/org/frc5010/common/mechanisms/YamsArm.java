package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.LQRConfig;
import yams.math.LQRController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * LQR-controlled single-jointed arm built on the YAMS {@link Arm} mechanism.
 *
 * <p>Common (robot-agnostic) wrapper — robot-specific values live in {@link Settings}
 * (see {@code frc.robot.mechanisms.ExampleArm}).
 *
 * <p><b>Controller:</b> ARM-type LQR (single-jointed-arm plant from motor model, gearing
 * and MOI = ⅓·m·L², matching the YAMS arm simulation) with a trapezoidal motion profile.
 * The linearized plant has no gravity term, so an {@link ArmFeedforward} kG·cos(θ)
 * compensates it; LQR has no integrator to absorb the residual.
 * Note: YAMS only applies the arm feedforward when a motion profile is configured —
 * keep the trapezoid profile.
 *
 * <p><b>Tuning:</b> qelms/relms live-tunable under {@code /Tuning/<name>/}
 * ({@link LqrTunables}); angular weights are in rotations and rotations/s.
 */
public class YamsArm extends SubsystemBase {

  /** Robot-specific arm parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Arm";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** Motor controller vendor. */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** CAN ID of the motor controller. */
    public int canId;
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {10, 5} = 50:1). */
    public double[] gearReductionStages = {10, 5};
    /** Arm length, pivot to tip. */
    public Distance length = Meters.of(0.6);
    /** Arm mass (assumed uniform rod for MOI = ⅓·m·L²). */
    public Mass mass = Kilograms.of(4.0);
    /** Lower hard limit (0° = horizontal). */
    public Angle minAngle = Degrees.of(-30);
    /** Upper hard limit. */
    public Angle maxAngle = Degrees.of(210);
    /** Arm angle at robot power-on. */
    public Angle startingAngle = Degrees.of(0);
    /** Motion profile cruise velocity. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(180);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360);
    /** Gravity feedforward (volts to hold the arm horizontal) — from SysId or sim ramp. */
    public Voltage kG = Volts.of(0);
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /** Position error tolerance. Smaller = more aggressive. */
    public Angle qelmsPosition = Degrees.of(1.5);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public AngularVelocity qelmsVelocity = DegreesPerSecond.of(20);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard, units = mechanism rotations) ---
    /** Proportional gain, volts per arm rotation of error. */
    public double kP = 30;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts (PROFILED_PID only). */
    public double kS = 0;
    /** Velocity feedforward, volts per arm rotation/s (PROFILED_PID only — the LQR provides its own). */
    public double kV = 0;

    // --- Kalman filter trust (rarely changed) ---
    /** Model position standard deviation. */
    public Angle modelPositionTrust = Radians.of(0.015);
    /** Model velocity standard deviation. */
    public AngularVelocity modelVelocityTrust = RadiansPerSecond.of(0.17);
    /** Encoder position standard deviation. */
    public Angle encoderPositionTrust = Radians.of(0.001);

    // --- Characterized plant (optional, LQR style — see docs/mechanisms.md) ---
    /**
     * Measured kV from a SysId run, volts per rotation/s (SysId tool set to Rotations).
     * Leave 0 to use the physics-model plant. When both kV and kA are set, the LQR
     * plant is built from these measured values instead of motor + gearing +
     * {@link #mass}/{@link #length} — the real inertia and losses are implied by how
     * the arm actually responded to voltage, so an unknown mass no longer matters to
     * the controller. (kG still comes from SysId separately — gravity is not part of
     * the linear plant.)
     */
    public double characterizedKv = 0;
    /** Measured kA from a SysId run, volts per rotation/s². See {@link #characterizedKv}. */
    public double characterizedKa = 0;
  }

  /**
   * AdvantageKit inputs — everything read back from the motor/mechanism crosses the
   * replay bubble here. Fields stay {@code double} (project convention).
   */
  @AutoLog
  public static class ArmInputs {
    /** Arm angle, degrees (0° = horizontal). */
    public double positionDegrees;
    /** Arm angular velocity, deg/s. */
    public double velocityDegreesPerSec;
    /** Active closed-loop setpoint, degrees. */
    public double setpointDegrees;
    /** Applied motor output voltage. */
    public double appliedVolts;
    /** Stator current, amps. */
    public double statorCurrentAmps;
  }

  private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

  private final Settings settings;
  private final MechanismGearing gearing;
  private final LQRController lqr; // null in PROFILED_PID style
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final Arm arm;
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style

  /**
   * Builds the arm subsystem, motor wrapper, controller, and simulation.
   *
   * @param settings robot-specific arm parameters
   */
  public YamsArm(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = new MechanismGearing(GearBox.fromReductionStages(settings.gearReductionStages));
    boolean useLqr = settings.controlStyle == ControlStyle.LQR;
    lqr = useLqr
        ? new LQRController(buildLqrConfig(
            settings.qelmsPosition.in(Rotations),
            settings.qelmsVelocity.in(RotationsPerSecond),
            settings.relms.in(Volts)))
        : null;
    motorConfig = new SmartMotorControllerConfig(this)
        .withGearing(gearing)
        .withSoftLimit(settings.minAngle, settings.maxAngle)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(settings.statorCurrentLimit)
        .withTelemetry(settings.name + "Motor", TelemetryVerbosity.HIGH)
        // kG·cos(θ) gravity compensation (TalonFX: onboard Arm_Cosine). kS/kV apply
        // only in PROFILED_PID — the LQR loop provides its own feedforward.
        .withFeedforward(useLqr
            ? new ArmFeedforward(0, settings.kG.in(Volts), 0)
            : new ArmFeedforward(settings.kS, settings.kG.in(Volts), settings.kV))
        .withTrapezoidalProfile(settings.maxVelocity, settings.maxAcceleration)
        .withControlMode(ControlMode.CLOSED_LOOP);
    if (useLqr) {
      // Must stay LAST: the PID-style withClosedLoopController overloads clear the LQR.
      motorConfig.withClosedLoopController(lqr);
    } else {
      motorConfig.withClosedLoopController(settings.kP, settings.kI, settings.kD);
    }
    motor = MechanismMotor.create(settings.vendor, settings.canId, settings.motorModel, motorConfig);
    arm = new Arm(new ArmConfig(motor)
        .withLength(settings.length)
        .withMass(settings.mass)
        .withHardLimit(settings.minAngle, settings.maxAngle)
        .withStartingPosition(settings.startingAngle)
        .withTelemetry(settings.name, TelemetryVerbosity.HIGH));
    lqrTunables = useLqr
        ? new LqrTunables(settings.name,
            settings.qelmsPosition.in(Rotations),
            settings.qelmsVelocity.in(RotationsPerSecond),
            settings.relms.in(Volts))
        : null;
    pidGains = useLqr ? null
        : new TunableGains(settings.name, "pid", settings.kP, settings.kI, settings.kD, 0);
  }

  private LQRConfig buildLqrConfig(double qelmsPosRot, double qelmsVelRps, double relmsVolts) {
    // Uniform-rod MOI, identical to SingleJointedArmSim.estimateMOI used by the YAMS sim.
    double moi = settings.mass.in(Kilograms) * Math.pow(settings.length.in(Meters), 2) / 3.0;
    return MechanismLqrConfig.arm(
            settings.motorModel,
            gearing,
            KilogramSquareMeters.of(moi),
            Rotations.of(qelmsPosRot),
            RotationsPerSecond.of(qelmsVelRps),
            settings.modelPositionTrust,
            settings.modelVelocityTrust,
            settings.encoderPositionTrust,
            Volts.of(relmsVolts))
        // Settings take SysId's rotation units; the arm plant works in radians.
        .withCharacterizedGains(
            settings.characterizedKv / (2 * Math.PI),
            settings.characterizedKa / (2 * Math.PI));
  }

  private void updateInputs() {
    inputs.positionDegrees = arm.getAngle().in(Degrees);
    inputs.velocityDegreesPerSec = motor.getMechanismVelocity().in(DegreesPerSecond);
    inputs.setpointDegrees = motor.getMechanismPositionSetpoint()
        .map(sp -> sp.in(Degrees))
        .orElse(inputs.positionDegrees);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(settings.name, inputs);
    if (lqr != null && lqrTunables.hasChanged()) {
      lqr.updateConfig(buildLqrConfig(
          lqrTunables.qelmsPosition(), lqrTunables.qelmsVelocity(), lqrTunables.relms()));
      motor.startClosedLoopController();
    }
    if (pidGains != null && pidGains.hasChanged()) {
      motorConfig.withClosedLoopController(pidGains.kP(), pidGains.kI(), pidGains.kD());
      motor.applyConfig(motorConfig);
    }
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

  /** Command: drive the arm to the given angle. Never finishes. */
  public Command goToAngle(Angle angle) {
    Logger.recordOutput(settings.name + "/CommandedAngleDegrees", angle.in(Degrees));
    return arm.setAngle(angle);
  }

  /** Command: open-loop duty cycle (e.g. for manual jog). */
  public Command setDutyCycle(double dutyCycle) {
    return arm.set(dutyCycle);
  }

  /** Command: SysId routine for characterizing kG/kS/kV on a real robot. */
  public Command sysId() {
    return arm.sysId(Volts.of(3), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current arm angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getAngle() {
    return Degrees.of(inputs.positionDegrees);
  }

  /** Trigger: true while the arm is within {@code tolerance} of {@code angle}. */
  public Trigger isAtAngle(Angle angle, Angle tolerance) {
    return new Trigger(
        () -> Math.abs(inputs.positionDegrees - angle.in(Degrees)) <= tolerance.in(Degrees));
  }

  /** Underlying YAMS mechanism, for advanced use. */
  public Arm getMechanism() {
    return arm;
  }

  /** Underlying YAMS motor wrapper, for advanced use. */
  public SmartMotorController getMotor() {
    return motor;
  }

  /** The settings this mechanism was built with (start positions, limits, ...). */
  public Settings getSettings() {
    return settings;
  }

  /** Stops the closed-loop Notifier and frees the CAN device. For unit tests. */
  public void close() {
    motor.close();
    MechanismMotor.closeDevice(motor);
  }
}
