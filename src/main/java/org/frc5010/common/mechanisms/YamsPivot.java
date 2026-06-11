package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
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
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * LQR-controlled pivot (turret, hood, wrist) built on the YAMS {@link Pivot} mechanism.
 *
 * <p>A pivot is a gravity-free rotating mechanism — same plant as an arm but without the
 * gravity feedforward. The ARM-type LQR (motor model + gearing + MOI) is the correct
 * state-space model for it; YAMS has no separate PIVOT LQR type.
 *
 * <p>Robot-specific values live in {@link Settings}
 * (see {@code frc.robot.mechanisms.ExampleTurret}). LQR weights are live-tunable under
 * {@code /Tuning/<name>/} ({@link LqrTunables}; rotations / rotations-per-second).
 */
public class YamsPivot extends SubsystemBase {

  /** Robot-specific pivot parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Pivot";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** Motor controller vendor. */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** CAN ID of the motor controller. */
    public int canId;
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {10, 4} = 40:1). */
    public double[] gearReductionStages = {10, 4};
    /** Moment of inertia of the rotating assembly about the pivot axis. */
    public MomentOfInertia moi = KilogramSquareMeters.of(0.5);
    /** Lower hard limit. */
    public Angle minAngle = Degrees.of(-180);
    /** Upper hard limit. */
    public Angle maxAngle = Degrees.of(180);
    /** Pivot angle at robot power-on. */
    public Angle startingAngle = Degrees.of(0);
    /** Motion profile cruise velocity. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(360);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(720);
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /** Position error tolerance. Smaller = more aggressive. */
    public Angle qelmsPosition = Degrees.of(1.0);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public AngularVelocity qelmsVelocity = DegreesPerSecond.of(20);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard, units = mechanism rotations) ---
    /** Proportional gain, volts per pivot rotation of error. */
    public double kP = 30;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts (PROFILED_PID only). */
    public double kS = 0;
    /** Velocity feedforward, volts per pivot rotation/s (PROFILED_PID only — the LQR provides its own). */
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
     * {@link #moi} — the real inertia and losses are implied by how the pivot actually
     * responded to voltage, so an unknown MOI no longer matters to the controller.
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
  public static class PivotInputs {
    /** Pivot angle, degrees. */
    public double positionDegrees;
    /** Pivot angular velocity, deg/s. */
    public double velocityDegreesPerSec;
    /** Active closed-loop setpoint, degrees. */
    public double setpointDegrees;
    /** Applied motor output voltage. */
    public double appliedVolts;
    /** Stator current, amps. */
    public double statorCurrentAmps;
  }

  private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();

  private final Settings settings;
  private final MechanismGearing gearing;
  private final LQRController lqr; // null in PROFILED_PID style
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final Pivot pivot;
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style

  /**
   * Builds the pivot subsystem, motor wrapper, controller, and simulation.
   *
   * @param settings robot-specific pivot parameters
   */
  public YamsPivot(Settings settings) {
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
        .withTrapezoidalProfile(settings.maxVelocity, settings.maxAcceleration)
        .withControlMode(ControlMode.CLOSED_LOOP);
    if (!useLqr && (settings.kS != 0 || settings.kV != 0)) {
      // Gravity-free mechanism: plain kS/kV feedforward (PROFILED_PID only).
      motorConfig.withFeedforward(new SimpleMotorFeedforward(settings.kS, settings.kV));
    }
    if (useLqr) {
      // Must stay LAST: the PID-style withClosedLoopController overloads clear the LQR.
      motorConfig.withClosedLoopController(lqr);
    } else {
      motorConfig.withClosedLoopController(settings.kP, settings.kI, settings.kD);
    }
    motor = MechanismMotor.create(settings.vendor, settings.canId, settings.motorModel, motorConfig);
    pivot = new Pivot(new PivotConfig(motor)
        .withMOI(settings.moi)
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
    return MechanismLqrConfig.arm(
            settings.motorModel,
            gearing,
            settings.moi,
            Rotations.of(qelmsPosRot),
            RotationsPerSecond.of(qelmsVelRps),
            settings.modelPositionTrust,
            settings.modelVelocityTrust,
            settings.encoderPositionTrust,
            Volts.of(relmsVolts))
        // Settings take SysId's rotation units; the pivot (arm-type) plant works in radians.
        .withCharacterizedGains(
            settings.characterizedKv / (2 * Math.PI),
            settings.characterizedKa / (2 * Math.PI));
  }

  private void updateInputs() {
    inputs.positionDegrees = pivot.getAngle().in(Degrees);
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
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    pivot.simIterate();
  }

  /** Command: rotate the pivot to the given angle. Never finishes. */
  public Command goToAngle(Angle angle) {
    Logger.recordOutput(settings.name + "/CommandedAngleDegrees", angle.in(Degrees));
    return pivot.setAngle(angle);
  }

  /** Command: open-loop duty cycle (e.g. for manual jog). */
  public Command setDutyCycle(double dutyCycle) {
    return pivot.set(dutyCycle);
  }

  /** Command: SysId routine for characterizing kS/kV on a real robot. */
  public Command sysId() {
    return pivot.sysId(Volts.of(3), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current pivot angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getAngle() {
    return Degrees.of(inputs.positionDegrees);
  }

  /** Trigger: true while the pivot is within {@code tolerance} of {@code angle}. */
  public Trigger isAtAngle(Angle angle, Angle tolerance) {
    return new Trigger(
        () -> Math.abs(inputs.positionDegrees - angle.in(Degrees)) <= tolerance.in(Degrees));
  }

  /** Underlying YAMS mechanism, for advanced use. */
  public Pivot getMechanism() {
    return pivot;
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
