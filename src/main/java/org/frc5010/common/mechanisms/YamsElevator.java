package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
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
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * LQR-controlled elevator built on the YAMS {@link Elevator} mechanism.
 *
 * <p>Common (robot-agnostic) wrapper: all robot-specific values — CAN ID, gearing,
 * carriage mass, travel limits — live in a {@link Settings} object owned by team code
 * (see {@code frc.robot.mechanisms.ExampleElevator}).
 *
 * <p><b>Controller:</b> a WPILib LQR (state-space {@code LinearSystemLoop} with Kalman
 * filter) wrapped by YAMS {@link LQRController}, running on the RIO in the YAMS
 * closed-loop Notifier at 20 ms. A trapezoidal motion profile feeds position+velocity
 * setpoints to the regulator; an {@link ElevatorFeedforward} kG term cancels gravity
 * (the linearized elevator plant does not model it, and LQR has no integrator).
 *
 * <p><b>Tuning:</b> the LQR weights (qelms position/velocity tolerance, relms control
 * effort) are live-tunable via NetworkTables under {@code /Tuning/<name>/} — see
 * {@link LqrTunables}. On change the regulator is rebuilt and the loop restarted at the
 * current state.
 *
 * <p><b>Units gotcha:</b> the closed loop must run in meters for an ELEVATOR-type LQR.
 * {@code withMechanismCircumference} plus the linear trapezoid profile / elevator
 * feedforward put the YAMS loop in linear (meters) mode — do not remove them.
 */
public class YamsElevator extends SubsystemBase {

  /** Robot-specific elevator parameters. Populate the fields, then construct {@link YamsElevator}. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Elevator";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** Motor controller vendor. */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** CAN ID of the motor controller. */
    public int canId;
    /** Motor physics model (count = motors on the gearbox). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {4, 3} = 12:1). */
    public double[] gearReductionStages = {4, 3};
    /** Drum/sprocket circumference — carriage travel per drum rotation. */
    public Distance drumCircumference = Inches.of(5.5);
    /** Mass of the moving carriage (plus load). */
    public Mass carriageMass = Kilograms.of(6.0);
    /** Lowest carriage position (hard + soft limit). */
    public Distance minHeight = Meters.of(0);
    /** Highest carriage position (hard + soft limit). */
    public Distance maxHeight = Meters.of(1.5);
    /** Carriage position at robot power-on. */
    public Distance startingHeight = Meters.of(0);
    /**
     * Motion profile cruise velocity. Must be achievable: below
     * motorFreeSpeed / gearing × drumCircumference, or the profile runs away from the
     * mechanism and the LQR saturates/overshoots chasing it.
     */
    public LinearVelocity maxVelocity = MetersPerSecond.of(0.9);
    /** Motion profile acceleration. */
    public LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    /** Gravity feedforward (volts to hold the carriage) — from SysId or sim ramp. */
    public Voltage kG = Volts.of(0);
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /**
     * Position error tolerance. Smaller = more aggressive. Note the YAMS loop adds
     * 20–40 ms of effective delay (Notifier + sim update), so weights tighter than
     * ~1 inch tend to oscillate.
     */
    public Distance qelmsPosition = Inches.of(2);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public LinearVelocity qelmsVelocity = MetersPerSecond.of(0.5);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard, units = mechanism rotations) ---
    /** Proportional gain, volts per drum rotation of error. */
    public double kP = 4;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts (PROFILED_PID only). */
    public double kS = 0;
    /** Velocity feedforward, volts per drum rotation/s (PROFILED_PID only — the LQR provides its own). */
    public double kV = 0;

    // --- Kalman filter trust (rarely changed) ---
    /** Model position standard deviation — how much you trust the plant model. */
    public Distance modelPositionTrust = Meters.of(0.05);
    /** Model velocity standard deviation. */
    public LinearVelocity modelVelocityTrust = MetersPerSecond.of(0.5);
    /** Encoder position standard deviation — how much you trust the sensor. */
    public Distance encoderPositionTrust = Meters.of(0.001);

    // --- Characterized plant (optional, LQR style — see docs/mechanisms.md) ---
    /**
     * Measured kV from a SysId run, volts per m/s. Leave 0 to use the physics-model
     * plant. When both kV and kA are set, the LQR plant is built from these measured
     * values instead of motor + gearing + {@link #carriageMass} — the real mass and
     * losses are implied by how the mechanism actually responded to voltage, so an
     * unknown or wrong carriage mass no longer matters to the controller.
     */
    public double characterizedKv = 0;
    /** Measured kA from a SysId run, volts per m/s². See {@link #characterizedKv}. */
    public double characterizedKa = 0;
  }

  /**
   * AdvantageKit inputs — everything read back from the motor/mechanism crosses the
   * replay bubble here. Fields stay {@code double} (project convention: the AutoLog
   * processor expects primitives; extract with {@code .in(unit)} before assigning).
   */
  @AutoLog
  public static class ElevatorInputs {
    /** Carriage height, meters. */
    public double positionMeters;
    /** Carriage velocity, m/s. */
    public double velocityMetersPerSec;
    /** Active closed-loop setpoint, meters. */
    public double setpointMeters;
    /** Applied motor output voltage. */
    public double appliedVolts;
    /** Stator current, amps. */
    public double statorCurrentAmps;
  }

  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private final Settings settings;
  private final MechanismGearing gearing;
  private final LQRController lqr; // null in PROFILED_PID style
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final Elevator elevator;
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style

  /**
   * Builds the elevator subsystem, its motor wrapper, controller, and simulation.
   *
   * @param settings robot-specific elevator parameters
   */
  public YamsElevator(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = new MechanismGearing(GearBox.fromReductionStages(settings.gearReductionStages));
    boolean useLqr = settings.controlStyle == ControlStyle.LQR;
    lqr = useLqr
        ? new LQRController(buildLqrConfig(
            settings.qelmsPosition.in(Meters),
            settings.qelmsVelocity.in(MetersPerSecond),
            settings.relms.in(Volts)))
        : null;
    motorConfig = new SmartMotorControllerConfig(this)
        .withMechanismCircumference(settings.drumCircumference)
        .withGearing(gearing)
        .withSoftLimit(settings.minHeight, settings.maxHeight)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(settings.statorCurrentLimit)
        .withTelemetry(settings.name + "Motor", TelemetryVerbosity.HIGH)
        // kG cancels gravity (TalonFX: onboard Elevator_Static); also flips the YAMS
        // loop into linear (meters) mode. kS/kV apply only in PROFILED_PID — the LQR
        // loop provides its own plant-inversion feedforward.
        .withFeedforward(useLqr
            ? new ElevatorFeedforward(0, settings.kG.in(Volts), 0)
            : new ElevatorFeedforward(settings.kS, settings.kG.in(Volts), settings.kV))
        .withTrapezoidalProfile(settings.maxVelocity, settings.maxAcceleration)
        .withControlMode(ControlMode.CLOSED_LOOP);
    if (useLqr) {
      // Must stay LAST: the PID-style withClosedLoopController overloads clear the LQR.
      motorConfig.withClosedLoopController(lqr);
    } else {
      motorConfig.withClosedLoopController(settings.kP, settings.kI, settings.kD);
    }
    motor = MechanismMotor.create(settings.vendor, settings.canId, settings.motorModel, motorConfig);
    elevator = new Elevator(new ElevatorConfig(motor)
        .withStartingHeight(settings.startingHeight)
        .withHardLimits(settings.minHeight, settings.maxHeight)
        .withMass(settings.carriageMass)
        .withTelemetry(settings.name, TelemetryVerbosity.HIGH));
    lqrTunables = useLqr
        ? new LqrTunables(settings.name,
            settings.qelmsPosition.in(Meters),
            settings.qelmsVelocity.in(MetersPerSecond),
            settings.relms.in(Volts))
        : null;
    pidGains = useLqr ? null
        : new TunableGains(settings.name, "pid", settings.kP, settings.kI, settings.kD, 0);
  }

  private LQRConfig buildLqrConfig(double qelmsPosMeters, double qelmsVelMps, double relmsVolts) {
    // MOI is required by the LQRConfig constructor but unused for the ELEVATOR plant.
    return MechanismLqrConfig.elevator(
            settings.motorModel,
            gearing,
            KilogramSquareMeters.of(0.001),
            Meters.of(qelmsPosMeters),
            MetersPerSecond.of(qelmsVelMps),
            settings.modelPositionTrust,
            settings.modelVelocityTrust,
            settings.encoderPositionTrust,
            settings.carriageMass,
            settings.drumCircumference.div(2 * Math.PI),
            Volts.of(relmsVolts))
        // Elevator loop runs in meters, same units SysId reports — no conversion.
        .withCharacterizedGains(settings.characterizedKv, settings.characterizedKa);
  }

  private void updateInputs() {
    inputs.positionMeters = elevator.getHeight().in(Meters);
    inputs.velocityMetersPerSec = elevator.getVelocity().in(MetersPerSecond);
    inputs.setpointMeters = motor.getMechanismPositionSetpoint()
        .map(sp -> motorConfig.convertFromMechanism(sp).in(Meters))
        .orElse(inputs.positionMeters);
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
      // Restart the loop so profile + Kalman state re-seed at the current position.
      motor.startClosedLoopController();
    }
    if (pidGains != null && pidGains.hasChanged()) {
      motorConfig.withClosedLoopController(pidGains.kP(), pidGains.kI(), pidGains.kD());
      motor.applyConfig(motorConfig);
    }
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    elevator.simIterate();
  }

  /** Command: drive the carriage to the given height. Never finishes. */
  public Command goToHeight(Distance height) {
    Logger.recordOutput(settings.name + "/CommandedHeightMeters", height.in(Meters));
    return elevator.setHeight(height);
  }

  /** Command: open-loop duty cycle (e.g. for manual jog). */
  public Command setDutyCycle(double dutyCycle) {
    return elevator.set(dutyCycle);
  }

  /** Command: SysId routine for characterizing kG/kS/kV on a real robot. */
  public Command sysId() {
    return elevator.sysId(Volts.of(7), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current carriage height (from the AdvantageKit inputs — replay-safe). */
  public Distance getHeight() {
    return Meters.of(inputs.positionMeters);
  }

  /** Current carriage velocity (from the AdvantageKit inputs — replay-safe). */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(inputs.velocityMetersPerSec);
  }

  /** Trigger: true while the carriage is within {@code tolerance} of {@code height}. */
  public Trigger isAtHeight(Distance height, Distance tolerance) {
    return new Trigger(
        () -> Math.abs(inputs.positionMeters - height.in(Meters)) <= tolerance.in(Meters));
  }

  /** Underlying YAMS mechanism, for advanced use (triggers, sim access). */
  public Elevator getMechanism() {
    return elevator;
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
