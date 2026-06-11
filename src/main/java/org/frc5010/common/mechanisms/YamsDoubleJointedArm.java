package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.DoubleJointedArm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Two-jointed arm (shoulder + elbow) built on the YAMS {@link DoubleJointedArm} mechanism.
 *
 * <p><b>Controller:</b> profiled PID per joint, <em>not</em> LQR — the YAMS LQR supports
 * only the single-DOF FLYWHEEL / ARM / ELEVATOR plants, and a double-jointed arm is a
 * coupled nonlinear system that a per-joint linear regulator does not model. Gains are
 * live-tunable per joint under {@code /Tuning/<name>/} via {@link TunableGains}; on
 * change the motor config is rebuilt and re-applied (gains land onboard for TalonFX).
 *
 * <p>Robot-specific values live in {@link Settings}
 * (see {@code frc.robot.mechanisms.ExampleDoubleJointedArm}).
 */
public class YamsDoubleJointedArm extends SubsystemBase {

  /** Parameters for one joint of the arm. */
  public static class JointSettings {
    /** CAN ID of this joint's motor controller. */
    public int canId;
    /** Gear reduction stages, rotor → joint. */
    public double[] gearReductionStages = {3, 4, 5};
    /** Segment length, joint to next joint (or tip). */
    public Distance length = Meters.of(0.6);
    /** Segment mass. */
    public Mass mass = Kilograms.of(2.0);
    /** Lower hard limit. Keep generous — the IK solver needs room. */
    public Angle minAngle = Degrees.of(-720);
    /** Upper hard limit. */
    public Angle maxAngle = Degrees.of(720);
    /** Joint angle at robot power-on. */
    public Angle startingAngle = Degrees.of(45);
    /** Proportional gain (rotations → volts). */
    public double kP = 16;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Motion profile cruise velocity. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(180);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360);
  }

  /** Robot-specific double-jointed arm parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "DoubleJointedArm";
    /** Motor controller vendor (both joints). */
    public MechanismMotor.Vendor vendor = MechanismMotor.Vendor.TALON_FX;
    /** Motor physics model (both joints). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Stator current limit (both joints). */
    public Current statorCurrentLimit = Amps.of(40);
    /** Shoulder joint (attached to the robot). */
    public JointSettings lowerJoint = new JointSettings();
    /** Elbow joint (attached to the end of the lower segment). */
    public JointSettings upperJoint = new JointSettings();
  }

  /**
   * AdvantageKit inputs — everything read back from the motors crosses the replay
   * bubble here. Fields stay {@code double} (project convention).
   */
  @AutoLog
  public static class DoubleJointedArmInputs {
    /** Shoulder angle, degrees. */
    public double lowerPositionDegrees;
    /** Elbow angle, degrees. */
    public double upperPositionDegrees;
    /** Shoulder closed-loop setpoint, degrees. */
    public double lowerSetpointDegrees;
    /** Elbow closed-loop setpoint, degrees. */
    public double upperSetpointDegrees;
    /** Shoulder applied voltage. */
    public double lowerAppliedVolts;
    /** Elbow applied voltage. */
    public double upperAppliedVolts;
    /** Shoulder stator current, amps. */
    public double lowerCurrentAmps;
    /** Elbow stator current, amps. */
    public double upperCurrentAmps;
  }

  private final DoubleJointedArmInputsAutoLogged inputs = new DoubleJointedArmInputsAutoLogged();

  private final Settings settings;
  private final SmartMotorControllerConfig lowerMotorConfig;
  private final SmartMotorControllerConfig upperMotorConfig;
  private final SmartMotorController lowerMotor;
  private final SmartMotorController upperMotor;
  private final DoubleJointedArm arm;
  private final TunableGains lowerGains;
  private final TunableGains upperGains;

  /**
   * Builds the double-jointed arm subsystem, both motor wrappers, and simulation.
   *
   * @param settings robot-specific parameters
   */
  public YamsDoubleJointedArm(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    lowerMotorConfig = jointMotorConfig(settings.lowerJoint, "Lower");
    upperMotorConfig = jointMotorConfig(settings.upperJoint, "Upper");
    lowerMotor = MechanismMotor.create(
        settings.vendor, settings.lowerJoint.canId, settings.motorModel, lowerMotorConfig);
    upperMotor = MechanismMotor.create(
        settings.vendor, settings.upperJoint.canId, settings.motorModel, upperMotorConfig);
    arm = new DoubleJointedArm(
        jointArmConfig(lowerMotor, settings.lowerJoint, "Lower"),
        jointArmConfig(upperMotor, settings.upperJoint, "Upper"));
    lowerGains = new TunableGains(settings.name, "lowerJoint",
        settings.lowerJoint.kP, settings.lowerJoint.kI, settings.lowerJoint.kD, 0);
    upperGains = new TunableGains(settings.name, "upperJoint",
        settings.upperJoint.kP, settings.upperJoint.kI, settings.upperJoint.kD, 0);
  }

  private SmartMotorControllerConfig jointMotorConfig(JointSettings joint, String label) {
    return new SmartMotorControllerConfig(this)
        .withClosedLoopController(joint.kP, joint.kI, joint.kD)
        .withTrapezoidalProfile(joint.maxVelocity, joint.maxAcceleration)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(joint.gearReductionStages)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(settings.statorCurrentLimit)
        .withTelemetry(settings.name + label + "Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private ArmConfig jointArmConfig(SmartMotorController motor, JointSettings joint, String label) {
    return new ArmConfig(motor)
        .withLength(joint.length)
        .withMass(joint.mass)
        .withHardLimit(joint.minAngle, joint.maxAngle)
        .withStartingPosition(joint.startingAngle)
        .withTelemetry(settings.name + label, TelemetryVerbosity.HIGH);
  }

  private void updateInputs() {
    inputs.lowerPositionDegrees = arm.getLowerAngle().in(Degrees);
    inputs.upperPositionDegrees = arm.getUpperAngle().in(Degrees);
    inputs.lowerSetpointDegrees = lowerMotor.getMechanismPositionSetpoint()
        .map(sp -> sp.in(Degrees)).orElse(inputs.lowerPositionDegrees);
    inputs.upperSetpointDegrees = upperMotor.getMechanismPositionSetpoint()
        .map(sp -> sp.in(Degrees)).orElse(inputs.upperPositionDegrees);
    inputs.lowerAppliedVolts = lowerMotor.getVoltage().in(Volts);
    inputs.upperAppliedVolts = upperMotor.getVoltage().in(Volts);
    inputs.lowerCurrentAmps = lowerMotor.getStatorCurrent().in(Amps);
    inputs.upperCurrentAmps = upperMotor.getStatorCurrent().in(Amps);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(settings.name, inputs);
    if (lowerGains.hasChanged()) {
      lowerMotorConfig.withClosedLoopController(lowerGains.kP(), lowerGains.kI(), lowerGains.kD());
      lowerMotor.applyConfig(lowerMotorConfig);
    }
    if (upperGains.hasChanged()) {
      upperMotorConfig.withClosedLoopController(upperGains.kP(), upperGains.kI(), upperGains.kD());
      upperMotor.applyConfig(upperMotorConfig);
    }
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

  /** Command: drive both joints to the given angles. Never finishes. */
  public Command goToAngles(Angle lowerAngle, Angle upperAngle) {
    Logger.recordOutput(settings.name + "/CommandedLowerDegrees", lowerAngle.in(Degrees));
    Logger.recordOutput(settings.name + "/CommandedUpperDegrees", upperAngle.in(Degrees));
    return arm.setAngle(lowerAngle, upperAngle);
  }

  /**
   * Command: drive the end effector to an (x, y) position via the YAMS IK solver.
   *
   * @param position    target relative to the shoulder joint, meters
   * @param elbowInvert choose the elbow-up vs elbow-down IK solution
   */
  public Command goToPosition(Translation2d position, boolean elbowInvert) {
    return arm.setPosition(position, elbowInvert);
  }

  /** Command: open-loop duty cycle for both joints. */
  public Command setDutyCycle(double lowerDutyCycle, double upperDutyCycle) {
    return arm.set(lowerDutyCycle, upperDutyCycle);
  }

  /** Command: SysId routine. */
  public Command sysId() {
    return arm.sysId(Volts.of(3), Volts.of(1).per(Second), Seconds.of(10));
  }

  /** Current shoulder angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getLowerAngle() {
    return Degrees.of(inputs.lowerPositionDegrees);
  }

  /** Current elbow angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getUpperAngle() {
    return Degrees.of(inputs.upperPositionDegrees);
  }

  /** Current end-effector position relative to the shoulder, meters. */
  public Translation2d getPosition() {
    return arm.getPosition();
  }

  /** Underlying YAMS mechanism, for advanced use. */
  public DoubleJointedArm getMechanism() {
    return arm;
  }

  /** The settings this mechanism was built with (start positions, limits, ...). */
  public Settings getSettings() {
    return settings;
  }

  /** Stops the closed-loop Notifiers and frees the CAN devices. For unit tests. */
  public void close() {
    lowerMotor.close();
    upperMotor.close();
    MechanismMotor.closeDevice(lowerMotor);
    MechanismMotor.closeDevice(upperMotor);
  }
}
