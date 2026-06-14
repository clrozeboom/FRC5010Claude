package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * Two-jointed arm (shoulder + elbow) on two TalonFXs, with REAL / SIM / REPLAY support
 * through one {@link MechanismIO} per joint.
 *
 * <p><b>Controller:</b> profiled PID per joint via onboard MotionMagic with Arm_Cosine
 * gravity compensation — <em>not</em> LQR: the single-DOF LQR plants don't model a
 * coupled two-joint system. Gains are live-tunable per joint under
 * {@code /Tuning/<name>/lowerJoint_*} and {@code upperJoint_*}.
 *
 * <p>SIM uses an independent {@link SingleJointedArmSim} per joint (gravity on each;
 * joint coupling is not modeled).
 */
public class DoubleJointedArm extends SubsystemBase implements AutoCloseable {

  /** Parameters for one joint of the arm. */
  public static class JointSettings {
    /** CAN ID of this joint's TalonFX. */
    public int canId;
    /** Gear reduction stages, rotor → joint. */
    public double[] gearReductionStages = {3, 4, 5};
    /** Segment length, joint to next joint (or tip). */
    public Distance length = Meters.of(0.6);
    /** Segment mass. */
    public Mass mass = Kilograms.of(2.0);
    /** Lower limit. */
    public Angle minAngle = Degrees.of(-720);
    /** Upper limit. */
    public Angle maxAngle = Degrees.of(720);
    /** Joint angle at robot power-on. */
    public Angle startingAngle = Degrees.of(45);
    /** Proportional gain, volts per joint rotation of error (onboard). */
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
    /** Motor physics model (both joints). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Drop the goals when the robot is disabled (stay put on re-enable). */
    public boolean clearGoalOnDisable = false;
    /**
     * Use FOC commutation on all control requests (~15% more torque). Requires
     * Phoenix Pro; unlicensed devices fall back to non-FOC with a fault. Set false
     * for non-Pro teams.
     */
    public boolean enableFoc = true;
    /** Stator current limit (both joints). */
    public Current statorCurrentLimit = Amps.of(40);
    /**
     * Where the shoulder joint sits on the robot for the 3D isometric view — robot
     * frame, x forward, y left, z up, meters from robot center at floor level. The
     * rotation re-aims the working plane: identity (default) swings both joints in
     * the robot's X-Z side-view plane.
     */
    public Pose3d visualPose3d = new Pose3d(-0.1, 0, 0.3, Rotation3d.kZero);
    /** Shoulder joint (attached to the robot). */
    public JointSettings lowerJoint = new JointSettings();
    /** Elbow joint (attached to the end of the lower segment). */
    public JointSettings upperJoint = new JointSettings();
  }

  private final Settings settings;
  private final MechanismIO lowerIo;
  private final MechanismIO upperIo;
  private final MechanismIOInputsAutoLogged lowerInputs = new MechanismIOInputsAutoLogged();
  private final MechanismIOInputsAutoLogged upperInputs = new MechanismIOInputsAutoLogged();
  private final TunableGains lowerGains;
  private final TunableGains upperGains;

  private final edu.wpi.first.wpilibj.Alert lowerDisconnectedAlert;
  private final edu.wpi.first.wpilibj.Alert upperDisconnectedAlert;

  private boolean hasGoal = false;
  private boolean wasEnabled = false;
  private double lowerGoalRot;
  private double upperGoalRot;

  /**
   * Builds the double-jointed arm subsystem, both IOs (per {@link RobotMode}), and sims.
   *
   * @param settings robot-specific parameters
   */
  public DoubleJointedArm(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    lowerIo = jointIo(settings.lowerJoint);
    upperIo = jointIo(settings.upperJoint);
    lowerGains = new TunableGains(settings.name, "lowerJoint",
        settings.lowerJoint.kP, settings.lowerJoint.kI, settings.lowerJoint.kD, 0);
    upperGains = new TunableGains(settings.name, "upperJoint",
        settings.upperJoint.kP, settings.upperJoint.kI, settings.upperJoint.kD, 0);
    lowerDisconnectedAlert = new edu.wpi.first.wpilibj.Alert(
        settings.name + " lower TalonFX disconnected",
        edu.wpi.first.wpilibj.Alert.AlertType.kError);
    upperDisconnectedAlert = new edu.wpi.first.wpilibj.Alert(
        settings.name + " upper TalonFX disconnected",
        edu.wpi.first.wpilibj.Alert.AlertType.kError);
  }

  private MechanismIO jointIo(JointSettings joint) {
    var config = new MechanismIOTalonFX.Config();
    config.canId = joint.canId;
    config.enableFoc = settings.enableFoc;
    config.gearing = product(joint.gearReductionStages);
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.softLimitLowRot = joint.minAngle.in(Radians) / (2 * Math.PI);
    config.softLimitHighRot = joint.maxAngle.in(Radians) / (2 * Math.PI);
    config.startingPositionRot = joint.startingAngle.in(Radians) / (2 * Math.PI);
    config.motionMagicCruiseRotPerSec = joint.maxVelocity.in(RotationsPerSecond);
    config.motionMagicAccelRotPerSecSq = joint.maxAcceleration.in(RotationsPerSecond.per(Second));
    config.kP = joint.kP;
    config.kI = joint.kI;
    config.kD = joint.kD;
    config.gravityType = GravityTypeValue.Arm_Cosine;

    return switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, jointSim(joint));
      case REAL -> new MechanismIOTalonFX(config);
    };
  }

  private MechanismSim jointSim(JointSettings joint) {
    var sim = new SingleJointedArmSim(
        settings.motorModel,
        product(joint.gearReductionStages),
        SingleJointedArmSim.estimateMOI(joint.length.in(Meters), joint.mass.in(Kilograms)),
        joint.length.in(Meters),
        joint.minAngle.in(Radians),
        joint.maxAngle.in(Radians),
        true,
        joint.startingAngle.in(Radians));
    return new MechanismSim() {
      @Override
      public void setInputVoltage(double volts) {
        sim.setInputVoltage(volts);
      }

      @Override
      public void update(double dtSeconds) {
        sim.update(dtSeconds);
      }

      @Override
      public double getPositionRot() {
        return sim.getAngleRads() / (2 * Math.PI);
      }

      @Override
      public double getVelocityRotPerSec() {
        return sim.getVelocityRadPerSec() / (2 * Math.PI);
      }
    };
  }

  private static double product(double[] stages) {
    double total = 1.0;
    for (double stage : stages) {
      total *= stage;
    }
    return total;
  }

  @Override
  public void periodic() {
    lowerIo.updateInputs(lowerInputs);
    upperIo.updateInputs(upperInputs);
    Logger.processInputs(settings.name + "/Lower", lowerInputs);
    Logger.processInputs(settings.name + "/Upper", upperInputs);
    lowerDisconnectedAlert.set(!lowerInputs.connected);
    upperDisconnectedAlert.set(!upperInputs.connected);

    if (lowerGains.hasChanged()) {
      lowerIo.setPidGains(lowerGains.kP(), lowerGains.kI(), lowerGains.kD());
    }
    if (upperGains.hasChanged()) {
      upperIo.setPidGains(upperGains.kP(), upperGains.kI(), upperGains.kD());
    }

    boolean enabled = DriverStation.isEnabled();
    if (!enabled && wasEnabled && settings.clearGoalOnDisable) {
      hasGoal = false;
    }
    wasEnabled = enabled;

    if (!enabled) {
      // Explicit neutral — the simulated Talon doesn't self-neutral while DS packets
      // stay fresh.
      lowerIo.stop();
      upperIo.stop();
    } else if (hasGoal) {
      lowerIo.runPosition(lowerGoalRot);
      upperIo.runPosition(upperGoalRot);
    }

    // 3D view: both joint angles are absolute in this model, so each segment is laid
    // out directly in the working plane and chained tip-to-tail.
    Pose3d mount = settings.visualPose3d;
    Translation3d shoulder = MechanismVisuals3d.planarPoint(mount, 0, 0);
    Translation3d elbow = MechanismVisuals3d.planarOffset(mount, shoulder,
        lowerInputs.positionRot * 2 * Math.PI, settings.lowerJoint.length.in(Meters));
    Translation3d tip = MechanismVisuals3d.planarOffset(mount, elbow,
        upperInputs.positionRot * 2 * Math.PI, settings.upperJoint.length.in(Meters));
    MechanismVisuals3d.publish(settings.name, java.util.List.of(
        new MechanismVisuals3d.Segment("lower", shoulder, elbow, "#ff7b72", 3),
        new MechanismVisuals3d.Segment("upper", elbow, tip, "#ffa198", 3)));
  }

  /** Command: drive both joints to the given angles. Never finishes. */
  public Command goToAngles(Angle lowerAngle, Angle upperAngle) {
    Logger.recordOutput(settings.name + "/CommandedLowerDegrees", lowerAngle.in(Degrees));
    Logger.recordOutput(settings.name + "/CommandedUpperDegrees", upperAngle.in(Degrees));
    return Commands.run(() -> {
      hasGoal = true;
      lowerGoalRot = lowerAngle.in(Radians) / (2 * Math.PI);
      upperGoalRot = upperAngle.in(Radians) / (2 * Math.PI);
    }, this).withName(settings.name + " GoToAngles");
  }

  /** Command: open-loop duty cycle for both joints. Neutral when it ends. */
  public Command setDutyCycle(double lowerDutyCycle, double upperDutyCycle) {
    return Commands.run(() -> {
      hasGoal = false;
      lowerIo.setDutyCycle(lowerDutyCycle);
      upperIo.setDutyCycle(upperDutyCycle);
    }, this).finallyDo(() -> {
      lowerIo.stop();
      upperIo.stop();
    }).withName(settings.name + " DutyCycle");
  }

  /** Current shoulder angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getLowerAngle() {
    return Radians.of(lowerInputs.positionRot * 2 * Math.PI);
  }

  /** Current elbow angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getUpperAngle() {
    return Radians.of(upperInputs.positionRot * 2 * Math.PI);
  }

  /** The settings this mechanism was built with. */
  public Settings getSettings() {
    return settings;
  }

  /** Stops control and frees both CAN devices. For unit tests. */
  @Override
  public void close() {
    MechanismVisuals3d.remove(settings.name);
    lowerIo.close();
    upperIo.close();
  }
}
