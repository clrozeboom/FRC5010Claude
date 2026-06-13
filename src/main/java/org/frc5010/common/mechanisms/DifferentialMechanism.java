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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * Differential (tilt + twist) wrist on two TalonFXs, with REAL / SIM / REPLAY support
 * through one {@link MechanismIO} per motor.
 *
 * <p>Two motors drive one gear assembly: spinning together produces <em>tilt</em>,
 * spinning opposite produces <em>twist</em>. The mixing convention here is
 * {@code left = tilt + twist}, {@code right = tilt − twist}, so
 * {@code tilt = (left + right) / 2} and {@code twist = (left − right) / 2}.
 *
 * <p><b>Controller:</b> profiled PID per motor via onboard MotionMagic — <em>not</em>
 * LQR: the single-DOF LQR plants don't model the coupled differential. Gains are shared
 * by both motors (the mechanism is symmetric) and live-tunable under
 * {@code /Tuning/<name>/motors_*}.
 */
public class DifferentialMechanism extends SubsystemBase implements AutoCloseable {

  /** Robot-specific differential mechanism parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "DiffWrist";
    /** CAN ID of the left TalonFX. */
    public int leftCanId;
    /** CAN ID of the right TalonFX. */
    public int rightCanId;
    /** Motor physics model (both motors). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (both motors). */
    public double[] gearReductionStages = {3, 4, 5};
    /** Effective lever length for the MOI estimate. */
    public Distance length = Meters.of(0.3);
    /** Mass carried by the wrist for the MOI estimate. */
    public Mass mass = Kilograms.of(1.8);
    /** Tilt angle at robot power-on. */
    public Angle startingTilt = Degrees.of(90);
    /** Twist angle at robot power-on. */
    public Angle startingTwist = Degrees.of(0);
    /**
     * Canvas to draw this mechanism on. Null (default) = the shared robot-overlay
     * canvas (SmartDashboard -> RobotMechanisms); pass your own Mechanism2d to split
     * mechanisms onto separate widgets (you publish custom canvases yourself).
     */
    public Mechanism2d mechanism2d = null;
    /**
     * Where the wrist sits on the canvas, meters — x along the robot's length,
     * y above the floor (side view).
     */
    public Translation2d visualPosition = new Translation2d(2.4, 0.8);
    /**
     * Where the wrist sits on the robot for the 3D isometric view — robot frame,
     * x forward, y left, z up, meters from robot center at floor level. The rotation
     * re-aims the tilt plane: identity (default) tilts in the robot's X-Z side-view
     * plane; the twist flag swings out of that plane.
     */
    public Pose3d visualPose3d = new Pose3d(0.25, 0, 0.5, Rotation3d.kZero);
    /** Proportional gain, both motors, volts per rotation of error (onboard). */
    public double kP = 16;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Motion profile cruise velocity. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(180);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360);
    /** Drop the goals when the robot is disabled (stay put on re-enable). */
    public boolean clearGoalOnDisable = false;
    /**
     * Use FOC commutation on all control requests (~15% more torque). Requires
     * Phoenix Pro; unlicensed devices fall back to non-FOC with a fault. Set false
     * for non-Pro teams.
     */
    public boolean enableFoc = true;
    /** Stator current limit (both motors). */
    public Current statorCurrentLimit = Amps.of(40);
  }

  private final Settings settings;
  private final MechanismIO leftIo;
  private final MechanismIO rightIo;
  private final MechanismIOInputsAutoLogged leftInputs = new MechanismIOInputsAutoLogged();
  private final MechanismIOInputsAutoLogged rightInputs = new MechanismIOInputsAutoLogged();
  private final TunableGains gains;

  private final edu.wpi.first.wpilibj.Alert leftDisconnectedAlert;
  private final edu.wpi.first.wpilibj.Alert rightDisconnectedAlert;

  private boolean hasGoal = false;
  private boolean wasEnabled = false;
  private double tiltGoalRot;
  private double twistGoalRot;

  private final MechanismLigament2d tiltLigament;
  private final MechanismLigament2d twistLigament;

  /**
   * Builds the differential mechanism subsystem, both IOs (per {@link RobotMode}), and sims.
   *
   * @param settings robot-specific parameters
   */
  public DifferentialMechanism(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    // left = tilt + twist, right = tilt − twist
    double leftStartRot = (settings.startingTilt.in(Radians) + settings.startingTwist.in(Radians))
        / (2 * Math.PI);
    double rightStartRot = (settings.startingTilt.in(Radians) - settings.startingTwist.in(Radians))
        / (2 * Math.PI);
    leftIo = motorIo(settings.leftCanId, leftStartRot);
    rightIo = motorIo(settings.rightCanId, rightStartRot);
    gains = new TunableGains(settings.name, "motors",
        settings.kP, settings.kI, settings.kD, 0);
    leftDisconnectedAlert = new edu.wpi.first.wpilibj.Alert(
        settings.name + " left TalonFX disconnected",
        edu.wpi.first.wpilibj.Alert.AlertType.kError);
    rightDisconnectedAlert = new edu.wpi.first.wpilibj.Alert(
        settings.name + " right TalonFX disconnected",
        edu.wpi.first.wpilibj.Alert.AlertType.kError);

    Mechanism2d canvas = MechanismVisuals.canvasFor(settings.mechanism2d);
    tiltLigament = canvas.getRoot(settings.name + "Root",
            settings.visualPosition.getX(), settings.visualPosition.getY())
        .append(new MechanismLigament2d("tilt", 0.3, settings.startingTilt.in(Degrees)));
    twistLigament = tiltLigament.append(
        new MechanismLigament2d("twist", 0.15, settings.startingTwist.in(Degrees), 4,
            new edu.wpi.first.wpilibj.util.Color8Bit(edu.wpi.first.wpilibj.util.Color.kOrange)));
  }

  private MechanismIO motorIo(int canId, double startingRot) {
    var config = new MechanismIOTalonFX.Config();
    config.canId = canId;
    config.enableFoc = settings.enableFoc;
    config.gearing = product(settings.gearReductionStages);
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.startingPositionRot = startingRot;
    config.motionMagicCruiseRotPerSec = settings.maxVelocity.in(RotationsPerSecond);
    config.motionMagicAccelRotPerSecSq =
        settings.maxAcceleration.in(RotationsPerSecond.per(Second));
    config.kP = settings.kP;
    config.kI = settings.kI;
    config.kD = settings.kD;
    config.gravityType = GravityTypeValue.Elevator_Static;

    return switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, motorSim(startingRot));
      case REAL -> new MechanismIOTalonFX(config);
    };
  }

  private MechanismSim motorSim(double startingRot) {
    double moi = settings.mass.in(Kilograms) * Math.pow(settings.length.in(Meters), 2) / 3.0;
    var sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            settings.motorModel, moi, product(settings.gearReductionStages)),
        settings.motorModel);
    sim.setState(startingRot * 2 * Math.PI, 0);
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
        return sim.getAngularPositionRad() / (2 * Math.PI);
      }

      @Override
      public double getVelocityRotPerSec() {
        return sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
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
    leftIo.updateInputs(leftInputs);
    rightIo.updateInputs(rightInputs);
    Logger.processInputs(settings.name + "/Left", leftInputs);
    Logger.processInputs(settings.name + "/Right", rightInputs);
    leftDisconnectedAlert.set(!leftInputs.connected);
    rightDisconnectedAlert.set(!rightInputs.connected);

    if (gains.hasChanged()) {
      leftIo.setPidGains(gains.kP(), gains.kI(), gains.kD());
      rightIo.setPidGains(gains.kP(), gains.kI(), gains.kD());
    }

    boolean enabled = DriverStation.isEnabled();
    if (!enabled && wasEnabled && settings.clearGoalOnDisable) {
      hasGoal = false;
    }
    wasEnabled = enabled;

    if (!enabled) {
      // Explicit neutral — the simulated Talon doesn't self-neutral while DS packets
      // stay fresh.
      leftIo.stop();
      rightIo.stop();
    } else if (hasGoal) {
      leftIo.runPosition(tiltGoalRot + twistGoalRot);
      rightIo.runPosition(tiltGoalRot - twistGoalRot);
    }

    Logger.recordOutput(settings.name + "/TiltDegrees", getTilt().in(Degrees));
    Logger.recordOutput(settings.name + "/TwistDegrees", getTwist().in(Degrees));
    tiltLigament.setAngle(getTilt().in(Degrees));
    twistLigament.setAngle(getTwist().in(Degrees));

    // 3D view: the tilt segment lies in the working plane; the twist "flag" starts
    // perpendicular to it within the plane and rotates about the segment's own axis
    // by the twist angle — so twist visibly swings the flag out of plane.
    double tiltRad = getTilt().in(Radians);
    double twistRad = getTwist().in(Radians);
    Pose3d mount = settings.visualPose3d;
    Translation3d base = MechanismVisuals3d.planarPoint(mount, 0, 0);
    Translation3d tip = MechanismVisuals3d.planarOffset(mount, base, tiltRad, 0.3);
    Translation3d flagTip = MechanismVisuals3d.localOffset(mount, tip, new Translation3d(
        -Math.sin(tiltRad) * Math.cos(twistRad) * 0.15,
        -Math.sin(twistRad) * 0.15,
        Math.cos(tiltRad) * Math.cos(twistRad) * 0.15));
    MechanismVisuals3d.publish(settings.name, java.util.List.of(
        new MechanismVisuals3d.Segment("tilt", base, tip, "#f2cc60", 3),
        new MechanismVisuals3d.Segment("twist", tip, flagTip, "#ffa657", 2)));
  }

  /** Command: drive the wrist to the given tilt and twist angles. Never finishes. */
  public Command goToAngles(Angle tilt, Angle twist) {
    Logger.recordOutput(settings.name + "/CommandedTiltDegrees", tilt.in(Degrees));
    Logger.recordOutput(settings.name + "/CommandedTwistDegrees", twist.in(Degrees));
    return Commands.run(() -> {
      hasGoal = true;
      tiltGoalRot = tilt.in(Radians) / (2 * Math.PI);
      twistGoalRot = twist.in(Radians) / (2 * Math.PI);
    }, this).withName(settings.name + " GoToAngles");
  }

  /** Command: open-loop duty cycle (tilt = common mode, twist = differential mode). */
  public Command setDutyCycle(double tilt, double twist) {
    return Commands.run(() -> {
      hasGoal = false;
      leftIo.setDutyCycle(tilt + twist);
      rightIo.setDutyCycle(tilt - twist);
    }, this).finallyDo(() -> {
      leftIo.stop();
      rightIo.stop();
    }).withName(settings.name + " DutyCycle");
  }

  /** Current tilt angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getTilt() {
    return Radians.of((leftInputs.positionRot + rightInputs.positionRot) * Math.PI);
  }

  /** Current twist angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getTwist() {
    return Radians.of((leftInputs.positionRot - rightInputs.positionRot) * Math.PI);
  }

  /** The settings this mechanism was built with. */
  public Settings getSettings() {
    return settings;
  }

  /** Stops control and frees both CAN devices. For unit tests. */
  @Override
  public void close() {
    MechanismVisuals3d.remove(settings.name);
    leftIo.close();
    rightIo.close();
  }
}
