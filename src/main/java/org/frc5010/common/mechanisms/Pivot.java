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

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.robot.RobotMode;
import org.littletonrobotics.junction.Logger;

/**
 * TalonFX pivot (turret, hood, gravity-free wrist) with selectable control style and
 * REAL / SIM / REPLAY support. The shared engine lives in {@link SingleDofMechanism};
 * this class supplies the angular units, the ARM-type LQR plant (same rotational plant
 * as an arm, no gravity), the {@link DCMotorSim}, and the angle-based command/getter
 * API. Native unit: radians.
 *
 * <p>Supports an absolute CANcoder mounted 1:1 on the pivot
 * ({@code settings.cancoderId}) — fused onboard, so position is correct at power-on
 * without seeding.
 */
public class Pivot extends SingleDofMechanism {

  /** Robot-specific pivot parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Pivot";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** CAN ID of the TalonFX. */
    public int canId;
    /** CAN ID of a follower TalonFX on the same gearbox; −1 = single motor. */
    public int followerCanId = -1;
    /** True if the follower is mounted opposing the lead motor. */
    public boolean followerOpposed = false;
    /** CAN ID of a fused CANcoder mounted 1:1 on the pivot; −1 = rotor sensor. */
    public int cancoderId = -1;
    /** CANcoder reading at the pivot's zero, for the magnet offset. */
    public Angle cancoderOffset = Degrees.of(0);
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {10, 4} = 40:1). */
    public double[] gearReductionStages = {10, 4};
    /** Moment of inertia of the rotating assembly about the pivot axis. */
    public MomentOfInertia moi = KilogramSquareMeters.of(0.5);
    /** Lower limit. */
    public Angle minAngle = Degrees.of(-180);
    /** Upper limit. */
    public Angle maxAngle = Degrees.of(180);
    /** Pivot angle at robot power-on (ignored when a CANcoder is configured). */
    public Angle startingAngle = Degrees.of(0);
    /** Motion profile cruise velocity — keep below free speed ÷ gearing. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(360);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(720);
    /**
     * Use FOC commutation on all control requests (~15% more torque). Requires
     * Phoenix Pro; unlicensed devices fall back to non-FOC with a fault. Set false
     * for non-Pro teams.
     */
    public boolean enableFoc = true;
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);
    /** Drop the goal when the robot is disabled (stay put on re-enable). */
    public boolean clearGoalOnDisable = false;

    /**
     * Canvas to draw this mechanism on. Null (default) = the shared robot-overlay
     * canvas (SmartDashboard -> RobotMechanisms); pass your own Mechanism2d to split
     * mechanisms onto separate widgets (you publish custom canvases yourself).
     */
    public Mechanism2d mechanism2d = null;
    /**
     * Where this mechanism's root sits on the canvas, meters — x along the robot's
     * length, y above the floor (side view). Lets the overlay reflect the real robot
     * layout.
     */
    public Translation2d visualPosition = new Translation2d(2.0, 1.2);
    /**
     * Where this mechanism sits on the robot for the 3D isometric view — robot frame,
     * x forward, y left, z up, meters from robot center at floor level. The rotation
     * re-aims the working plane: identity (default) swings the pivot in the robot's
     * X-Z side-view plane (hood/wrist); use
     * {@link MechanismVisuals3d#YAW_PLANE} to lay the plane flat so the pivot becomes
     * a turret spinning about the vertical axis.
     */
    public Pose3d visualPose3d = new Pose3d(0, 0, 0.6, Rotation3d.kZero);
    /**
     * Optional parent for 3D coupling: when set, {@link #visualPose3d} becomes an offset
     * from this supplier's live attachment pose instead of an absolute robot-frame mount,
     * so the pivot rides another mechanism's moving endpoint.
     */
    public java.util.function.Supplier<Pose3d> visualParent = null;
    // --- LQR weights (live-tunable in DEGREES; these are the initial values) ---
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

  private final Settings settings;
  private final SysIdRoutine sysIdRoutine;
  private final MechanismLigament2d pivotLigament;
  private final MechanismLigament2d goalLigament;

  /**
   * Builds the pivot subsystem, its IO (per {@link RobotMode}), controller, and sim.
   *
   * @param settings robot-specific pivot parameters
   */
  public Pivot(Settings settings) {
    super(baseParams(settings));
    this.settings = settings;

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            volts -> io.setVoltage(volts.in(Volts)),
            log -> log.motor(settings.name)
                .voltage(Volts.of(inputs.appliedVolts))
                .angularPosition(Radians.of(positionNative()))
                .angularVelocity(RadiansPerSecond.of(velocityNative())),
            this));

    Mechanism2d canvas = MechanismVisuals.canvasFor(settings.mechanism2d);
    double rootX = settings.visualPosition.getX();
    double rootY = settings.visualPosition.getY();
    pivotLigament = canvas.getRoot(settings.name + "Root", rootX, rootY)
        .append(new MechanismLigament2d("pivot", 0.4, settings.startingAngle.in(Degrees)));
    goalLigament = canvas.getRoot(settings.name + "GoalRoot", rootX, rootY)
        .append(new MechanismLigament2d("goal", 0.4, settings.startingAngle.in(Degrees), 3,
            new Color8Bit(Color.kWhite)));
  }

  private static BaseParams baseParams(Settings settings) {
    double gearing = totalReduction(settings.gearReductionStages);

    var config = new MechanismIOTalonFX.Config();
    config.canId = settings.canId;
    config.enableFoc = settings.enableFoc;
    config.followerCanId = settings.followerCanId;
    config.followerOpposed = settings.followerOpposed;
    config.cancoderId = settings.cancoderId;
    config.cancoderOffsetRot = settings.cancoderOffset.in(Rotations);
    config.gearing = gearing;
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.softLimitLowRot = settings.minAngle.in(Rotations);
    config.softLimitHighRot = settings.maxAngle.in(Rotations);
    config.startingPositionRot = settings.startingAngle.in(Rotations);
    config.motionMagicCruiseRotPerSec = settings.maxVelocity.in(RotationsPerSecond);
    config.motionMagicAccelRotPerSecSq =
        settings.maxAcceleration.in(RotationsPerSecond.per(Second));
    config.kP = settings.kP;
    config.kI = settings.kI;
    config.kD = settings.kD;
    config.kS = settings.kS;
    config.kV = settings.kV;
    config.kG = 0; // gravity-free mechanism
    config.gravityType = GravityTypeValue.Elevator_Static;

    var params = new BaseParams();
    params.name = settings.name;
    params.nativePerRot = 2 * Math.PI; // radians per mechanism rotation
    params.displayPerNative = 180.0 / Math.PI; // tunables displayed in degrees
    params.io = switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, pivotSim(settings, gearing));
      case REAL -> new MechanismIOTalonFX(config);
    };
    params.profileConstraints = new TrapezoidProfile.Constraints(
        settings.maxVelocity.in(RadiansPerSecond),
        settings.maxAcceleration.in(RadiansPerSecond.per(Second)));
    params.lqrFactory = settings.controlStyle == ControlStyle.LQR
        ? (qPos, qVel, relms) -> MechanismLqr.arm(
            settings.motorModel, gearing,
            settings.moi.in(KilogramSquareMeters),
            qPos, qVel, relms,
            settings.modelPositionTrust.in(Radians),
            settings.modelVelocityTrust.in(RadiansPerSecond),
            settings.encoderPositionTrust.in(Radians),
            // Settings take SysId's rotation units; the plant works in radians.
            settings.characterizedKv / (2 * Math.PI),
            settings.characterizedKa / (2 * Math.PI))
        : null;
    params.initialQelmsPosDisplay = settings.qelmsPosition.in(Degrees);
    params.initialQelmsVelDisplay = settings.qelmsVelocity.in(DegreesPerSecond);
    params.initialRelmsVolts = settings.relms.in(Volts);
    params.kP = settings.kP;
    params.kI = settings.kI;
    params.kD = settings.kD;
    params.kGVolts = 0;
    params.cosineGravity = false;
    params.clearGoalOnDisable = settings.clearGoalOnDisable;
    return params;
  }

  private static MechanismSim pivotSim(Settings settings, double gearing) {
    var sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            settings.motorModel, settings.moi.in(KilogramSquareMeters), gearing),
        settings.motorModel);
    sim.setState(settings.startingAngle.in(Radians), 0);
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

      @Override
      public double getCurrentDrawAmps() {
        return sim.getCurrentDrawAmps();
      }
    };
  }

  private static double totalReduction(double[] stages) {
    double total = 1.0;
    for (double stage : stages) {
      total *= stage;
    }
    return total;
  }

  @Override
  protected void logGoal(double goalNative) {
    Logger.recordOutput(settings.name + "/GoalDegrees", Math.toDegrees(goalNative));
  }

  @Override
  protected void updateVisualization() {
    double goalRad = mode == OutputMode.GOAL ? goalNative : positionNative();
    pivotLigament.setAngle(Math.toDegrees(positionNative()));
    goalLigament.setAngle(Math.toDegrees(goalRad));

    Pose3d mount = MechanismVisuals3d.resolveMount(settings.visualPose3d, settings.visualParent);
    Translation3d base = MechanismVisuals3d.planarPoint(mount, 0, 0);
    MechanismVisuals3d.publish(settings.name, java.util.List.of(
        new MechanismVisuals3d.Segment("goal", base,
            MechanismVisuals3d.planarOffset(mount, base, goalRad, 0.4), "#ffffff", 1),
        new MechanismVisuals3d.Segment("pivot", base,
            MechanismVisuals3d.planarOffset(mount, base, positionNative(), 0.4),
            "#d2a8ff", 3)));
  }

  /** Command: rotate the pivot to the given angle. Never finishes. */
  public Command goToAngle(Angle angle) {
    Logger.recordOutput(settings.name + "/CommandedAngleDegrees", angle.in(Degrees));
    return goalCommand(angle.in(Radians), settings.name + " GoToAngle");
  }

  /** Command: SysId routine for characterizing kS/kV/kA, guarded by the travel limits. */
  public Command sysId() {
    Trigger nearMax = isAtAngle(settings.maxAngle, Degrees.of(8));
    Trigger nearMin = isAtAngle(settings.minAngle, Degrees.of(8));
    return Commands.sequence(
            Commands.runOnce(this::enterVoltageMode),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(nearMax),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(nearMin),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(nearMax),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(nearMin))
        .finallyDo(this::exitVoltageMode)
        .withName(settings.name + " SysId");
  }

  /** Current pivot angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getAngle() {
    return Radians.of(positionNative());
  }

  /** Trigger: true while the pivot is within {@code tolerance} of {@code angle}. */
  public Trigger isAtAngle(Angle angle, Angle tolerance) {
    return new Trigger(
        () -> Math.abs(positionNative() - angle.in(Radians)) <= tolerance.in(Radians));
  }

  /** The settings this mechanism was built with (start positions, limits, ...). */
  public Settings getSettings() {
    return settings;
  }

  /**
   * The live robot-frame pose where a child mechanism mounts: the pivot tip, rotated by
   * the current angle. Pass {@code pivot::attachmentPose} as another mechanism's
   * {@code visualParent}.
   */
  public Pose3d attachmentPose() {
    Pose3d mount = MechanismVisuals3d.resolveMount(settings.visualPose3d, settings.visualParent);
    Translation3d base = MechanismVisuals3d.planarPoint(mount, 0, 0);
    Translation3d tip = MechanismVisuals3d.planarOffset(mount, base, positionNative(), 0.4);
    return new Pose3d(tip, mount.getRotation().rotateBy(new Rotation3d(0, -positionNative(), 0)));
  }
}
