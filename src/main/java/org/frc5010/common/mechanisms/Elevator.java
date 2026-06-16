package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.List;
import java.util.Set;
import org.frc5010.common.robot.RobotMode;
import org.littletonrobotics.junction.Logger;

/**
 * TalonFX elevator with selectable control style and full REAL / SIM / REPLAY support.
 * The shared engine (goal state machine, LQR/MotionMagic dispatch, tuning, alerts,
 * enable transitions) lives in {@link SingleDofMechanism}; this class supplies the
 * linear units, the elevator LQR plant, the {@link ElevatorSim}, homing, and the
 * meters-based command/getter API. Native unit: meters.
 */
public class Elevator extends SingleDofMechanism {

  /** Robot-specific elevator parameters. Populate the fields, then construct {@link Elevator}. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Elevator";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** CAN ID of the TalonFX. */
    public int canId;
    /** CAN ID of a follower TalonFX on the same gearbox; −1 = single motor. */
    public int followerCanId = -1;
    /** True if the follower is mounted opposing the lead motor. */
    public boolean followerOpposed = false;
    /**
     * Where to draw the follower as an offset mirror of this mechanism — the same
     * geometry (frame + carriage) redrawn at this offset from the mount, in the mount's
     * local frame (x = plane horizontal, y = plane normal, z = plane vertical), meters.
     * Use it to show the far side of the elevator. Only drawn when {@link #followerCanId}
     * is set; the mirror tracks the live carriage every cycle.
     */
    public Translation3d followerVisualOffset = new Translation3d(0, 0.5, 0);
    /** Motor physics model (count = motors on the gearbox, including the follower). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {4, 3} = 12:1). */
    public double[] gearReductionStages = {4, 3};
    /** Drum/sprocket circumference — carriage travel per drum rotation. */
    public Distance drumCircumference = Inches.of(5.5);
    /** Mass of the moving carriage (plus load). */
    public Mass carriageMass = Kilograms.of(6.0);
    /** Lowest carriage position (soft limit + sim hard limit). */
    public Distance minHeight = Meters.of(0);
    /** Highest carriage position (soft limit + sim hard limit). */
    public Distance maxHeight = Meters.of(1.5);
    /** Carriage position at robot power-on. */
    public Distance startingHeight = Meters.of(0);
    /**
     * Motion profile cruise velocity. Must be achievable: below
     * motorFreeSpeed / gearing × drumCircumference, or the profile runs away from the
     * mechanism and the controller saturates/overshoots chasing it.
     */
    public LinearVelocity maxVelocity = MetersPerSecond.of(0.9);
    /** Motion profile acceleration. */
    public LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    /** Gravity feedforward (volts to hold the carriage) — from SysId or sim ramp. */
    public Voltage kG = Volts.of(0);
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
     * Where this mechanism sits on the robot for the 3D isometric view — robot frame,
     * x forward, y left, z up, meters from robot center at floor level. The rotation
     * re-aims the working plane: identity (default) keeps it in the robot's X-Z
     * side-view plane, exactly like the 2D canvas.
     */
    public Pose3d visualPose3d = new Pose3d(0.2, 0, 0, Rotation3d.kZero);
    /**
     * Optional parent for 3D coupling: when set, {@link #visualPose3d} becomes an offset
     * from this supplier's live attachment pose instead of an absolute robot-frame mount,
     * so this mechanism rides another's moving endpoint (e.g. an elevator carriage). Use
     * the parent's {@code attachmentPose} method reference.
     */
    public java.util.function.Supplier<Pose3d> visualParent = null;
    /**
     * Structural offset from the parent's endpoint to where this mechanism attaches,
     * expressed in the parent's attachment frame (the bracket/standoff carrying it off
     * the parent). Applied before {@link #visualPose3d} when {@link #visualParent} is
     * set; identity (default) mounts straight on the parent's endpoint.
     */
    public Transform3d visualParentOffset = new Transform3d();
    // --- Homing (current-spike zeroing; see homeCommand()) ---
    /** Voltage applied while homing toward the bottom hard stop (negative = down). */
    public Voltage homingVoltage = Volts.of(-1.5);
    /**
     * Stator current that indicates the hard stop has been reached. Keep well below
     * {@link #statorCurrentLimit}: the Talon's limiter caps the stall current, so a
     * threshold at/above the limit never triggers (in sim the observable ceiling is
     * ~0.75 × the limit because Phoenix and WPILib use slightly different motor models).
     */
    public Current homingCurrentThreshold = Amps.of(20);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /**
     * Position error tolerance. Smaller = more aggressive. Note the RIO loop runs at
     * 20 ms, so weights tighter than ~1 inch tend to oscillate.
     */
    public Distance qelmsPosition = Inches.of(2);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public LinearVelocity qelmsVelocity = MetersPerSecond.of(0.5);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard, units = drum rotations) ---
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

  private final Settings settings;
  private final double metersPerRot;
  private final SysIdRoutine sysIdRoutine;

  /**
   * Builds the elevator subsystem, its IO (per {@link RobotMode}), controller, and sim.
   *
   * @param settings robot-specific elevator parameters
   */
  public Elevator(Settings settings) {
    super(baseParams(settings));
    this.settings = settings;
    this.metersPerRot = settings.drumCircumference.in(Meters);

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            volts -> io.setVoltage(volts.in(Volts)),
            log -> log.motor(settings.name)
                .voltage(Volts.of(inputs.appliedVolts))
                .linearPosition(Meters.of(positionNative()))
                .linearVelocity(MetersPerSecond.of(velocityNative())),
            this));
  }

  private static BaseParams baseParams(Settings settings) {
    double gearing = totalReduction(settings.gearReductionStages);
    double metersPerRot = settings.drumCircumference.in(Meters);

    var config = new MechanismIOTalonFX.Config();
    config.canId = settings.canId;
    config.enableFoc = settings.enableFoc;
    config.followerCanId = settings.followerCanId;
    config.followerOpposed = settings.followerOpposed;
    config.gearing = gearing;
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.softLimitLowRot = settings.minHeight.in(Meters) / metersPerRot;
    config.softLimitHighRot = settings.maxHeight.in(Meters) / metersPerRot;
    config.startingPositionRot = settings.startingHeight.in(Meters) / metersPerRot;
    config.motionMagicCruiseRotPerSec = settings.maxVelocity.in(MetersPerSecond) / metersPerRot;
    config.motionMagicAccelRotPerSecSq =
        settings.maxAcceleration.in(MetersPerSecondPerSecond) / metersPerRot;
    config.kP = settings.kP;
    config.kI = settings.kI;
    config.kD = settings.kD;
    config.kS = settings.kS;
    config.kV = settings.kV;
    config.kG = settings.kG.in(Volts);
    config.gravityType = GravityTypeValue.Elevator_Static;

    var params = new BaseParams();
    params.name = settings.name;
    params.nativePerRot = metersPerRot;
    params.displayPerNative = 1.0; // tunables in meters = native units
    params.io = switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, elevatorSim(settings, gearing, metersPerRot));
      case REAL -> new MechanismIOTalonFX(config);
    };
    params.profileConstraints = new TrapezoidProfile.Constraints(
        settings.maxVelocity.in(MetersPerSecond),
        settings.maxAcceleration.in(MetersPerSecondPerSecond));
    params.lqrFactory = settings.controlStyle == ControlStyle.LQR
        ? (qPos, qVel, relms) -> MechanismLqr.elevator(
            settings.motorModel, gearing,
            settings.carriageMass.in(Kilograms), metersPerRot / (2 * Math.PI),
            qPos, qVel, relms,
            settings.modelPositionTrust.in(Meters),
            settings.modelVelocityTrust.in(MetersPerSecond),
            settings.encoderPositionTrust.in(Meters),
            settings.characterizedKv, settings.characterizedKa)
        : null;
    params.initialQelmsPosDisplay = settings.qelmsPosition.in(Meters);
    params.initialQelmsVelDisplay = settings.qelmsVelocity.in(MetersPerSecond);
    params.initialRelmsVolts = settings.relms.in(Volts);
    params.kP = settings.kP;
    params.kI = settings.kI;
    params.kD = settings.kD;
    params.kGVolts = settings.kG.in(Volts);
    params.cosineGravity = false;
    params.clearGoalOnDisable = settings.clearGoalOnDisable;
    return params;
  }

  private static MechanismSim elevatorSim(Settings settings, double gearing, double metersPerRot) {
    var sim = new ElevatorSim(
        settings.motorModel,
        gearing,
        settings.carriageMass.in(Kilograms),
        metersPerRot / (2 * Math.PI),
        settings.minHeight.in(Meters),
        settings.maxHeight.in(Meters),
        true,
        settings.startingHeight.in(Meters));
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
        return sim.getPositionMeters() / metersPerRot;
      }

      @Override
      public double getVelocityRotPerSec() {
        return sim.getVelocityMetersPerSecond() / metersPerRot;
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
    Logger.recordOutput(settings.name + "/GoalMeters", goalNative);
  }

  @Override
  protected void updateVisualization() {
    double height = Math.max(0.02, positionNative());
    double goal = Math.max(0.02, mode == OutputMode.GOAL ? goalNative : positionNative());

    Pose3d mount = MechanismVisuals3d.resolveMount(
        settings.visualPose3d, settings.visualParent, settings.visualParentOffset);
    var segments = new java.util.ArrayList<>(elevatorSegments(mount, height, goal));
    appendFollowerMirror(segments, mount, settings.followerCanId,
        settings.followerVisualOffset, m -> elevatorSegments(m, height, goal));
    MechanismVisuals3d.publish(settings.name, segments);
  }

  /** The frame/goal/carriage segments for one mount — drawn again at the follower offset. */
  private List<MechanismVisuals3d.Segment> elevatorSegments(Pose3d mount, double height, double goal) {
    return List.of(
        new MechanismVisuals3d.Segment("frame",
            MechanismVisuals3d.planarPoint(mount, 0, settings.minHeight.in(Meters)),
            MechanismVisuals3d.planarPoint(mount, 0, settings.maxHeight.in(Meters)),
            "#666666", 1),
        new MechanismVisuals3d.Segment("goal",
            MechanismVisuals3d.planarPoint(mount, -0.07, goal),
            MechanismVisuals3d.planarPoint(mount, 0.07, goal),
            "#ffffff", 1),
        new MechanismVisuals3d.Segment("carriage",
            MechanismVisuals3d.planarPoint(mount, -0.1, height),
            MechanismVisuals3d.planarPoint(mount, 0.1, height),
            "#58a6ff", 3));
  }

  /** Command: drive the carriage to the given height. Never finishes. */
  public Command goToHeight(Distance height) {
    Logger.recordOutput(settings.name + "/CommandedHeightMeters", height.in(Meters));
    return goalCommand(height.in(Meters), settings.name + " GoToHeight");
  }

  /**
   * Command: home the elevator — drive gently into the bottom hard stop (soft limits
   * temporarily disabled), detect the stall via a debounced current spike, re-seed the
   * sensor to {@code minHeight}, and stop. Run this once after power-on whenever the
   * carriage may not be at its configured starting position.
   */
  public Command homeCommand() {
    return Commands.defer(() -> {
      Debouncer stalled = new Debouncer(0.25);
      return Commands.runOnce(() -> {
            enterVoltageMode();
            io.setSoftLimitsEnabled(false);
          })
          // Drive blind for the first 0.4 s: breakaway/startup current transients
          // would otherwise satisfy the stall detector before the carriage moves.
          .andThen(Commands.run(() -> io.setVoltage(settings.homingVoltage.in(Volts)), this)
              .withTimeout(0.4))
          .andThen(Commands.run(() -> io.setVoltage(settings.homingVoltage.in(Volts)), this)
              .until(() -> stalled.calculate(
                  inputs.statorCurrentAmps > settings.homingCurrentThreshold.in(Amps))))
          .andThen(Commands.runOnce(
              () -> io.setSensorPosition(settings.minHeight.in(Meters) / metersPerRot)))
          .finallyDo(() -> {
            io.setSoftLimitsEnabled(true);
            exitVoltageMode();
          });
    }, Set.of(this)).withName(settings.name + " Home");
  }

  /** Command: SysId routine for characterizing kG/kS/kV/kA, guarded by the travel limits. */
  public Command sysId() {
    return sysId(settings.minHeight, settings.maxHeight);
  }

  /**
   * Command: SysId routine limited to a custom height range.
   * Use this for first-time characterization when you want to stay within a safe
   * fraction of full travel (e.g. {@code maxLimit = Meters.of(maxHeight * 0.8)}).
   */
  public Command sysId(Distance minLimit, Distance maxLimit) {
    Trigger nearTop = isAtHeight(maxLimit, Inches.of(3));
    Trigger nearBottom = isAtHeight(minLimit, Inches.of(3));
    return Commands.sequence(
            Commands.runOnce(this::enterVoltageMode),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(nearTop),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(nearBottom),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(nearTop),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(nearBottom))
        .finallyDo(this::exitVoltageMode)
        .withName(settings.name + " SysId");
  }

  /**
   * Command: open-loop voltage from a live supplier. Use for manual joystick-driven
   * testing (e.g. right trigger = up, left trigger = down). Positive = up.
   * Exits voltage mode and stops when the command ends.
   */
  public Command runVoltage(DoubleSupplier voltsSupplier) {
    return Commands.run(() -> {
      enterVoltageMode();
      io.setVoltage(voltsSupplier.getAsDouble());
    }, this).finallyDo(this::exitVoltageMode).withName(settings.name + " ManualVoltage");
  }

  /** Current carriage height (from the AdvantageKit inputs — replay-safe). */
  public Distance getHeight() {
    return Meters.of(positionNative());
  }

  /** Current carriage velocity (from the AdvantageKit inputs — replay-safe). */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(velocityNative());
  }

  /** Trigger: true while the carriage is within {@code tolerance} of {@code height}. */
  public Trigger isAtHeight(Distance height, Distance tolerance) {
    return new Trigger(
        () -> Math.abs(positionNative() - height.in(Meters)) <= tolerance.in(Meters));
  }

  /** The settings this mechanism was built with (start positions, limits, ...). */
  public Settings getSettings() {
    return settings;
  }

  /**
   * The live robot-frame pose where a child mechanism mounts: the carriage at its
   * current height, oriented like the elevator. Pass {@code elevator::attachmentPose}
   * as another mechanism's {@code visualParent} to ride the carriage in the 3D view.
   */
  public Pose3d attachmentPose() {
    Pose3d mount = MechanismVisuals3d.resolveMount(
        settings.visualPose3d, settings.visualParent, settings.visualParentOffset);
    return new Pose3d(
        MechanismVisuals3d.planarPoint(mount, 0, positionNative()), mount.getRotation());
  }
}
