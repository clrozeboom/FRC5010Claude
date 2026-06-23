package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.frc5010.common.mechanisms.MechanismVisuals3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.frc5010.common.mechanisms.Arm;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Flywheel;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.subsystems.ShotCalculator.ShootingParameters;
import org.littletonrobotics.junction.Logger;

/**
 * Launcher subsystem — ported from the 2026 "Rebuilt" robot.
 *
 * <p>Three mechanisms: a dual-KrakenX60 {@link Flywheel} (CAN 16/17, 18:1), a hood
 * {@link Arm} (KrakenX44 CAN 19, ≈30.76:1, limits 12.723°–45.723°), and a
 * {@link SmartTurretController} turret (KrakenX44 CAN 18, 30:1, CANivore). Aiming comes from
 * {@link ShotCalculator}.
 *
 * <p>The turret is the ported torque-current-FOC {@link SmartTurretController} — a 200 Hz
 * Notifier loop with SEEKING/TRACKING hysteresis and velocity feedforward — for fast tracking
 * of a moving shot. PREP/LOW/PRESET feed it {@code setTarget(angle, velocity, accel)} each
 * 20 ms cycle; HAMMERTIME/IDLE park it forward.
 *
 * <p>The {@link LauncherState} machine (initial {@link LauncherState#HAMMERTIME}) selects
 * what the three mechanisms track each cycle. When the intake hopper blocks the turret the
 * launcher is forced to HAMMERTIME; firing is signalled via {@link #isReadyToFire()}, which
 * the container couples to the indexer FEED state.
 */
public class RebuiltLauncher extends SubsystemBase implements AutoCloseable {

  /**
   * Launcher state. The trench-duck states ({@code AUTO_HAMMERTIME}/{@code ESCAPE_HAMMERTIME})
   * from the source are intentionally omitted — the robot falls back to {@code LOW_SPEED} when
   * not shooting instead of ducking under trenches.
   */
  public enum LauncherState {
    IDLE,
    LOW_SPEED,
    PREP,
    HAMMERTIME,
    PRESET
  }

  /** A named preset shot: fixed hood angle + flywheel speed; turret still aims at the hub. */
  public record Preset(Angle hoodAngle, AngularVelocity flywheelSpeed, Angle turretAngle) {}

  // Tolerances (Constants.Launcher).
  private static final Angle HOOD_TOL = Degrees.of(Constants.Launcher.HOOD_ANGLE_TOLERANCE_DEGREES);
  private static final Angle TURRET_TOL =
      Degrees.of(Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES);
  private static final AngularVelocity FLYWHEEL_TOL =
      RPM.of(Constants.Launcher.SHOOTER_TOLERANCE_RPM);

  // Robot → turret offset (−4.856, 4.863) in.
  private static final Translation2d TURRET_OFFSET =
      new Translation2d(Units.inchesToMeters(-4.856), Units.inchesToMeters(4.863));

  private final Flywheel flywheel;
  private final Arm hood;
  private final SmartTurretController turret;
  private final ShotCalculator shotCalculator = new ShotCalculator();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldVelocitySupplier;

  private LauncherState state = LauncherState.HAMMERTIME;
  private Preset activePreset = null;
  private BooleanSupplier turretBlocked = () -> false;

  private ShootingParameters latestParams;

  public RebuiltLauncher(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldVelocitySupplier) {
    this.poseSupplier = poseSupplier;
    this.fieldVelocitySupplier = fieldVelocitySupplier;

    // Turret first: hood and flywheel use it as a visual parent so their 3D poses track
    // the live turret heading (both mount at the turret tip).
    turret = new SmartTurretController(turretConfig());
    hood = new Arm(hoodSettings(turret));
    flywheel = new Flywheel(flywheelSettings(turret));
    turret.start(); // 200 Hz control Notifier

    setDefaultCommand(Commands.run(this::applyState, this).withName("Launcher/StateMachine"));
  }

  // ── mechanism configuration ────────────────────────────────────────────────

  private static Flywheel.Settings flywheelSettings(SmartTurretController turret) {
    Flywheel.Settings s = new Flywheel.Settings();
    s.name = "Flywheel";
    s.controlStyle = ControlStyle.PROFILED_PID;
    s.canId = 16;
    s.followerCanId = 17;
    s.followerOpposed = true;
    s.motorModel = DCMotor.getKrakenX60(2);
    // 3:1 reduction gives wheel free speed ≈ 33 rot/s ≈ 209 rad/s, covering the shot
    // table's 88–173 rad/s range. The source robot uses 18:1 at the motor but the table
    // values are referenced to the wheel after the final belt stage (≈ 6:1 net output).
    s.gearReductionStages = new double[] {3.0};
    s.diameter = Inches.of(3.95); // radius 1.975 in
    s.mass = Kilograms.of(2.31); // 5.1 lb
    s.kP = 2.0; // sim
    s.kV = 0.36; // sim ≈ 12 ÷ free-speed-rot/s for 3:1 gearing
    s.statorCurrentLimit = Amps.of(60);
    // Mount at the turret tip — disc appears at the shooter, oriented in the turret's
    // forward-vertical plane (correct for a flywheel spinning around a horizontal axis).
    s.visualPose3d = new Pose3d(0, 0, 0, Rotation3d.kZero);
    s.visualParent = turret::attachmentPose;
    return s;
  }

  private static Arm.Settings hoodSettings(SmartTurretController turret) {
    Arm.Settings s = new Arm.Settings();
    s.name = "Hood";
    s.controlStyle = ControlStyle.PROFILED_PID;
    s.canId = 19;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {1015.0 / 33.0}; // ≈ 30.76:1
    s.length = Inches.of(9.466);
    s.minAngle = Degrees.of(Constants.Launcher.HOOD_CORRECTED_START_ANGLE_DEGREES);
    s.maxAngle = Degrees.of(Constants.Launcher.HOOD_CORRECTED_END_ANGLE_DEGREES);
    s.startingAngle = Degrees.of(Constants.Launcher.HOOD_CORRECTED_START_ANGLE_DEGREES);
    s.maxVelocity = DegreesPerSecond.of(1080);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(5000);
    s.kP = 40.0; // sim
    s.kV = 3.7; // ≈ 12 V ÷ free-speed-rot/s (KrakenX60 @ ≈30.76:1) — MotionMagic needs kV
    s.kG = Volts.of(0.3);
    s.statorCurrentLimit = Amps.of(60);
    // Mount at the turret tip, rotated 180° in yaw so the hood arm extends backward
    // (opposite the turret indicator), matching the physical shooter orientation.
    s.visualPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI));
    s.visualParent = turret::attachmentPose;
    return s;
  }

  /** Turret config (CAN 18 on CANivore, 30:1) for the torque-current-FOC controller. */
  private static SmartTurretConfig turretConfig() {
    SmartTurretConfig c = new SmartTurretConfig();
    c.canId = 18;
    c.canBus = "canivore";
    c.gearRatio = 30.0;
    c.motorModel = DCMotor.getKrakenX60(1);
    c.moiKgM2 = 0.05;
    // Physical turret pivot position on the robot (from CAD).
    c.visualPose3d = new Pose3d(
        Units.inchesToMeters(-4.856), Units.inchesToMeters(4.863),
        Units.inchesToMeters(14.723), MechanismVisuals3d.YAW_PLANE);
    c.visualArmLengthM = Units.inchesToMeters(7);
    return c; // gains/limits default to the ported source values
  }

  // ── target ─────────────────────────────────────────────────────────────────

  /** Alliance-aware hub top-centre aim point (2D). */
  private Translation2d hubTarget() {
    boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    return red
        ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
        : FieldConstants.Hub.topCenterPoint.toTranslation2d();
  }

  private ShootingParameters solve() {
    shotCalculator.clearShootingParameters();
    return shotCalculator.getParameters(
        TURRET_OFFSET,
        Rotation2d.kZero,
        poseSupplier.get(),
        fieldVelocitySupplier.get(),
        hubTarget());
  }

  // ── state machine ──────────────────────────────────────────────────────────

  private void applyState() {
    // Intake interference: the hopper arm blocks the turret while retracting/retracted.
    if (turretBlocked.getAsBoolean() && state != LauncherState.HAMMERTIME) {
      state = LauncherState.HAMMERTIME;
    }

    latestParams = solve();

    switch (state) {
      case PREP:
        // Full track: turret aims with velocity feedforward (fast moving-target tracking).
        aimTurret(latestParams.turretAngle().getMeasure(), latestParams.turretVelocityRadPerSec());
        hood.track(Degrees.of(latestParams.hoodAngleDegrees()));
        flywheel.track(RadiansPerSecond.of(latestParams.flywheelSpeed()));
        break;
      case LOW_SPEED:
        // Track turret + flywheel, but hold the hood at its low preset (ready to aim).
        aimTurret(latestParams.turretAngle().getMeasure(), latestParams.turretVelocityRadPerSec());
        hood.track(Constants.Launcher.LOW_HOOD_ANGLE);
        flywheel.track(RadiansPerSecond.of(latestParams.flywheelSpeed()));
        break;
      case PRESET:
        Preset p = activePreset != null ? activePreset : towerPreset();
        aimTurret(latestParams.turretAngle().getMeasure(), latestParams.turretVelocityRadPerSec());
        hood.track(p.hoodAngle());
        flywheel.track(p.flywheelSpeed());
        break;
      case IDLE:
        stowTurret();
        hood.track(Constants.Launcher.LOW_HOOD_ANGLE);
        flywheel.track(RPM.of(0));
        break;
      case HAMMERTIME:
      default:
        // Safe stow clear of intake interference: turret forward, hood low, flywheel idle.
        stowTurret();
        hood.track(Constants.Launcher.LOW_HOOD_ANGLE);
        flywheel.track(Constants.Launcher.LOW_FLYWHEEL_RPM);
        break;
    }
  }

  /** Aim the turret at {@code angle} with a velocity feedforward (rad/s) for moving targets. */
  private void aimTurret(Angle angle, double velocityRadPerSec) {
    turret.setTarget(angle, velocityRadPerSec, 0.0);
  }

  /** Park the turret forward with no feedforward. */
  private void stowTurret() {
    turret.setTarget(Constants.Launcher.TURRET_FORWARD, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    // Default commands don't run while disabled, so the turret target stops updating — neutralize
    // its 200 Hz controller here (periodic() always runs) or it would keep driving the Talon.
    if (DriverStation.isDisabled()) {
      turret.disable();
    }
    turret.updateVisualization();
    Logger.recordOutput("Launcher/State", state.name());
    Logger.recordOutput("Launcher/AtGoal", isAtGoal());
    Logger.recordOutput("Launcher/TurretState", turret.getState().name());
    Logger.recordOutput("Launcher/ReadyToFire", isReadyToFire());
    if (latestParams != null) {
      Logger.recordOutput("Launcher/ShotValid", latestParams.isValid());
      Logger.recordOutput("Launcher/DistanceToTarget", latestParams.distanceToVirtualTargetMeters());
    }
  }

  // ── readiness ──────────────────────────────────────────────────────────────

  /** Whether all three mechanisms are within tolerance of the current shot solution. */
  public boolean isAtGoal() {
    if (latestParams == null) return false;
    boolean turretOk = turret.isAtTarget(TURRET_TOL.in(edu.wpi.first.units.Units.Rotations));
    double hoodTarget =
        state == LauncherState.LOW_SPEED
            ? Constants.Launcher.LOW_HOOD_ANGLE.in(Degrees)
            : latestParams.hoodAngleDegrees();
    boolean hoodOk = Math.abs(hood.getAngle().in(Degrees) - hoodTarget) <= HOOD_TOL.in(Degrees);
    boolean flywheelOk =
        flywheel.isAtSpeed(RadiansPerSecond.of(latestParams.flywheelSpeed()), FLYWHEEL_TOL).getAsBoolean();
    return turretOk && hoodOk && flywheelOk;
  }

  /** True when in PREP, the shot is geometrically valid, and the mechanisms are at goal. */
  public boolean isReadyToFire() {
    return state == LauncherState.PREP && latestParams != null && latestParams.isValid() && isAtGoal();
  }

  /** True when the flywheel is at/above its goal — gates the indexer CHURN. */
  public boolean isFlywheelReadyForChurn() {
    return latestParams != null
        && flywheel.getSpeed().in(RadiansPerSecond) >= latestParams.flywheelSpeed() - 5;
  }

  public boolean isPrepping() {
    return state == LauncherState.PREP;
  }

  public LauncherState getState() {
    return state;
  }

  // ── wiring ─────────────────────────────────────────────────────────────────

  public void setState(LauncherState newState) {
    state = newState;
  }

  /** Wires the intake-interference gate (forces HAMMERTIME while the hopper blocks the turret). */
  public void setTurretBlocked(BooleanSupplier blocked) {
    this.turretBlocked = blocked;
  }

  // ── presets ────────────────────────────────────────────────────────────────

  public static Preset towerPreset() {
    return new Preset(
        Constants.Launcher.TOWER_HOOD_ANGLE, Constants.Launcher.TOWER_FLYWHEEL_RPM, Degrees.of(0));
  }

  public static Preset hubPreset() {
    return new Preset(
        Constants.Launcher.HUB_HOOD_ANGLE, Constants.Launcher.HUB_FLYWHEEL_RPM, Degrees.of(0));
  }

  public static Preset forwardPreset() {
    return new Preset(
        Constants.Launcher.FWD_HOOD_ANGLE, Constants.Launcher.FWD_FLYWHEEL_RPM,
        Constants.Launcher.TURRET_FORWARD);
  }

  // ── command factories ──────────────────────────────────────────────────────

  public Command prepCommand() {
    return Commands.runOnce(() -> setState(LauncherState.PREP)).withName("Launcher/Prep");
  }

  public Command lowSpeedCommand() {
    return Commands.runOnce(() -> setState(LauncherState.LOW_SPEED)).withName("Launcher/LowSpeed");
  }

  public Command idleCommand() {
    return Commands.runOnce(() -> setState(LauncherState.IDLE)).withName("Launcher/Idle");
  }

  public Command hammertimeCommand() {
    return Commands.runOnce(() -> setState(LauncherState.HAMMERTIME)).withName("Launcher/Hammertime");
  }

  public Command presetCommand(Preset preset) {
    return Commands.runOnce(
            () -> {
              activePreset = preset;
              setState(LauncherState.PRESET);
            })
        .withName("Launcher/Preset");
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Arm getHood() {
    return hood;
  }

  public SmartTurretController getTurret() {
    return turret;
  }

  @Override
  public void close() {
    flywheel.close();
    hood.close();
    turret.close();
  }
}
