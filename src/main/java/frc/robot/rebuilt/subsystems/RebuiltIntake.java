package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.mechanisms.Arm;
import org.frc5010.common.mechanisms.ControlStyle;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem — ported from the 2026 "Rebuilt" robot.
 *
 * <p>Two spintake roller groups ({@code inner}, {@code outer}) plus a {@link HopperArm}
 * (KrakenX44 ×2, CAN 15 lead / 14 follower, 24:1, PROFILED MotionMagic) that deploys
 * (0° = floor hard stop) and retracts (120°) the intake. Behaviour is driven by an
 * {@link IntakeState} machine modelled with WPILib {@link Trigger}s rather than the
 * source robot's config-driven {@code StateMachine}.
 *
 * <p>The hopper has no absolute encoder on the real robot; it homes against the deployed
 * hard stop. The homing duty values, stall thresholds, and periodic auto-rezero logic are
 * ported from {@code frc.robot.rebuilt.subsystems.intake} (see {@link Constants.Intake}).
 *
 * <p>Spintake rollers are modelled as logged duty state (the source treats them as simple
 * velocity flywheels); actual Fuel collection is performed by the sim game-piece layer,
 * which the container starts/stops from {@link #isCollecting()}.
 */
public class RebuiltIntake extends SubsystemBase implements AutoCloseable {

  /** Intake state, ported from {@code IntakeCommands.IntakeState}. */
  public enum IntakeState {
    UNKNOWN,
    RETRACTED,
    RETRACTING,
    DEPLOYING,
    INTAKING,
    DEPLOYED,
    ANGLED
  }

  private final HopperArm hopper;
  private final Supplier<Pose2d> poseSupplier;

  private IntakeState requested = IntakeState.RETRACTED;
  private IntakeState current = IntakeState.RETRACTED;
  private boolean hopperZeroed = false;

  private double outerSpintakeSpeed = 0.0;
  private double innerSpintakeSpeed = 0.0;
  /** Last hopper goal scheduled, degrees — avoids re-scheduling the goal command each cycle. */
  private double lastHopperTargetDeg = Double.NaN;
  /** FPGA timestamp when the current DEPLOYING/RETRACTING transit began. */
  private double transitionStart = 0.0;
  /** Settle time for a deploy/retract to complete if the angle target isn't reached. */
  private static final double HOPPER_SETTLE_SECONDS = 0.6;

  /** Speed used while INTAKING; overridden to INTAKE_AUTO in autonomous. */
  private DoubleSupplier intakeSpeedSupplier = () -> Constants.Intake.INTAKE_IN;

  public RebuiltIntake(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    Arm.Settings s = new Arm.Settings();
    s.name = "Hopper";
    s.controlStyle = ControlStyle.LQR;
    s.canId = 15;
    s.followerCanId = 14;
    s.followerOpposed = true; // hopper follower is inverted
    s.motorModel = DCMotor.getKrakenX60(2);
    s.gearReductionStages = new double[] {24.0};
    s.length = edu.wpi.first.units.Units.Meters.of(0.5);
    s.mass = edu.wpi.first.units.Units.Kilograms.of(2.0);
    s.minAngle = Degrees.of(-10); // headroom below the 0° deployed hard stop
    s.maxAngle = Degrees.of(130); // headroom above the 120° retracted hard stop
    s.startingAngle = Constants.Intake.HOPPER_RETRACTED_ANGLE; // starts retracted (120°)
    s.maxVelocity = DegreesPerSecond.of(360);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(720);
    s.statorCurrentLimit = Amps.of(60);
    s.kG = Volts.of(0.30);
    s.qelmsPosition = Degrees.of(5);       // loose enough for a heavy intake arm
    s.qelmsVelocity = DegreesPerSecond.of(60);
    s.relms = Volts.of(12);
    // TalonFX soft limits have headroom for homing (minAngle = -10°, maxAngle = 130°), but
    // the physics simulation must clamp at the real physical hard stops so the arm can't
    // escape through gravity when there is no motor output (e.g. while the robot is disabled).
    s.physicsMinAngle = Constants.Intake.HOPPER_DEPLOYED_ANGLE;   // 0° — floor hard stop
    s.physicsMaxAngle = Constants.Intake.HOPPER_RETRACTED_ANGLE;  // 120° — stowed hard stop
    s.clearGoalOnDisable = false;
    hopper = new HopperArm(s);

    setDefaultCommand(Commands.run(this::applyState, this).withName("Intake/StateMachine"));
  }

  // ── state machine ──────────────────────────────────────────────────────────

  /** Drives the hopper + spintakes from the current/requested state each cycle. */
  private void applyState() {
    advanceState();
    driveOutputs();
  }

  /**
   * Advances {@link #current} toward {@link #requested}, mirroring the source transitions.
   *
   * <p>The transit states ({@code DEPLOYING}/{@code RETRACTING}) complete when the hopper
   * reaches its commanded angle <em>or</em> after a settle time. The real robot detects
   * arrival from the deployed hard stop (stall current), which can't be faithfully
   * simulated; the settle timer is the robust sim/fallback equivalent.
   */
  private void advanceState() {
    switch (requested) {
      case INTAKING:
      case DEPLOYED:
        if (current != IntakeState.INTAKING && current != IntakeState.DEPLOYED) {
          if (current != IntakeState.DEPLOYING) {
            transitionStart = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
          }
          current = IntakeState.DEPLOYING;
        }
        if (current == IntakeState.DEPLOYING
            && (hopper.getAngleDegrees() <= Constants.Intake.HOPPER_ANGLE_TOLERANCE + 3
                || settleElapsed())) {
          current = requested; // INTAKING or DEPLOYED
          hopperZeroed = true;
        }
        break;
      case RETRACTED:
        if (current != IntakeState.RETRACTED) {
          if (current != IntakeState.RETRACTING) {
            transitionStart = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
          }
          current = IntakeState.RETRACTING;
        }
        if (current == IntakeState.RETRACTING
            && (hopper.getAngleDegrees()
                    >= Constants.Intake.HOPPER_RETRACTED_ANGLE.in(Degrees)
                        - Constants.Intake.HOPPER_ANGLE_TOLERANCE
                || settleElapsed())) {
          current = IntakeState.RETRACTED;
        }
        break;
      case ANGLED:
        current = IntakeState.ANGLED;
        break;
      default:
        break;
    }
  }

  /** Whether the hopper has had time to reach its commanded angle since the transit began. */
  private boolean settleElapsed() {
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - transitionStart > HOPPER_SETTLE_SECONDS;
  }

  /** Sets hopper goal + spintake speeds for the current state. */
  private void driveOutputs() {
    switch (current) {
      case DEPLOYING:
      case INTAKING:
      case DEPLOYED:
        setHopperTarget(Constants.Intake.HOPPER_DEPLOYED_ANGLE);
        if (current == IntakeState.INTAKING) {
          double speed = DriverStation.isAutonomous()
              ? Constants.Intake.INTAKE_AUTO
              : intakeSpeedSupplier.getAsDouble();
          outerSpintakeSpeed = speed;
          innerSpintakeSpeed = speed * (Constants.Intake.INTAKE_INNER_IN / Constants.Intake.INTAKE_IN);
        } else {
          outerSpintakeSpeed = 0;
          innerSpintakeSpeed = 0;
        }
        break;
      case ANGLED:
        setHopperTarget(Constants.Intake.HOPPER_ANGLED);
        outerSpintakeSpeed = 0.5;
        innerSpintakeSpeed = Constants.Intake.INTAKE_CHURN;
        break;
      case RETRACTING:
      case RETRACTED:
      default:
        setHopperTarget(Constants.Intake.HOPPER_RETRACTED_ANGLE);
        outerSpintakeSpeed = 0;
        innerSpintakeSpeed = 0;
        break;
    }
  }

  /** Commands the hopper toward {@code target} each cycle via the mechanism's track() setter. */
  private void setHopperTarget(edu.wpi.first.units.measure.Angle target) {
    lastHopperTargetDeg = target.in(Degrees);
    hopper.track(target);
  }

  @Override
  public void periodic() {
    // Auto-rezero (faithful to the source): if zeroed + on the deploy side + at a hard stop
    // between tolerance and the rezero-max angle, OR the angle dropped below the past-stop
    // threshold, re-seed the encoder to 0°.
    boolean rezero = shouldAutoRezeroAtDeployHardStop() || shouldAutoRezeroPastStop();
    if (rezero) {
      hopper.resetEncoder(Degrees.of(0));
    }
    Logger.recordOutput("Intake/AutoRezeroTriggered", rezero);
    Logger.recordOutput("Intake/StateRequested", requested.name());
    Logger.recordOutput("Intake/StateCurrent", current.name());
    Logger.recordOutput("Intake/OuterSpintake", outerSpintakeSpeed);
    Logger.recordOutput("Intake/InnerSpintake", innerSpintakeSpeed);
    Logger.recordOutput("Intake/HopperAngleDeg", hopper.getAngleDegrees());
    Logger.recordOutput("Intake/HopperZeroed", hopperZeroed);
    Logger.recordOutput("Intake/NearTrench", isNearTrench());
  }

  private boolean shouldAutoRezeroAtDeployHardStop() {
    boolean deploySide =
        requested == IntakeState.INTAKING
            || requested == IntakeState.DEPLOYED
            || current == IntakeState.DEPLOYING
            || current == IntakeState.INTAKING
            || current == IntakeState.DEPLOYED;
    double angle = hopper.getAngleDegrees();
    return hopperZeroed
        && deploySide
        && hopper.atHardStop(
            Constants.Intake.HOPPER_MOVING_VELOCITY_THRESHOLD,
            Constants.Intake.HOPPER_STALL_CURRENT_THRESHOLD)
        && angle < Constants.Intake.HOPPER_DEPLOY_STOP_REZERO_MAX_ANGLE
        && angle > Constants.Intake.HOPPER_ANGLE_TOLERANCE;
  }

  private boolean shouldAutoRezeroPastStop() {
    return hopperZeroed
        && hopper.getAngleDegrees() < Constants.Intake.HOPPER_AUTO_REZERO_THRESHOLD;
  }

  // ── requests + accessors ───────────────────────────────────────────────────

  public void setRequestedState(IntakeState state) {
    requested = state;
  }

  public IntakeState getRequestedState() {
    return requested;
  }

  public IntakeState getCurrentState() {
    return current;
  }

  public boolean isCurrent(IntakeState state) {
    return current == state;
  }

  public boolean isRequested(IntakeState state) {
    return requested == state;
  }

  /** Whether the spintakes are actively pulling in Fuel (drives the sim game-piece intake). */
  public boolean isCollecting() {
    return current == IntakeState.INTAKING && outerSpintakeSpeed > Constants.Intake.INTAKE_DEADZONE;
  }

  /** Whether the hopper is deployed clear of the floor (extended, for web UI / overlays). */
  public boolean isExtended() {
    return current == IntakeState.DEPLOYING
        || current == IntakeState.INTAKING
        || current == IntakeState.DEPLOYED
        || current == IntakeState.ANGLED;
  }

  /** Whether the hopper blocks the turret (retracting/retracted) — launcher must stow. */
  public boolean isBlockingTurret() {
    return current == IntakeState.RETRACTING || current == IntakeState.RETRACTED;
  }

  /**
   * True while deploying near a trench opening — the turret should duck under the trench.
   * Mirrors the source {@code isNearTrench()} (only meaningful while DEPLOYING).
   */
  public boolean isNearTrench() {
    if (current != IntakeState.DEPLOYING || poseSupplier == null) {
      return false;
    }
    Translation2d robot = poseSupplier.get().getTranslation();
    double half = FieldConstants.TRENCH_HALF_WIDTH.getX();
    return robot.getDistance(FieldConstants.TrenchZoneTop.nearAlliance) < half
        || robot.getDistance(FieldConstants.TrenchZoneBottom.nearAlliance) < half;
  }

  public HopperArm getHopper() {
    return hopper;
  }

  // ── command factories (bound by the container) ─────────────────────────────

  /** Request INTAKING with a driver-controlled spintake speed. */
  public Command intakeCommand(DoubleSupplier speedSupplier) {
    return Commands.runOnce(() -> {
          intakeSpeedSupplier = speedSupplier;
          setRequestedState(IntakeState.INTAKING);
        })
        .withName("Intake/Intaking");
  }

  /** Request DEPLOYED (hopper down, no spintakes). */
  public Command deployCommand() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.DEPLOYED)).withName("Intake/Deploy");
  }

  /** Request RETRACTING → RETRACTED. */
  public Command retractCommand() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.RETRACTED)).withName("Intake/Retract");
  }

  /** Request ANGLED (jam-clearing pose). */
  public Command angledCommand() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.ANGLED)).withName("Intake/Angled");
  }

  @Override
  public void close() {
    hopper.close();
  }
}
