package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;

/**
 * Demo intake and scoring simulation for the 2026 Rebuilt game.
 * Not a real mechanism — for interactive demonstration only.
 *
 * <p>Uses IronMaple's {@link IntakeSimulation} for physics-driven game-piece collection.
 * Controls are exposed as WPILib {@link Command}s so callers bind them to
 * {@link edu.wpi.first.wpilibj2.command.button.Trigger}s in the standard WPILib pattern:
 *
 * <ul>
 *   <li>{@link #extendCommand()} — extends the intake and starts collecting Fuel
 *   <li>{@link #retractCommand()} — retracts the intake and stops collecting
 *   <li>{@link #fireCommand()} — fires one held Fuel piece toward the current target
 * </ul>
 *
 * <p>When inside the alliance zone (X &lt; 3.952 m for Blue, X &gt; 12.589 m for
 * Red) the shot is aimed at the alliance hub at a steep 65° arc. Outside the
 * zone the shot lobs toward the zone-accumulation centre at 40° so fuel builds
 * up where the robot can collect it later.  In both cases launch speed is
 * computed from the ballistic formula v = √(g·d² / (2·cos²θ·(d·tanθ − Δh))).
 */
public class DemoIntake extends SubsystemBase {

  // ---- geometry ----
  private static final double BUMPER_HALF_M = Units.inchesToMeters(15);

  // ---- projectile launch parameters ----
  private static final double LAUNCH_HEIGHT_M         = 0.4;
  private static final double HUB_SHOT_ELEVATION_DEG  = 65.0;
  private static final double ZONE_SHOT_ELEVATION_DEG = 40.0;
  private static final double GRAVITY_MPS2             = 9.80665;
  private static final double MIN_LAUNCH_SPEED_MPS    = 2.0;
  private static final double MAX_LAUNCH_SPEED_MPS    = 20.0;

  // ---- hub 3D positions (decoded from RebuiltHub static initialiser) ----
  private static final Translation3d BLUE_HUB_3D = new Translation3d(4.5974, 4.034536, 1.5748);
  private static final Translation3d RED_HUB_3D  = new Translation3d(11.938, 4.034536, 1.5748);

  // ---- alliance zone boundaries and accumulation targets ----
  private static final double ZONE_DEPTH_M  = 3.952;
  private static final double FIELD_WIDTH_M = 16.540988;
  private static final Translation3d BLUE_ZONE_TARGET =
      new Translation3d(ZONE_DEPTH_M / 2, 4.035, 0.1);
  private static final Translation3d RED_ZONE_TARGET =
      new Translation3d(FIELD_WIDTH_M - ZONE_DEPTH_M / 2, 4.035, 0.1);

  private final Supplier<Pose2d> poseSupplier;
  private final IntakeSimulation intakeSimulation;

  // Volatile so HTTP thread (web /api/state) sees fresh values without locking.
  private volatile boolean intakeExtended = false;
  // Scored count written on robot thread (in projectile hit callback), read by HTTP thread.
  private final AtomicInteger scoredFuelCount = new AtomicInteger(0);

  /**
   * Creates a demo intake attached to the given IronMaple drive-train simulation.
   *
   * @param driveSim     physics drive-train (from {@code drive.getDriveTrainSimulation().get()})
   * @param poseSupplier supplier of the current robot pose (used for projectile launch origin)
   * @param webControl   web UI facade for state binding, or {@code null} when not in web UI mode
   */
  public DemoIntake(
      AbstractDriveTrainSimulation driveSim,
      Supplier<Pose2d> poseSupplier,
      org.frc5010.common.sim.WebControl webControl) {
    this.poseSupplier = poseSupplier;
    intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        "Fuel", driveSim, Inches.of(24), Inches.of(12), IntakeSide.FRONT, 5);
    intakeSimulation.register();
    if (webControl != null) {
      webControl.bindDemoState(this::getHeldFuel, this::isIntakeExtended, this::getScoredCount);
    }
  }

  @Override
  public void periodic() {
    intakeSimulation.removeObtainedGamePieces(SimulatedArena.getInstance());
  }

  // ---- state accessors (thread-safe reads for HTTP thread) ----

  /** Number of Fuel pieces currently held. */
  public int getHeldFuel()          { return intakeSimulation.getGamePiecesAmount(); }
  /** Whether the intake is currently extended. */
  public boolean isIntakeExtended() { return intakeExtended; }
  /** Fuel pieces scored in the hub since startup. */
  public int getScoredCount()       { return scoredFuelCount.get(); }

  // ---- command factories ----

  /** Extends the intake and starts collecting game pieces. */
  public Command extendCommand() {
    return Commands.runOnce(() -> {
      intakeSimulation.startIntake();
      intakeExtended = true;
    }, this).withName("ExtendIntake");
  }

  /** Retracts the intake and stops collecting. */
  public Command retractCommand() {
    return Commands.runOnce(() -> {
      intakeSimulation.stopIntake();
      intakeExtended = false;
    }, this).withName("RetractIntake");
  }

  /** Fires one held Fuel piece using ballistic physics. No-op if nothing is held. */
  public Command fireCommand() {
    return Commands.runOnce(() -> fireFuel(poseSupplier.get()), this).withName("FireFuel");
  }

  // ---- private helpers ----

  private void fireFuel(Pose2d pose) {
    if (intakeSimulation.getGamePiecesAmount() <= 0) return;
    intakeSimulation.obtainGamePieceFromIntake();

    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Translation3d hubTarget = isBlue ? BLUE_HUB_3D : RED_HUB_3D;

    double robotX = pose.getX();
    boolean inZone = isBlue
        ? robotX < ZONE_DEPTH_M
        : robotX > FIELD_WIDTH_M - ZONE_DEPTH_M;

    double theta = pose.getRotation().getRadians();
    Translation2d launchPos = new Translation2d(
        pose.getX() + BUMPER_HALF_M * Math.cos(theta),
        pose.getY() + BUMPER_HALF_M * Math.sin(theta));

    Translation3d target3d  = inZone ? hubTarget : (isBlue ? BLUE_ZONE_TARGET : RED_ZONE_TARGET);
    double elevationDeg     = inZone ? HUB_SHOT_ELEVATION_DEG : ZONE_SHOT_ELEVATION_DEG;

    double dx    = target3d.getX() - launchPos.getX();
    double dy    = target3d.getY() - launchPos.getY();
    double hDist = Math.sqrt(dx * dx + dy * dy);
    double dz    = target3d.getZ() - LAUNCH_HEIGHT_M;
    double launchSpeedMps = computeLaunchSpeed(hDist, dz, elevationDeg);

    RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
        launchPos,
        new Translation2d(),
        new ChassisSpeeds(),
        new Rotation2d(dx, dy),
        Meters.of(LAUNCH_HEIGHT_M),
        MetersPerSecond.of(launchSpeedMps),
        Degrees.of(elevationDeg));

    if (inZone) {
      final AtomicInteger scored = scoredFuelCount;
      projectile
          .withTargetPosition(() -> hubTarget)
          .withTargetTolerance(new Translation3d(RebuiltHub.GoalRadius, RebuiltHub.GoalRadius, 0.4))
          .withHitTargetCallBack(scored::incrementAndGet);
    }
    projectile.enableBecomesGamePieceOnFieldAfterTouchGround();
    projectile.launch();
    SimulatedArena.getInstance().addGamePieceProjectile(projectile);
  }

  /**
   * Returns the launch speed (m/s) to reach a target at horizontal distance {@code d} and
   * height delta {@code dz}, fired at {@code elevationDeg}.
   * Clamped to [{@link #MIN_LAUNCH_SPEED_MPS}, {@link #MAX_LAUNCH_SPEED_MPS}].
   *
   * <p>v² = g·d² / (2·cos²θ·(d·tanθ − dz))
   */
  private static double computeLaunchSpeed(double d, double dz, double elevationDeg) {
    double theta = Math.toRadians(elevationDeg);
    double cosT  = Math.cos(theta);
    double denom = 2.0 * cosT * cosT * (d * Math.tan(theta) - dz);
    if (denom <= 0 || d < 0.01) return MIN_LAUNCH_SPEED_MPS;
    double v2 = GRAVITY_MPS2 * d * d / denom;
    return Math.max(MIN_LAUNCH_SPEED_MPS, Math.min(MAX_LAUNCH_SPEED_MPS, Math.sqrt(v2)));
  }
}
