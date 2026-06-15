package org.frc5010.examples;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import org.frc5010.common.sim.SimRobotState;
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
 * <p>Extends {@link SimRobotState}, which owns the {@link IntakeSimulation} instance,
 * the {@code periodic()} cleanup loop, generic {@code extendCommand()} /
 * {@code retractCommand()}, and automatic web UI state binding. This class adds
 * game-specific logic: Fuel piece type and dimensions, ballistic projectile firing,
 * and hub-scoring callbacks.
 *
 * <p>All Fuel pieces (grounded + in-flight) are published each cycle to the
 * {@code "Fuel"} {@link FieldObject2d} on the provided {@link Field2d} so they
 * appear in Glass/AdvantageScope. The same positions are served by the web
 * server's {@code /api/gamepieces} endpoint.
 *
 * <p>Controls are exposed as WPILib {@link Command}s so callers bind them to
 * {@link edu.wpi.first.wpilibj2.command.button.Trigger}s in the standard WPILib pattern:
 *
 * <ul>
 *   <li>{@code extendCommand()} — extends the intake and starts collecting Fuel (from base)
 *   <li>{@code retractCommand()} — retracts the intake and stops collecting (from base)
 *   <li>{@link #fireCommand()} — fires one held Fuel piece toward the current target
 * </ul>
 *
 * <p>When inside the alliance zone (X &lt; 3.952 m for Blue, X &gt; 12.589 m for
 * Red) the shot is aimed at the alliance hub at a steep 65° arc. Outside the
 * zone the shot lobs toward the zone-accumulation centre at 40° so fuel builds
 * up where the robot can collect it later.  In both cases launch speed is
 * computed from the ballistic formula v = √(g·d² / (2·cos²θ·(d·tanθ − Δh))).
 */
public class DemoIntake extends SimRobotState {

  // ---- geometry ----
  private static final double BUMPER_HALF_M = Units.inchesToMeters(15);

  /** Fuel the robot is preloaded with at startup (a typical match preload). */
  private static final int PRELOADED_FUEL = 8;

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

  // Written on robot thread (projectile hit callback), read by HTTP thread via getScoredCount().
  private final AtomicInteger scoredFuelCount = new AtomicInteger(0);
  // Nullable — non-null only when Field2d is provided (sim mode with NT publishing).
  private final FieldObject2d fuelField;

  /**
   * Creates a demo intake attached to the given IronMaple drive-train simulation.
   * Web UI state binding happens automatically via {@link org.frc5010.common.sim.WebControl#getInstance()}.
   *
   * @param driveSim     physics drive-train (from {@code drive.getDriveTrainSimulation().get()})
   * @param poseSupplier supplier of the current robot pose (used for projectile launch origin)
   * @param field2d      the Field2d published to NetworkTables, used to render Fuel pieces in
   *                     Glass and AdvantageScope; pass {@code null} to skip NT publishing
   */
  public DemoIntake(
      AbstractDriveTrainSimulation driveSim,
      Supplier<Pose2d> poseSupplier,
      Field2d field2d) {
    super(
        IntakeSimulation.OverTheBumperIntake("Fuel", driveSim, Inches.of(24), Inches.of(12), IntakeSide.FRONT, 50),
        poseSupplier,
        BUMPER_HALF_M + Units.inchesToMeters(6),
        field2d != null ? field2d.getObject("Intake") : null);
    this.fuelField = field2d != null ? field2d.getObject("Fuel") : null;
    // Start the match preloaded with Fuel (well under the 50-piece capacity).
    intakeSimulation.setGamePiecesCount(PRELOADED_FUEL);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (fuelField == null) return;

    List<Pose2d> poses = new ArrayList<>();
    try {
      for (var piece : SimulatedArena.getInstance().gamePiecesOnField()) {
        if ("Fuel".equals(piece.getType())) poses.add(piece.getPoseOnField());
      }
      for (var proj : SimulatedArena.getInstance().gamePieceLaunched()) {
        if ("Fuel".equals(proj.getType())) {
          var p3 = proj.getPose3d();
          poses.add(new Pose2d(p3.getX(), p3.getY(), p3.getRotation().toRotation2d()));
        }
      }
    } catch (Exception ignored) {}
    fuelField.setPoses(poses);
  }

  @Override
  protected int getScoredCount() { return scoredFuelCount.get(); }

  // ---- public accessors (aliases onto SimRobotState state) ----

  /** Number of Fuel pieces currently held. */
  public int getHeldFuel()          { return getHeldPieces(); }
  /** Whether the intake is currently extended. */
  public boolean isIntakeExtended() { return isExtended(); }

  /**
   * Whether the given robot pose is inside the current alliance's zone
   * (X &lt; 3.952 m for Blue, X &gt; 12.589 m for Red; Blue when alliance is unknown).
   * Drives shot aiming here and the side-segment laser colour in {@link DemoLeds}.
   */
  public static boolean isInAllianceZone(Pose2d pose) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue
        ? pose.getX() < ZONE_DEPTH_M
        : pose.getX() > FIELD_WIDTH_M - ZONE_DEPTH_M;
  }

  // ---- game-specific command ----

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

    boolean inZone = isInAllianceZone(pose);

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
      final Translation3d capturedHub = hubTarget;
      projectile
          .withTargetPosition(() -> capturedHub)
          .withTargetTolerance(new Translation3d(RebuiltHub.GoalRadius, RebuiltHub.GoalRadius, 0.4))
          .withHitTargetCallBack(() -> {
            scored.incrementAndGet();
            // Exit toward field center (common area), with ±30° spread
            double baseAngle = isBlue ? 0.0 : Math.PI;
            double exitAngle = baseAngle + (Math.random() - 0.5) * Math.toRadians(60);
            RebuiltFuelOnFly exitPiece = new RebuiltFuelOnFly(
                new Translation2d(capturedHub.getX(), capturedHub.getY()),
                new Translation2d(),
                new ChassisSpeeds(),
                new Rotation2d(exitAngle),
                Meters.of(capturedHub.getZ() / 2.0),
                MetersPerSecond.of(2.5),
                Degrees.of(0.0));
            exitPiece.enableBecomesGamePieceOnFieldAfterTouchGround();
            exitPiece.launch();
            SimulatedArena.getInstance().addGamePieceProjectile(exitPiece);
          });
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
