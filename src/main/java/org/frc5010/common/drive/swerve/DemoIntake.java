package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;

/**
 * Demo intake and scoring simulation for the 2026 Rebuilt game.
 * Not a real mechanism — for interactive demonstration only.
 *
 * <p>Web controller button mapping:
 * <ul>
 *   <li>LB (idx 4) — click to latch intake extended
 *   <li>RB (idx 5) — click to retract intake
 *   <li>A  (idx 0) — fire one Fuel; speed and loft angle are computed
 *                    automatically to reach the current target
 * </ul>
 *
 * <p>When inside the alliance zone (X &lt; 3.952 m for Blue, X &gt; 12.589 m for
 * Red) the shot is aimed at the alliance hub at a steep 65° arc. Outside the
 * zone the shot lobs toward the zone-accumulation centre at 40° so fuel builds
 * up where the robot can collect it later.  In both cases launch speed is
 * computed from the ballistic formula v = √(g·d² / (2·cos²θ·(d·tanθ − Δh))).
 *
 * <p>Must be called from the robot thread (inside the drive default command).
 */
public class DemoIntake {

  // ---- geometry ----
  private static final double BUMPER_HALF_M      = Units.inchesToMeters(15);
  private static final double INTAKE_EXTENSION_M = Units.inchesToMeters(12);
  /** Distance from robot centre to intake tip while extended (= bumper edge + 12"). */
  public  static final double INTAKE_REACH_M     = BUMPER_HALF_M + INTAKE_EXTENSION_M;
  private static final double INTAKE_RADIUS_M    = 0.15;

  // ---- projectile launch parameters ----
  private static final double LAUNCH_HEIGHT_M         = 0.4;    // shooter height above ground
  private static final double HUB_SHOT_ELEVATION_DEG  = 65.0;   // steep arc into elevated hub
  private static final double ZONE_SHOT_ELEVATION_DEG = 40.0;   // flatter lob toward zone centre
  private static final double GRAVITY_MPS2             = 9.80665;
  private static final double MIN_LAUNCH_SPEED_MPS    = 2.0;
  private static final double MAX_LAUNCH_SPEED_MPS    = 20.0;

  // ---- hub 3D positions (decoded from RebuiltHub static initialiser) ----
  // blueHubPose / redHubPose: Translation3d(x, y, 1.5748) — Z = 62" scoring height
  private static final Translation3d BLUE_HUB_3D = new Translation3d(4.5974, 4.034536, 1.5748);
  private static final Translation3d RED_HUB_3D  = new Translation3d(11.938, 4.034536, 1.5748);

  // ---- alliance zone boundaries and accumulation targets ----
  // Zone extends from each alliance wall to X = 3.952 m inward (trench/hub wall line).
  private static final double ZONE_DEPTH_M  = 3.952;
  private static final double FIELD_WIDTH_M = 16.540988;
  // When out of zone, aim for the centre of the alliance's zone so fuel accumulates there.
  private static final Translation3d BLUE_ZONE_TARGET = new Translation3d(ZONE_DEPTH_M / 2, 4.035, 0.1);
  private static final Translation3d RED_ZONE_TARGET  = new Translation3d(FIELD_WIDTH_M - ZONE_DEPTH_M / 2, 4.035, 0.1);

  private final WebDriveController wdc;
  private boolean intakeExtended = false;
  private int heldFuel   = 0;
  // Scored count written on robot thread (in projectile hit callback), read by HTTP thread.
  private final AtomicInteger scoredFuelCount = new AtomicInteger(0);
  private final boolean[] prevBtn = new boolean[6];

  public DemoIntake(WebDriveController wdc) {
    this.wdc = wdc;
  }

  /**
   * Run once per 20 ms robot cycle (inside the drive default command, while enabled).
   * @param robotPose current robot pose from the drive subsystem
   */
  public void periodic(Pose2d robotPose) {
    // ---- intake: LB click latches extended; RB click retracts ----
    if (wdc.getButton(4).getAsBoolean() && !prevBtn[4]) intakeExtended = true;
    if (wdc.getButton(5).getAsBoolean() && !prevBtn[5]) intakeExtended = false;

    if (intakeExtended) collectNearbyFuel(robotPose);

    // ---- A button fires (rising-edge); B/X/Y unused ----
    boolean heldA = wdc.getButton(0).getAsBoolean();
    if (heldA && !prevBtn[0]) fireFuel(robotPose);
    for (int i = 0; i < 6; i++) prevBtn[i] = wdc.getButton(i).getAsBoolean();

    // Push state to WebDriveController atomics so /api/state includes them.
    wdc.setHeldFuel(heldFuel);
    wdc.setIntakeExtended(intakeExtended);
    wdc.setScored(scoredFuelCount.get());
  }

  // ---- private helpers ----

  private void collectNearbyFuel(Pose2d pose) {
    double theta = pose.getRotation().getRadians();
    Translation2d tip = new Translation2d(
        pose.getX() + INTAKE_REACH_M * Math.cos(theta),
        pose.getY() + INTAKE_REACH_M * Math.sin(theta));

    SimulatedArena arena = SimulatedArena.getInstance();
    List<GamePieceOnFieldSimulation> toRemove = new ArrayList<>();
    for (GamePieceOnFieldSimulation piece : arena.gamePiecesOnField()) {
      if (!"Fuel".equals(piece.getType())) continue;
      if (tip.getDistance(piece.getPoseOnField().getTranslation()) < INTAKE_RADIUS_M) {
        toRemove.add(piece);
      }
    }
    for (GamePieceOnFieldSimulation piece : toRemove) {
      if (arena.removeGamePiece(piece)) heldFuel++;
    }
  }

  private void fireFuel(Pose2d pose) {
    if (heldFuel <= 0) return;
    heldFuel--;

    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Translation3d hubTarget = isBlue ? BLUE_HUB_3D : RED_HUB_3D;

    double robotX = pose.getX();
    boolean inZone = isBlue
        ? robotX < ZONE_DEPTH_M
        : robotX > FIELD_WIDTH_M - ZONE_DEPTH_M;

    // Launch from the robot's front bumper edge.
    double theta = pose.getRotation().getRadians();
    Translation2d launchPos = new Translation2d(
        pose.getX() + BUMPER_HALF_M * Math.cos(theta),
        pose.getY() + BUMPER_HALF_M * Math.sin(theta));

    Translation3d target3d  = inZone ? hubTarget : (isBlue ? BLUE_ZONE_TARGET : RED_ZONE_TARGET);
    double elevationDeg     = inZone ? HUB_SHOT_ELEVATION_DEG : ZONE_SHOT_ELEVATION_DEG;

    double dx   = target3d.getX() - launchPos.getX();
    double dy   = target3d.getY() - launchPos.getY();
    double hDist = Math.sqrt(dx * dx + dy * dy);
    double dz   = target3d.getZ() - LAUNCH_HEIGHT_M;
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
   * Returns the launch speed (m/s) required to reach a target at horizontal distance
   * {@code d} metres and height delta {@code dz} metres, when fired at {@code elevationDeg}.
   * Clamped to [{@link #MIN_LAUNCH_SPEED_MPS}, {@link #MAX_LAUNCH_SPEED_MPS}].
   *
   * <p>Derivation: d = v·cosθ·t ; dz = v·sinθ·t − ½g·t²
   * → v² = g·d² / (2·cos²θ·(d·tanθ − dz))
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
