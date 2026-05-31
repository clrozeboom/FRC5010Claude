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
 *   <li>A  (idx 0) — fire one Fuel at power level 1 toward the nearest hub
 *   <li>B  (idx 1) — fire at power level 2
 *   <li>X  (idx 2) — fire at power level 3
 *   <li>Y  (idx 3) — fire at power level 4 (highest, longest range)
 * </ul>
 *
 * <p>Each power level maps to a 45° arc at 5×speed m/s (levels 1–4 → 5–20 m/s).
 * At 45° the maximum horizontal range is v²/g, so:
 * level 1 ≈ 2.5 m, level 2 ≈ 10 m, level 3 ≈ 23 m, level 4 ≈ 41 m.
 * Low-power shots fall short and become collectible ground pieces again;
 * high-power shots reach both hubs from anywhere on the field.
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
  private static final double SPEED_MULTIPLIER = 5.0;  // user level 1–4 → 5–20 m/s
  private static final double LAUNCH_HEIGHT_M  = 0.4;  // shooter height above ground
  private static final double ELEVATION_DEG    = 45.0; // flat-optimal arc

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

    // ---- fire buttons (rising-edge, A/B/X/Y → power levels 1/2/3/4) ----
    for (int i = 0; i < 4; i++) {
      boolean held = wdc.getButton(i).getAsBoolean();
      if (held && !prevBtn[i]) fireFuel(robotPose, i + 1);
      prevBtn[i] = held;
    }
    prevBtn[4] = wdc.getButton(4).getAsBoolean();
    prevBtn[5] = wdc.getButton(5).getAsBoolean();

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

  private void fireFuel(Pose2d pose, int powerLevel) {
    if (heldFuel <= 0) return;
    heldFuel--;

    double launchSpeedMps = powerLevel * SPEED_MULTIPLIER;

    // Determine which alliance hub to use (own alliance, not nearest).
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Translation3d hubTarget = isBlue ? BLUE_HUB_3D : RED_HUB_3D;

    // Check if the robot is inside its alliance zone (can score directly at the hub).
    double robotX = pose.getX();
    boolean inZone = isBlue
        ? robotX < ZONE_DEPTH_M
        : robotX > FIELD_WIDTH_M - ZONE_DEPTH_M;

    // Launch from the robot's front bumper edge.
    double theta = pose.getRotation().getRadians();
    Translation2d launchPos = new Translation2d(
        pose.getX() + BUMPER_HALF_M * Math.cos(theta),
        pose.getY() + BUMPER_HALF_M * Math.sin(theta));

    // Aim at the hub when in zone; otherwise aim toward the zone accumulation point.
    Translation3d target3d = inZone ? hubTarget : (isBlue ? BLUE_ZONE_TARGET : RED_ZONE_TARGET);
    double dx = target3d.getX() - launchPos.getX();
    double dy = target3d.getY() - launchPos.getY();
    Rotation2d heading = new Rotation2d(dx, dy);

    RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
        launchPos,
        new Translation2d(),
        new ChassisSpeeds(),
        heading,
        Meters.of(LAUNCH_HEIGHT_M),
        MetersPerSecond.of(launchSpeedMps),
        Degrees.of(ELEVATION_DEG));

    if (inZone) {
      // In-zone: register scoring callback — hit the hub = counted as scored.
      final AtomicInteger scored = scoredFuelCount;
      projectile
          .withTargetPosition(() -> hubTarget)
          .withTargetTolerance(new Translation3d(RebuiltHub.GoalRadius, RebuiltHub.GoalRadius, 0.4))
          .withHitTargetCallBack(scored::incrementAndGet);
    }
    // Always become a ground piece on landing so misses (and out-of-zone shots) are collectible.
    projectile.enableBecomesGamePieceOnFieldAfterTouchGround();

    projectile.launch();
    SimulatedArena.getInstance().addGamePieceProjectile(projectile);
  }
}
