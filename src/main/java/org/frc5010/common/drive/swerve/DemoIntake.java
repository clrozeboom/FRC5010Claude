package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Demo intake and scoring simulation for the 2019 <em>DESTINATION: DEEP SPACE</em> game.
 * Not a real mechanism — for interactive demonstration only.
 *
 * <p>Web controller button mapping:
 * <ul>
 *   <li>LB (idx 4) — click to latch intake extended (picks up nearby Cargo <em>and</em> Hatch)
 *   <li>RB (idx 5) — click to retract intake
 *   <li>A  (idx 0) — score one held Cargo if the robot is next to a Rocket or Cargo Ship;
 *                    otherwise drop it back on the field at the intake tip
 *   <li>B  (idx 1) — score one held Hatch Panel if next to a structure; otherwise drop it
 * </ul>
 *
 * <p>Both piece types are picked up by the same intake while extended. Deep Space places game
 * pieces rather than launching them, so there is no projectile physics here — scoring is decided
 * purely by proximity to a scoring structure.
 *
 * <p>Must be called from the robot thread (inside the drive default command).
 */
public class DemoIntake {

  // ---- geometry ----
  private static final double BUMPER_HALF_M      = Units.inchesToMeters(15);
  private static final double INTAKE_EXTENSION_M = Units.inchesToMeters(12);
  /** Distance from robot centre to intake tip while extended (= bumper edge + 12"). */
  public  static final double INTAKE_REACH_M     = BUMPER_HALF_M + INTAKE_EXTENSION_M;
  private static final double INTAKE_RADIUS_M    = 0.30;

  /** Robot centre must be within this distance of a scoring structure for A to score. */
  private static final double SCORE_RADIUS_M     = 1.7;

  // ---- scoring structures (centres, from Arena2019DeepSpace) ----
  private static final Translation2d[] SCORE_TARGETS = {
    // four rockets
    new Translation2d(5.7912, 0.449), new Translation2d(5.7912, 7.781),
    new Translation2d(10.668, 0.449), new Translation2d(10.668, 7.781),
    // two cargo ships
    new Translation2d(6.785, 4.1148), new Translation2d(9.674, 4.1148),
  };

  private final WebDriveController wdc;
  private boolean intakeExtended = false;
  private int heldCargo   = 0;
  private int heldHatch   = 0;
  private int scoredCargo = 0;
  private int scoredHatch = 0;
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

    if (intakeExtended) collectNearbyPieces(robotPose);

    // ---- A scores/drops a Cargo; B scores/drops a Hatch (both rising-edge) ----
    if (wdc.getButton(0).getAsBoolean() && !prevBtn[0]) scoreOrDropCargo(robotPose);
    if (wdc.getButton(1).getAsBoolean() && !prevBtn[1]) scoreOrDropHatch(robotPose);
    for (int i = 0; i < 6; i++) prevBtn[i] = wdc.getButton(i).getAsBoolean();

    // Push state to WebDriveController atomics so /api/state includes them.
    wdc.setHeldCargo(heldCargo);
    wdc.setHeldHatch(heldHatch);
    wdc.setIntakeExtended(intakeExtended);
    wdc.setScoredCargo(scoredCargo);
    wdc.setScoredHatch(scoredHatch);
  }

  // ---- private helpers ----

  private void collectNearbyPieces(Pose2d pose) {
    Translation2d tip = intakeTip(pose);

    SimulatedArena arena = SimulatedArena.getInstance();
    List<GamePieceOnFieldSimulation> toRemove = new ArrayList<>();
    for (GamePieceOnFieldSimulation piece : arena.gamePiecesOnField()) {
      String type = piece.getType();
      if (!"Cargo".equals(type) && !"Hatch".equals(type)) continue;
      if (tip.getDistance(piece.getPoseOnField().getTranslation()) < INTAKE_RADIUS_M) {
        toRemove.add(piece);
      }
    }
    for (GamePieceOnFieldSimulation piece : toRemove) {
      if (!arena.removeGamePiece(piece)) continue;
      if ("Cargo".equals(piece.getType())) heldCargo++; else heldHatch++;
    }
  }

  private void scoreOrDropCargo(Pose2d pose) {
    if (heldCargo <= 0) return;
    heldCargo--;
    if (nearStructure(pose)) {
      scoredCargo++;
    } else {
      SimulatedArena.getInstance().addGamePiece(new CargoOnField(intakeTip(pose)));
    }
  }

  private void scoreOrDropHatch(Pose2d pose) {
    if (heldHatch <= 0) return;
    heldHatch--;
    if (nearStructure(pose)) {
      scoredHatch++;
    } else {
      SimulatedArena.getInstance().addGamePiece(new HatchPanelOnField(intakeTip(pose)));
    }
  }

  /** Field-frame position of the intake tip (bumper edge + extension) ahead of the robot. */
  private static Translation2d intakeTip(Pose2d pose) {
    double theta = pose.getRotation().getRadians();
    return new Translation2d(
        pose.getX() + INTAKE_REACH_M * Math.cos(theta),
        pose.getY() + INTAKE_REACH_M * Math.sin(theta));
  }

  /** True when the robot centre is within {@link #SCORE_RADIUS_M} of any scoring structure. */
  private static boolean nearStructure(Pose2d pose) {
    Translation2d center = pose.getTranslation();
    for (Translation2d target : SCORE_TARGETS) {
      if (center.getDistance(target) <= SCORE_RADIUS_M) return true;
    }
    return false;
  }
}
