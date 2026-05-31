package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/**
 * Seeds the 2026 Rebuilt arena with a curated set of Fuel game pieces.
 *
 * <p>The default {@code Arena2026Rebuilt.placeGamePiecesOnField()} spawns ≥ 360 pieces
 * (a dense 12×30 center grid plus two depot grids), which is far more physics
 * load than needed for interactive simulation. This spawner clears the default
 * pieces and places 40 Fuel discs in a layout that mirrors the real field:
 * <ul>
 *   <li>a dense center-field rectangle (30 pieces, ~x 7.4–9.1, y 1.9–6.2), and
 *   <li>two small depot clusters near the driver-station walls in opposite
 *       corners (5 near the Blue wall, 5 near the Red wall),
 * </ul>
 * while staying clear of hub footprints, trench bars, tower walls, and field edges.
 *
 * <p>Call {@link #spawnInitialFuel()} once after
 * {@link SimulatedArena#addDriveTrainSimulation} from
 * {@link SwerveFactory#build(SwerveConstants, edu.wpi.first.math.geometry.Pose2d)}.
 * It is a no-op on arenas other than {@code Arena2026Rebuilt}.
 *
 * <p>Note: if the user clicks "Reset Field" in the Glass DriverStation NT widget,
 * {@code Arena2026Rebuilt.resetFieldForAuto()} will clear our pieces and call
 * {@code placeGamePiecesOnField()} — restoring the default 360-piece grid.
 * That is expected Glass behaviour and does not require any workaround here.
 */
public final class GamePieceSpawner {

  private GamePieceSpawner() {}

  // 40 curated {x, y} positions in metres (WPILib field frame: X toward Red wall,
  // Y toward Blue driver's left). Mirrors the real field: a dense center-field
  // rectangle plus two small depot clusters near the driver-station walls.
  // All positions stay clear of hub footprints, trench bars, towers, and edges.
  private static final double[][] FUEL_POSITIONS = {
    // ---- center-field rectangle: 5 columns × 6 rows, centred on (8.27, 4.035) ----
    { 7.43, 1.91 }, { 7.85, 1.91 }, { 8.27, 1.91 }, { 8.69, 1.91 }, { 9.11, 1.91 },
    { 7.43, 2.76 }, { 7.85, 2.76 }, { 8.27, 2.76 }, { 8.69, 2.76 }, { 9.11, 2.76 },
    { 7.43, 3.61 }, { 7.85, 3.61 }, { 8.27, 3.61 }, { 8.69, 3.61 }, { 9.11, 3.61 },
    { 7.43, 4.46 }, { 7.85, 4.46 }, { 8.27, 4.46 }, { 8.69, 4.46 }, { 9.11, 4.46 },
    { 7.43, 5.31 }, { 7.85, 5.31 }, { 8.27, 5.31 }, { 8.69, 5.31 }, { 9.11, 5.31 },
    { 7.43, 6.16 }, { 7.85, 6.16 }, { 8.27, 6.16 }, { 8.69, 6.16 }, { 9.11, 6.16 },
    // ---- Blue-wall depot cluster (x≈0.7–1.0, high-y corner; clear of tower @4.04) ----
    { 0.70, 5.30 }, { 1.00, 5.65 }, { 0.70, 6.00 }, { 1.00, 6.35 }, { 0.70, 6.70 },
    // ---- Red-wall depot cluster (x≈15.5–16.0, low-y corner; clear of tower @4.32) ----
    { 15.80, 1.50 }, { 15.50, 1.85 }, { 16.00, 2.20 }, { 15.80, 2.55 }, { 16.00, 1.30 },
  };

  /**
   * Clears all existing game pieces from the arena and spawns the curated
   * set of 25 Fuel pieces. No-op when not running {@code Arena2026Rebuilt}.
   */
  public static void spawnInitialFuel() {
    SimulatedArena arena = SimulatedArena.getInstance();
    if (!arena.getClass().getName().contains("Arena2026Rebuilt")) return;

    arena.clearGamePieces();
    for (double[] pos : FUEL_POSITIONS) {
      arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(pos[0], pos[1])));
    }
    System.out.println("[GamePieceSpawner] Spawned " + FUEL_POSITIONS.length
        + " Fuel pieces on Arena2026Rebuilt.");
  }
}
