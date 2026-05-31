package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Seeds {@link Arena2019DeepSpace} with a curated set of Cargo and Hatch Panel pieces.
 *
 * <p>Rather than flooding the field, this places ~20 Cargo balls in two neutral-zone bands
 * (clear of the cargo-ship footprints) plus 8 Hatch Panels in the loading-station corners near
 * the alliance walls. Every position is kept clear of the rocket and cargo-ship obstacle
 * footprints, the field edges, and the existing test spawn points (2,&nbsp;2), (2,&nbsp;4),
 * and (5,&nbsp;4) so the physics/vision tests stay green.
 *
 * <p>Called from {@link Arena2019DeepSpace#placeGamePiecesOnField()} (on construction and on the
 * Glass "Reset Field" widget) and from {@link SwerveFactory} right after the arena is installed.
 */
public final class GamePieceSpawner2019 {

  private GamePieceSpawner2019() {}

  // Cargo (orange ball) {x, y} positions in metres. Two horizontal bands either side of the
  // cargo-ship Y band (3.41-4.82), spanning the centre columns. All clear of rockets/ships.
  private static final double[][] CARGO_POSITIONS = {
    // ---- lower neutral band (Y below the cargo ships) ----
    { 6.50, 1.80 }, { 7.30, 1.80 }, { 8.23, 1.80 }, { 9.10, 1.80 }, { 9.90, 1.80 },
    { 6.50, 2.60 }, { 7.30, 2.60 }, { 8.23, 2.60 }, { 9.10, 2.60 }, { 9.90, 2.60 },
    // ---- upper neutral band (Y above the cargo ships) ----
    { 6.50, 5.60 }, { 7.30, 5.60 }, { 8.23, 5.60 }, { 9.10, 5.60 }, { 9.90, 5.60 },
    { 6.50, 6.40 }, { 7.30, 6.40 }, { 8.23, 6.40 }, { 9.10, 6.40 }, { 9.90, 6.40 },
  };

  // Hatch Panel (grey disc) {x, y} positions: loading-station corners near the alliance walls.
  private static final double[][] HATCH_POSITIONS = {
    // ---- Blue wall corners ----
    { 0.90, 0.80 }, { 0.90, 1.40 }, { 0.90, 7.00 }, { 0.90, 7.60 },
    // ---- Red wall corners ----
    { 15.56, 0.80 }, { 15.56, 1.40 }, { 15.56, 7.00 }, { 15.56, 7.60 },
  };

  /**
   * Clears the arena and spawns the curated Cargo + Hatch set. No-op when the active arena is
   * not {@link Arena2019DeepSpace} (so it stays harmless if wiring changes).
   */
  public static void spawnInitialPieces(SimulatedArena arena) {
    if (!(arena instanceof Arena2019DeepSpace)) return;

    arena.clearGamePieces();
    for (double[] p : CARGO_POSITIONS) {
      arena.addGamePiece(new CargoOnField(new Translation2d(p[0], p[1])));
    }
    for (double[] p : HATCH_POSITIONS) {
      arena.addGamePiece(new HatchPanelOnField(new Translation2d(p[0], p[1])));
    }
    System.out.println("[GamePieceSpawner2019] Spawned " + CARGO_POSITIONS.length
        + " Cargo + " + HATCH_POSITIONS.length + " Hatch pieces on Arena2019DeepSpace.");
  }
}
