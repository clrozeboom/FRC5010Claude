package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import org.dyn4j.geometry.Circle;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;

/**
 * Shared {@link GamePieceInfo} specifications for the 2019 <em>DESTINATION: DEEP SPACE</em> game.
 *
 * <p>Two piece types, both modelled as circular collision discs (IronMaple game pieces are 2D
 * disc/box bodies):
 * <ul>
 *   <li><b>Cargo</b> — a 13&nbsp;inch diameter orange ball (radius 6.5&nbsp;in).</li>
 *   <li><b>Hatch</b> — a 19&nbsp;inch outer-diameter toroidal hatch panel (radius 9.5&nbsp;in),
 *       modelled as a thin solid disc.</li>
 * </ul>
 *
 * <p>The {@code type} strings ({@code "Cargo"}, {@code "Hatch"}) are the keys the arena and the
 * web renderer filter on — keep them in sync with {@code WebDriveController.handleGamePieces}
 * and {@code index.html}'s {@code drawGamePieces}.
 */
public final class GamePieces2019 {

  private GamePieces2019() {}

  /** Orange 13" cargo ball. type, shape, height, mass, linearDamping, angularDamping, restitution. */
  public static final GamePieceInfo CARGO = new GamePieceInfo(
      "Cargo",
      new Circle(Inches.of(6.5).in(Meters)),
      Inches.of(13),
      Pounds.of(1.0),
      1.8, 5, 0.6);

  /** Grey 19" OD hatch panel, modelled as a thin solid disc. */
  public static final GamePieceInfo HATCH = new GamePieceInfo(
      "Hatch",
      new Circle(Inches.of(9.5).in(Meters)),
      Inches.of(2),
      Pounds.of(1.2),
      2.5, 6, 0.2);
}
