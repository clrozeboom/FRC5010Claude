package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Custom IronMaple arena for the 2019 FRC game <em>DESTINATION: DEEP SPACE</em>.
 *
 * <p>Built by hand from the game manual because IronMaple does not ship an
 * {@code Arena2019DeepSpace} field map. Installed via
 * {@link SimulatedArena#overrideInstance(SimulatedArena)} in
 * {@link SwerveFactory#build(SwerveConstants, Pose2d)} before the first
 * {@link SimulatedArena#getInstance()} call. All geometry is in the WPILib field frame
 * (origin bottom-left from Blue's POV, +X toward Red wall, +Y to Blue's left).
 *
 * <h3>Collision obstacles</h3>
 * <ul>
 *   <li>Field perimeter walls (54&nbsp;ft &times; 27&nbsp;ft).</li>
 *   <li>Four <b>Rockets</b> — one against each side guardrail, &plusmn;8&nbsp;ft from field centre
 *       along X.</li>
 *   <li>Two <b>Cargo Ships</b> — straddling the field midline, sterns 9&nbsp;in from centre.</li>
 * </ul>
 *
 * <h3>Intentionally NOT collided</h3>
 * The two <b>HAB platforms</b> against the alliance walls are drawn in the web view but are
 * <em>not</em> collision bodies: they are climbable ramps in the real game, and the existing
 * vision integration test spawns the robot at (2,&nbsp;4) — inside the Blue HAB footprint —
 * so colliding it would break that test.
 */
public class Arena2019DeepSpace extends SimulatedArena {

  /** Field length (X, toward Red wall): 54 ft. */
  public static final double FIELD_LENGTH = Inches.of(648).in(Meters); // 16.4592 m
  /** Field width (Y, to Blue's left): 27 ft. */
  public static final double FIELD_WIDTH = Inches.of(324).in(Meters);  //  8.2296 m

  public Arena2019DeepSpace() {
    super(new FieldMap2019());
  }

  /** Obstacle list for the 2019 field — perimeter walls, four rockets, two cargo ships. */
  public static final class FieldMap2019 extends SimulatedArena.FieldMap {
    FieldMap2019() {
      final double L = FIELD_LENGTH;
      final double W = FIELD_WIDTH;

      // ---- perimeter walls ----
      addBorderLine(new Translation2d(0, 0), new Translation2d(L, 0));
      addBorderLine(new Translation2d(L, 0), new Translation2d(L, W));
      addBorderLine(new Translation2d(L, W), new Translation2d(0, W));
      addBorderLine(new Translation2d(0, W), new Translation2d(0, 0));

      // ---- Rockets (one per side guardrail, +/-8 ft from field centre on X) ----
      // Footprint ~33" wide (X) x 19-5/8" deep (Y), front face 27.5" off the guardrail.
      final double rocketW = Inches.of(33).in(Meters);     // along X
      final double rocketD = Inches.of(19.625).in(Meters); // along Y
      final double rocketCenterOffX = Inches.of(96).in(Meters); // 8 ft
      final double rocketCenterY = Inches.of(27.5).in(Meters) - rocketD / 2.0; // 0.449 m off wall
      final double rxBlue = L / 2.0 - rocketCenterOffX; //  5.7912
      final double rxRed = L / 2.0 + rocketCenterOffX;  // 10.6680

      addRectangularObstacle(rocketW, rocketD, new Pose2d(rxBlue, rocketCenterY, Rotation2d.kZero));
      addRectangularObstacle(rocketW, rocketD, new Pose2d(rxBlue, W - rocketCenterY, Rotation2d.kZero));
      addRectangularObstacle(rocketW, rocketD, new Pose2d(rxRed, rocketCenterY, Rotation2d.kZero));
      addRectangularObstacle(rocketW, rocketD, new Pose2d(rxRed, W - rocketCenterY, Rotation2d.kZero));

      // ---- Cargo Ships (straddle the midline, sterns 9" from field centre) ----
      final double shipLen = Inches.of(95.75).in(Meters); // along X, 7'11-3/4"
      final double shipWid = Inches.of(55.75).in(Meters); // along Y, 4'7-3/4"
      final double sternOff = Inches.of(9).in(Meters);    // stern 9" from centre
      final double shipBlueX = L / 2.0 - sternOff - shipLen / 2.0; // 6.785
      final double shipRedX = L / 2.0 + sternOff + shipLen / 2.0;  // 9.674
      addRectangularObstacle(shipLen, shipWid, new Pose2d(shipBlueX, W / 2.0, Rotation2d.kZero));
      addRectangularObstacle(shipLen, shipWid, new Pose2d(shipRedX, W / 2.0, Rotation2d.kZero));
    }
  }

  /** Spawn the curated 2019 starting pieces. Called on construction and on "Reset Field". */
  @Override
  public void placeGamePiecesOnField() {
    GamePieceSpawner2019.spawnInitialPieces(this);
  }
}
