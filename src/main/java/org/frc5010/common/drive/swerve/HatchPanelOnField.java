package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/** A 2019 Hatch Panel (19" OD disc) resting on the field. */
public class HatchPanelOnField extends GamePieceOnFieldSimulation {
  public HatchPanelOnField(Translation2d initialPosition) {
    super(GamePieces2019.HATCH, new Pose2d(initialPosition, new Rotation2d()));
  }
}
