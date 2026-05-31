package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/** A 2019 Cargo (13" orange ball) resting on the field. */
public class CargoOnField extends GamePieceOnFieldSimulation {
  public CargoOnField(Translation2d initialPosition) {
    super(GamePieces2019.CARGO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
