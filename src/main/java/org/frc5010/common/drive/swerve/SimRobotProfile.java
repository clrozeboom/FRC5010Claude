package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;

/**
 * Default {@link RobotProfile} for library development and CI testing.
 *
 * <p>Builds a 24×24-inch simulated robot with IronMaple physics, spawning at
 * (1.5, 2.0) on the Blue alliance side so the dyn4j solver starts collision-free.
 * No real CAN IDs are required.
 *
 * <p>For a real robot, implement a subclass of {@link RobotProfile} with the actual
 * hardware constants and a mode-branching {@link #createDrive()} — see the Javadoc on
 * {@link RobotProfile} for the recommended template.
 */
public class SimRobotProfile extends RobotProfile {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private static final Pose2d BLUE_START = new Pose2d(1.5, 2.0, new Rotation2d());

  @Override
  public AkitSwerveDrive createDrive() {
    return SwerveFactory.build(CONSTANTS, BLUE_START);
  }

  @Override
  public Pose2d getBlueAllianceStartPose() {
    return BLUE_START;
  }
}
