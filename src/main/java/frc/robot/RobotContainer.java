package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.RealRobot;

/**
 * Top-level entry point wired by {@link Robot}.
 *
 * <p>Delegates all robot behaviour to {@link RealRobot}, which extends
 * {@link org.frc5010.common.profiles.SwerveRobotContainer} and owns drive wiring,
 * vision, and the {@link frc.robot.rebuilt.DemoIntake} demo. Team-specific code lives in
 * {@link RealRobot}; common library code lives in
 * {@link org.frc5010.common.profiles.SwerveRobotContainer}.
 */
public class RobotContainer {

  private final RealRobot robot;

  public RobotContainer() {
    robot = new RealRobot();
  }

  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void resetToAllianceStart() {
    robot.resetToAllianceStart();
  }
}
