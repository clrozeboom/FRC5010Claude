package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.RebuiltRobot;

/**
 * Top-level entry point wired by {@link Robot}.
 *
 * <p>Delegates all robot behaviour to a {@link org.frc5010.common.profiles.SwerveRobotContainer}
 * subclass — the ported 2026 "Rebuilt" competition robot ({@link RebuiltRobot}). Teams swap
 * this for their own subclass when wiring their real robot. Reusable example subsystems and
 * mechanisms live under {@code org.frc5010.examples}; common library code lives in
 * {@link org.frc5010.common.profiles.SwerveRobotContainer}.
 */
public class RobotContainer {

  private final RebuiltRobot robot;

  public RobotContainer() {
    robot = new RebuiltRobot();
  }

  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void resetToAllianceStart() {
    robot.resetToAllianceStart();
  }
}
