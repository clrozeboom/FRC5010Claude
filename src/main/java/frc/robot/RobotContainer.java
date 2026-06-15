package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.tigershark.TigerSharkRobot;

/**
 * Top-level entry point wired by {@link Robot}.
 *
 * <p>Delegates all robot behaviour to {@link TigerSharkRobot}, which extends
 * {@link org.frc5010.common.profiles.SwerveRobotContainer} and owns drive wiring,
 * vision, and the {@link frc.robot.tigershark.TigerSharkElevator}. Team-specific code lives in
 * {@link TigerSharkRobot}; common library code lives in
 * {@link org.frc5010.common.profiles.SwerveRobotContainer}. Reusable example
 * subsystems and mechanisms live under {@code org.frc5010.examples}.
 */
public class RobotContainer {

  private final TigerSharkRobot robot;

  public RobotContainer() {
    robot = new TigerSharkRobot();
  }

  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void resetToAllianceStart() {
    robot.resetToAllianceStart();
  }
}
