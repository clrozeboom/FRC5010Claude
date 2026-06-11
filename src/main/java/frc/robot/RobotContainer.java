package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.example.ExampleRobot;

/**
 * Top-level entry point wired by {@link Robot}.
 *
 * <p>Delegates all robot behaviour to {@link ExampleRobot}, which extends
 * {@link org.frc5010.common.profiles.SwerveRobotContainer} and owns drive wiring,
 * vision, and the {@link frc.robot.example.DemoIntake} demo. Team-specific code lives in
 * {@link ExampleRobot}; common library code lives in
 * {@link org.frc5010.common.profiles.SwerveRobotContainer}.
 */
public class RobotContainer {

  private final ExampleRobot robot;

  public RobotContainer() {
    robot = new ExampleRobot();
  }

  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void resetToAllianceStart() {
    robot.resetToAllianceStart();
  }
}
