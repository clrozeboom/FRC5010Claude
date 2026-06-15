package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.examples.ExampleRobot;

/**
 * Top-level entry point wired by {@link Robot}.
 *
 * <p>Delegates all robot behaviour to a {@link org.frc5010.common.profiles.SwerveRobotContainer}
 * subclass — by default the demo {@link ExampleRobot} so simulation works out of the box.
 * Teams swap this for their own subclass when wiring their real robot. Reusable example
 * subsystems and mechanisms live under {@code org.frc5010.examples}; common library code
 * lives in {@link org.frc5010.common.profiles.SwerveRobotContainer}.
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
