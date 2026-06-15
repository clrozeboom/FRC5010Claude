package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Top-level entry point wired by {@link Robot}.
 *
 * <p>Delegates all robot behaviour to a {@link org.frc5010.common.profiles.SwerveRobotContainer}
 * subclass — here the lesson's own {@link LessonRobot} (arm + intake roller + shooter + status
 * LEDs), built entirely in the {@code frc.robot} package on top of the common library. Teams
 * swap this for their own subclass when wiring their real robot. The library's reusable example
 * subsystems and mechanisms still live under {@code org.frc5010.examples}; common library code
 * lives in {@link org.frc5010.common.profiles.SwerveRobotContainer}.
 */
public class RobotContainer {

  private final LessonRobot robot;

  public RobotContainer() {
    robot = new LessonRobot();
  }

  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void resetToAllianceStart() {
    robot.resetToAllianceStart();
  }
}
