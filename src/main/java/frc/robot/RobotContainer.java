package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc5010.common.drive.swerve.RobotProfile;
import org.frc5010.common.drive.swerve.SimRobotProfile;
import org.frc5010.common.drive.swerve.SwerveRobotContainer;

/**
 * Top-level robot container — wired by {@link Robot} on startup.
 *
 * <p>Selects the robot profile based on context:
 * <ul>
 *   <li>{@code RobotBase.isReal()} → {@link RealRobotProfile} (hardware IO)</li>
 *   <li>Simulation → {@link SimRobotProfile} by default (test robot, no real CAN IDs)</li>
 *   <li>To simulate the real robot's physics, add
 *       {@code -PsimulateRealRobot} to the Gradle command and uncomment the relevant
 *       line in {@link #selectProfile()}.</li>
 * </ul>
 *
 * <p>All wiring (keyboard drive, alliance reset, visual test) is inherited from
 * {@link SwerveRobotContainer}. Override {@code configureBindings()} or
 * {@code getAutonomousCommand()} here to add robot-specific bindings or auto routines.
 */
public class RobotContainer extends SwerveRobotContainer {

  public RobotContainer() {
    super(selectProfile());
  }

  private static RobotProfile selectProfile() {
    if (RobotBase.isReal()) return new RealRobotProfile();
    // Automated testing agents pass -PtestSim (./gradlew simulateJava -PtestSim) to use the
    // lightweight test robot (no real CAN IDs required).
    if (Boolean.getBoolean("testSim")) return new SimRobotProfile();
    // Default for VSCode "Simulate Robot Code": real robot parameters in IronMaple physics.
    return new RealRobotProfile();
  }
}
