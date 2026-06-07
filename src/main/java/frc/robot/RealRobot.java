package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * Team-specific robot container — extends {@link SwerveRobotContainer} with
 * competition hardware constants ({@link RealRobotProfile}) and the {@link DemoIntake} demo.
 *
 * <p>Profile selection is handled by {@link SwerveRobotContainer#selectProfile(String)}: pass
 * the fully-qualified class name of the desired profile. To use a custom profile that extends
 * {@link RealRobotProfile}, just change the class name string here.
 *
 * <p>Note: {@link SwerveRobotContainer}'s constructor calls {@link #configureBindings()} before
 * this class's constructor body runs. Fields initialised after {@code super(...)} would be
 * {@code null} when {@code configureBindings()} fires — initialise such fields inside
 * {@code configureBindings()} itself (as {@code demoIntake} is here).
 */
public class RealRobot extends SwerveRobotContainer {

  private DemoIntake demoIntake;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RealRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.RealRobotProfile"));

    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("BLine: Example Score (JSON)", AutoRoutines.exampleScore(drive));
    autoChooser.addOption("BLine: Example Score (code)", AutoRoutines.exampleScoreInCode(drive));
    if (demoIntake != null) {
      autoChooser.addOption("BLine: Pickup + Score", AutoRoutines.pickupAndScore(drive, demoIntake));
    }
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    controller.y().onTrue(TeleopRoutines.driveToHub(drive));

    if (!RobotBase.isSimulation()) return;

    drive.getDriveTrainSimulation().ifPresent(driveSim -> {
      demoIntake = new DemoIntake(driveSim, drive::getPose, drive.getField2d());

      controller.leftBumper().onTrue(demoIntake.extendCommand());
      controller.rightBumper().onTrue(demoIntake.retractCommand());
      controller.a().onTrue(demoIntake.fireCommand());
    });
  }

  @Override
  public Command getAutonomousCommand() {
    Command visual = super.getAutonomousCommand();
    return visual != null ? visual : autoChooser.getSelected();
  }
}
