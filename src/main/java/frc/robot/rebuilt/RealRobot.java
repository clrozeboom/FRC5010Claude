package frc.robot.rebuilt;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * Team-specific robot container for the 2026 Rebuilt season — extends
 * {@link SwerveRobotContainer} with competition hardware constants ({@link RealRobotProfile})
 * and the {@link DemoIntake} demo.
 *
 * <p>Auto registration is handled in {@link #buildAutos()}, which the base class schedules
 * automatically on the first scheduler tick (disabled, before the first autonomous mode).
 * This defers construction until all subsystems are fully initialised so that game-piece
 * subsystems like {@link DemoIntake} are available when autos are built.
 *
 * <p>Note: {@link SwerveRobotContainer}'s constructor calls {@link #configureBindings()} before
 * this class's constructor body runs. Fields initialised after {@code super(...)} would be
 * {@code null} when {@code configureBindings()} fires — initialise such fields inside
 * {@code configureBindings()} itself (as {@code demoIntake} is here).
 */
public class RealRobot extends SwerveRobotContainer {

  private DemoIntake demoIntake;

  public RealRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.rebuilt.RealRobotProfile"));
  }

  @Override
  protected void buildAutos() {
    addAuto("None", Commands.none());
    addAuto("BLine: Example Score (JSON)", scoreAfter(AutoRoutines.exampleScore(drive)));
    addAuto("BLine: Example Score (code)", scoreAfter(AutoRoutines.exampleScoreInCode(drive)));
    if (demoIntake != null) {
      addAuto("BLine: Pickup + Score", AutoRoutines.pickupAndScore(drive, demoIntake));
    }
  }

  /**
   * Appends a "fire all held Fuel at the Hub" step to a drive-only auto so the robot scores
   * its preload. No-op (returns the auto unchanged) when the demo intake is absent (non-sim).
   */
  private Command scoreAfter(Command driveAuto) {
    return demoIntake == null
        ? driveAuto
        : Commands.sequence(driveAuto, AutoRoutines.fireAllFuel(demoIntake));
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
}
