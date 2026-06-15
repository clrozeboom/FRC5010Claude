package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FuelHandler;
import frc.robot.subsystems.StatusLeds;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * The lesson robot: a swerve drive plus a {@link FuelHandler} (arm + intake roller + shooter)
 * and {@link StatusLeds}. This is <em>your</em> robot — it lives in the {@code frc.robot}
 * package and builds only on library classes, not on the library's demo code.
 *
 * <p>Extending {@link SwerveRobotContainer} gives you keyboard/controller swerve drive,
 * alliance-aware starting pose, and the autonomous chooser for free. You add your subsystems
 * and button bindings in {@link #configureBindings()}.
 *
 * <p><b>Controls (simulation):</b>
 * <ul>
 *   <li>Left bumper — deploy the intake (arm out, roller spinning, collecting Fuel)</li>
 *   <li>Right bumper — retract the intake</li>
 *   <li>A — score: spin the shooter up and launch one Fuel in the direction you're facing</li>
 * </ul>
 *
 * <p>The drivetrain numbers come from {@code ExampleRobotProfile} (the placeholder team
 * profile); when wiring a real robot you would supply your own profile per
 * {@code docs/student-setup.md}.
 *
 * <p>Note: {@link SwerveRobotContainer}'s constructor calls {@link #configureBindings()} before
 * this class's constructor body runs, so the subsystems are created <em>inside</em>
 * {@code configureBindings()} (fields assigned after {@code super(...)} would still be
 * {@code null} when it fires).
 */
public class LessonRobot extends SwerveRobotContainer {

  /** PWM header the status LED strip is plugged into. */
  private static final int LED_PWM_PORT = 9;

  private FuelHandler fuelHandler;
  private StatusLeds statusLeds;

  public LessonRobot() {
    super(SwerveRobotContainer.selectProfile("org.frc5010.examples.ExampleRobotProfile"));
  }

  /** The Fuel handler, present only in simulation. Exposed for tests. */
  public FuelHandler getFuelHandler() {
    return fuelHandler;
  }

  /** The status LEDs, present only in simulation. Exposed for tests. */
  public StatusLeds getStatusLeds() {
    return statusLeds;
  }

  @Override
  protected void configureBindings() {
    super.configureBindings(); // keyboard / web swerve drive

    // The Fuel handler is a simulation game-piece subsystem, so it only exists in sim. On a
    // real robot you would create your real mechanisms here, outside this guard.
    if (!RobotBase.isSimulation()) {
      return;
    }

    drive.getDriveTrainSimulation().ifPresent(driveSim -> {
      fuelHandler = new FuelHandler(driveSim, drive::getPose, drive.getField2d());
      statusLeds = new StatusLeds(
          LED_PWM_PORT,
          fuelHandler::isIntakeExtended,
          fuelHandler::isRollerSpinning,
          fuelHandler::isShooterSpinning,
          fuelHandler::isShooterAtSpeed);

      // Register cleanup so tests (and shutdown) free the motors and the LED PWM port.
      registerMechanism(fuelHandler::close);
      registerMechanism(statusLeds::close);

      controller.leftBumper().onTrue(fuelHandler.deployCommand());
      controller.rightBumper().onTrue(fuelHandler.retractCommand());
      controller.a().onTrue(
          Commands.runOnce(() -> {
            if (fuelHandler.getHeldFuel() > 0) {
              statusLeds.notifyShot();
            }
          }).andThen(fuelHandler.scoreCommand()));
    });
  }

  @Override
  protected void buildAutos() {
    addAuto("None", Commands.none());
    // A tiny auto: score the preloaded Fuel. (Drive autos can be added later with BLine.)
    if (fuelHandler != null) {
      addAuto("Score Preload",
          Commands.repeatingSequence(fuelHandler.scoreCommand(), Commands.waitSeconds(0.3))
              .until(() -> fuelHandler.getHeldFuel() == 0)
              .withTimeout(8.0));
    }
  }
}
