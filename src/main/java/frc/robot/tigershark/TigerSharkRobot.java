package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Meters;

import org.frc5010.common.profiles.SwerveRobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Team-side container for TigerShark — wires the profile (drive + vision) plus the
 * elevator and LED strip into one cohesive robot. Pattern mirrors {@code ExampleRobot}.
 *
 * <p>Subsystems are constructed inside {@link #configureBindings()} because
 * {@link SwerveRobotContainer}'s constructor calls it before this subclass's
 * constructor body runs — fields initialised after {@code super(...)} would be
 * {@code null} at the moment {@code configureBindings()} fires.
 */
public class TigerSharkRobot extends SwerveRobotContainer {

  /** roboRIO PWM header the LED strip is plugged into. */
  private static final int LED_PWM_PORT = 9;

  private TigerSharkElevator elevator;
  private TigerSharkLeds leds;

  public TigerSharkRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.tigershark.TigerSharkRobotProfile"));
  }

  @Override
  protected void buildAutos() {
    addAuto("None", Commands.none());
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    elevator = new TigerSharkElevator();
    registerMechanism(elevator::close);

    // Three preset heights — low / scoring / max
    controller.a().onTrue(elevator.goToHeight(Meters.of(0.0)));
    controller.b().onTrue(elevator.goToHeight(TigerSharkElevator.SCORING_HEIGHT));
    controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));

    leds = new TigerSharkLeds(LED_PWM_PORT, elevator::isMoving);
    registerMechanism(leds::close);
  }
}
