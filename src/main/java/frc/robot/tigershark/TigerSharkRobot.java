package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.frc5010.common.input.DriveVector;
import org.frc5010.common.input.JoystickAxis;
import org.frc5010.common.profiles.SwerveRobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    // Replace the base keyboard drive (axis 2 = left trigger) with proper Xbox axes.
    // Left stick = translation, right stick X = rotation.
    JoystickAxis forward  = controller.leftY().negate().deadzone(0.05).power(2.0);
    JoystickAxis strafe   = controller.leftX().negate().deadzone(0.05).power(2.0);
    JoystickAxis rotation = controller.rightX().negate().deadzone(0.10);
    DriveVector translate = DriveVector.of(forward, strafe).unitCircle();

    drive.setDefaultCommand(
        Commands.run(
            () -> {
              double flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                  ? -1.0 : 1.0;
              Translation2d xy = translate.get();
              drive.runVelocityFieldRelative(new ChassisSpeeds(
                  flip * xy.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                  flip * xy.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                  rotation.getAsDouble() * drive.getMaxAngularSpeed().in(RadiansPerSecond)));
            },
            drive
        ).withName("XboxDrive")
    );

    elevator = new TigerSharkElevator();
    registerMechanism(elevator::close);

    // Three preset heights — low / scoring / max
    controller.a().onTrue(elevator.goToHeight(Meters.of(0.0)));
    controller.b().onTrue(elevator.goToHeight(TigerSharkElevator.SCORING_HEIGHT));
    controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));

    // TEMPORARY: Start button runs SysId limited to 80% of travel.
    // Remove once kV/kA/kG values are recorded and set in TigerSharkElevator.
    double sysIdMax = elevator.getSettings().maxHeight.in(Meters) * 0.8;
    controller.start().whileTrue(elevator.sysId(Meters.of(0), Meters.of(sysIdMax)));

    // Manual open-loop: right trigger = up, left trigger = down (6 V max).
    // Use for functional checks before running SysId.
    Trigger eitherTrigger = new Trigger(
        () -> controller.rightTrigger().getAsDouble() > 0.05
           || controller.leftTrigger().getAsDouble() > 0.05);
    eitherTrigger.whileTrue(elevator.runVoltage(
        () -> (controller.rightTrigger().getAsDouble()
             - controller.leftTrigger().getAsDouble()) * 6.0));

    leds = new TigerSharkLeds(LED_PWM_PORT, elevator::isMoving);
    registerMechanism(leds::close);
  }
}
