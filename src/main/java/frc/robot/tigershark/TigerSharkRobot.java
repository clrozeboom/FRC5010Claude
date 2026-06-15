package frc.robot.tigershark;

import org.frc5010.common.profiles.SwerveRobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;

public class TigerSharkRobot extends SwerveRobotContainer {
    protected TigerSharkElevator elevator;
    protected TigerSharkLeds leds;

    public TigerSharkRobot() {
        super(SwerveRobotContainer.selectProfile("frc.robot.tigershark.TigerSharkRobotProfile"));
    }

    @Override
    public void configureBindings() {
        super.configureBindings();
        elevator = new TigerSharkElevator();
        leds = new TigerSharkLeds(elevator::isMoving, elevator::isNearTop, elevator::isNearBottom);
        registerMechanism(elevator::close);
        registerMechanism(leds::close);

        controller.x().onTrue(elevator.goToHeight(TigerSharkElevator.SCORING_HEIGHT));
        controller.x().onFalse(elevator.goToHeight(elevator.getSettings().startingHeight));
        controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));
        controller.y().onFalse(elevator.goToHeight(elevator.getSettings().startingHeight));
    }

    @Override
    public void buildAutos() {
        addAuto("None", Commands.none());
    }
}
