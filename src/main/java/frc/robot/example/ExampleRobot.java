package frc.robot.example;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.mechanisms.ExampleArm;
import frc.robot.mechanisms.ExampleCharacterizedElevator;
import frc.robot.mechanisms.ExampleDifferentialWrist;
import frc.robot.mechanisms.ExampleDoubleJointedArm;
import frc.robot.mechanisms.ExampleElevator;
import frc.robot.mechanisms.ExampleProfiledArm;
import frc.robot.mechanisms.ExampleProfiledElevator;
import frc.robot.mechanisms.ExampleProfiledShooter;
import frc.robot.mechanisms.ExampleProfiledTurret;
import frc.robot.mechanisms.ExampleShooter;
import frc.robot.mechanisms.ExampleTurret;
import java.util.List;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * Example team robot container for the 2026 Rebuilt season — the template teams copy.
 * Extends {@link SwerveRobotContainer} with hardware constants ({@link ExampleRobotProfile}),
 * the {@link DemoIntake} demo, and the demo mechanisms.
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
public class ExampleRobot extends SwerveRobotContainer {

  /** PWM header the demo LED strip is plugged into. */
  private static final int LED_PWM_PORT = 9;

  private DemoIntake demoIntake;
  private DemoLeds demoLeds;

  /** Test hook: one representative demo mechanism, to verify the X binding end to end. */
  private static ExampleElevator demoElevator;

  /** The sim demo elevator, if demo mechanisms exist. For tests. */
  public static java.util.Optional<ExampleElevator> getDemoElevator() {
    return java.util.Optional.ofNullable(demoElevator);
  }

  public ExampleRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.example.ExampleRobotProfile"));
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
      // On a real robot the LED strip would be created unconditionally (outside the sim
      // guard); here its states come from the sim-only DemoIntake, so it lives with it.
      demoLeds = new DemoLeds(LED_PWM_PORT, drive::getPose, demoIntake::isIntakeExtended);
      registerMechanism(demoLeds::close);

      controller.leftBumper().onTrue(demoIntake.extendCommand());
      controller.rightBumper().onTrue(demoIntake.retractCommand());
      controller.a().onTrue(
          Commands.runOnce(() -> {
            if (demoIntake.getHeldFuel() > 0) demoLeds.notifyShot();
          }).alongWith(demoIntake.fireCommand()));
    });

    configureDemoMechanisms();
  }

  /**
   * Instantiates every example mechanism (sim only — CAN 21–35 don't exist on the
   * real robot; a competition robot would create just its own mechanisms, outside the
   * sim guard) and binds X to drive them all to a mid-travel point at once. Watch them
   * under SmartDashboard → {@code <name>/mechanism} or in AdvantageScope.
   */
  private void configureDemoMechanisms() {
    var elevator = new ExampleElevator();
    var arm = new ExampleArm();
    var turret = new ExampleTurret();
    var shooter = new ExampleShooter();
    var jointedArm = new ExampleDoubleJointedArm();
    var wrist = new ExampleDifferentialWrist();
    var profiledElevator = new ExampleProfiledElevator();
    var profiledArm = new ExampleProfiledArm();
    var profiledTurret = new ExampleProfiledTurret();
    var profiledShooter = new ExampleProfiledShooter();
    var characterizedElevator = new ExampleCharacterizedElevator();

    // Coupled-mechanism demo: the arm rides the elevator carriage and the shooter rides
    // the arm tip — a three-link chain. Each child's visualParent points at its parent's
    // live attachmentPose(), so in the 3D view raising the elevator lifts the whole
    // arm+shooter assembly and swinging the arm carries the shooter with it. (With a
    // parent set, visualPose3d is the offset from the parent's endpoint; zero = on it.)
    arm.getSettings().visualParent = elevator::attachmentPose;
    arm.getSettings().visualPose3d = new edu.wpi.first.math.geometry.Pose3d();
    shooter.getSettings().visualParent = arm::attachmentPose;
    shooter.getSettings().visualPose3d = new edu.wpi.first.math.geometry.Pose3d();
    // The shooter rides a short standoff past the arm tip rather than dead on it:
    // visualParentOffset is the bracket carrying it off the parent's endpoint (here 8 cm
    // further along the arm), kept separate from the shooter's own visualPose3d.
    shooter.getSettings().visualParentOffset = new edu.wpi.first.math.geometry.Transform3d(
        new edu.wpi.first.math.geometry.Translation3d(0.08, 0, 0),
        edu.wpi.first.math.geometry.Rotation3d.kZero);

    demoElevator = elevator;
    registerMechanism(() -> demoElevator = null);
    List.of((Runnable) elevator::close, arm::close, turret::close, shooter::close,
            jointedArm::close, wrist::close,
            profiledElevator::close, profiledArm::close, profiledTurret::close,
            profiledShooter::close, characterizedElevator::close)
        .forEach(SwerveRobotContainer::registerMechanism);

    controller.x().onTrue(Commands.parallel(
        // Position mechanisms → middle of their travel range
        elevator.goToHeight(Meters.of(0.75)),
        profiledElevator.goToHeight(Meters.of(0.75)),
        characterizedElevator.goToHeight(Meters.of(0.75)),
        arm.goToAngle(Degrees.of(90)),
        profiledArm.goToAngle(Degrees.of(90)),
        // Turrets start at mid-range (0°), so aim mid of the positive half instead
        turret.goToAngle(Degrees.of(90)),
        profiledTurret.goToAngle(Degrees.of(90)),
        // Velocity mechanisms → half of the ~6000 RPM free speed
        shooter.goToSpeed(RPM.of(3000)),
        profiledShooter.goToSpeed(RPM.of(3000)),
        jointedArm.goToAngles(Degrees.of(90), Degrees.of(0)),
        wrist.goToAngles(Degrees.of(45), Degrees.of(30))
    ).withName("AllMechanismsToMidpoints"));

    // Releasing X returns everything to its configured start point (read from the
    // settings so the values can't drift from the Example* classes). The onFalse
    // commands share requirements with the midpoint group, so they interrupt it.
    controller.x().onFalse(Commands.parallel(
        elevator.goToHeight(elevator.getSettings().startingHeight),
        profiledElevator.goToHeight(profiledElevator.getSettings().startingHeight),
        characterizedElevator.goToHeight(characterizedElevator.getSettings().startingHeight),
        arm.goToAngle(arm.getSettings().startingAngle),
        profiledArm.goToAngle(profiledArm.getSettings().startingAngle),
        turret.goToAngle(turret.getSettings().startingAngle),
        profiledTurret.goToAngle(profiledTurret.getSettings().startingAngle),
        // Flywheels have no position; "start point" = spun down to rest
        shooter.goToSpeed(RPM.of(0)),
        profiledShooter.goToSpeed(RPM.of(0)),
        jointedArm.goToAngles(
            jointedArm.getSettings().lowerJoint.startingAngle,
            jointedArm.getSettings().upperJoint.startingAngle),
        wrist.goToAngles(
            wrist.getSettings().startingTilt,
            wrist.getSettings().startingTwist)
    ).withName("AllMechanismsToStartPoints"));
  }
}
