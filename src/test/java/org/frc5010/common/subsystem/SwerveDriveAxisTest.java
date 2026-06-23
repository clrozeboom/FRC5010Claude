package org.frc5010.common.subsystem;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.profiles.SwerveRobotContainer;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Pins the keyboard-vs-Xbox rotation-axis selection in
 * {@link SwerveRobotContainer}'s drive default command.
 *
 * <p>On a real Xbox pad axis 2 is the <b>left trigger</b> and rotation is the right-stick X
 * (axis 4). The container picks the rotation axis at runtime by axis count (a Glass keyboard
 * joystick has 3 axes; an Xbox pad has 6). A {@link XboxControllerSim} reports 6 axes, so these
 * tests exercise the Xbox branch: the right stick rotates, the left trigger does not.
 */
class SwerveDriveAxisTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .trackWidth(Inches.of(22.75))
          .wheelBase(Inches.of(22.75))
          .wheelRadius(Inches.of(2.0))
          .build();

  /** Minimal container exposing the drive so the test can read heading. */
  private static final class TestContainer extends SwerveRobotContainer {
    TestContainer(AkitSwerveDrive drive) {
      super(drive);
    }

    AkitSwerveDrive drive() {
      return drive;
    }

    @Override
    protected Pose2d getBlueAllianceStartPose() {
      return Pose2d.kZero;
    }
  }

  private AkitSwerveDrive drive;
  private TestContainer container;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS);
    container = new TestContainer(drive);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    CommandScheduler.getInstance().cancelAll();
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  private void runCycles(int n) {
    for (int i = 0; i < n; i++) {
      CommandScheduler.getInstance().run();
      stepOneCycle();
    }
  }

  private double headingMagnitudeRad() {
    return Math.abs(drive.getPose().getRotation().getRadians());
  }

  /** Zeros all drive-relevant axes (sim joystick state persists across tests in the JVM). */
  private static void zeroAxes(XboxControllerSim driver) {
    driver.setLeftX(0);
    driver.setLeftY(0);
    driver.setRightX(0);
    driver.setRightY(0);
    driver.setLeftTriggerAxis(0);
    driver.setRightTriggerAxis(0);
  }

  @Test
  void rightStickXRotatesTheRobot() {
    enableTeleop();
    XboxControllerSim driver = new XboxControllerSim(0);
    zeroAxes(driver);
    driver.setRightX(0.9); // axis 4
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    runCycles(40);

    assertTrue(
        headingMagnitudeRad() > 0.1,
        "right-stick X (axis 4) must rotate the robot, heading was " + headingMagnitudeRad());
  }

  @Test
  void leftTriggerDoesNotRotateTheRobot() {
    enableTeleop();
    XboxControllerSim driver = new XboxControllerSim(0);
    zeroAxes(driver);
    // On an Xbox pad axis 2 is the left trigger — it must NOT be read as rotation.
    driver.setLeftTriggerAxis(0.9);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    runCycles(40);

    assertTrue(
        headingMagnitudeRad() < 0.02,
        "left trigger (axis 2) must not rotate an Xbox-driven robot, heading was "
            + headingMagnitudeRad());
  }
}
