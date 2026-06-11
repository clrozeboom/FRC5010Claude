package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.*;

/**
 * Team-specific regression tests for the drive subsystem.
 *
 * <p>Uses the real robot's geometry constants so any mismatch between
 * SwerveConstants and actual kinematics shows up here rather than on the field.
 *
 * <p>Add tests here when you find a field bug that should never come back.
 */
class DriveRegressionTest extends SimTestBase {

  // Match ExampleRobotProfile's constants exactly — tests are meaningless if
  // they use different geometry than the deployed code.
  private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
      .moduleType(ModuleType.SIM)
      .gyroType(GyroType.SIM)
      .trackWidth(Inches.of(22.75))
      .wheelBase(Inches.of(22.75))
      .wheelRadius(Inches.of(2.0))
      .build();

  private AkitSwerveDrive drive;

  @BeforeEach @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS);
  }

  @AfterEach @Override
  public void simTeardown() {
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  @Test
  void forwardCommandProducesPositiveXDisplacement() {
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(1.0, 0, 0));
      drive.periodic();
      stepOneCycle();
    }
    assertTrue(drive.getPose().getX() > 0.1,
        "1 m/s forward for 1 s should displace > 0.1 m in X");
  }

  @Test
  void maxSpeedAccessorMatchesConstants() {
    assertEquals(
        CONSTANTS.maxLinearSpeed.in(edu.wpi.first.units.Units.MetersPerSecond),
        drive.getMaxLinearSpeed().in(edu.wpi.first.units.Units.MetersPerSecond),
        1e-9,
        "getMaxLinearSpeed() must return the configured constant");
  }

  @Test
  void disabledRobotDoesNotMove() {
    // disabled() is the default from SimTestBase — do not call enableTeleop()
    // Do not call runVelocity(): periodic() while disabled calls module.stop(),
    // but DCMotorSim integrates one cycle of any previously set velocity (coasting).
    // This test verifies that an undriven disabled robot stays at origin.
    for (int i = 0; i < 10; i++) {
      drive.periodic();
      stepOneCycle();
    }
    assertEquals(0.0, drive.getPose().getX(), 1e-6,
        "Undriven disabled robot must not move from origin");
  }
}
