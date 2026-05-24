package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 2 — subsystem simulation tests for {@link AkitSwerveDrive}.
 *
 * <p>These tests run the full {@code periodic()} loop with lightweight WPILib
 * {@code DCMotorSim} modules (no YAGSL physics overhead). They verify
 * observable drive behaviour: odometry integration, pose reset, and
 * disabled-state safety — rather than internal implementation details.
 *
 * <p>Timing is controlled deterministically via {@link SimTestBase#stepOneCycle()},
 * so assertions are not sensitive to host machine speed.
 */
class AkitSwerveDriveTest extends SimTestBase {

  /** Default 24 × 24 inch square robot with sim hardware. */
  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.buildWithoutPhysics(CONSTANTS);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    // SimTestBase.simTeardown() already cancels commands and unregisters subsystems.
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  // ---------------------------------------------------------------------------
  // Startup
  // ---------------------------------------------------------------------------

  @Test
  void startsAtOrigin() {
    drive.periodic();
    Pose2d pose = drive.getPose();
    assertEquals(0.0, pose.getX(),                    1e-6, "X starts at 0");
    assertEquals(0.0, pose.getY(),                    1e-6, "Y starts at 0");
    assertEquals(0.0, pose.getRotation().getRadians(), 1e-6, "Heading starts at 0°");
  }

  @Test
  void maxSpeedAccessorsMatchConstants() {
    assertEquals(CONSTANTS.maxLinearSpeedMps,    drive.getMaxLinearSpeedMetersPerSec(), 1e-9);
    assertEquals(CONSTANTS.maxAngularSpeedRadps, drive.getMaxAngularSpeedRadPerSec(),   1e-9);
  }

  // ---------------------------------------------------------------------------
  // Disabled safety
  // ---------------------------------------------------------------------------

  @Test
  void noCommandsWhileDisabledKeepsPoseAtOrigin() {
    // Robot starts disabled (SimTestBase guarantees this). Running periodic()
    // without any velocity commands must leave the pose unchanged.
    for (int i = 0; i < 50; i++) {
      drive.periodic();
      stepOneCycle();
    }
    Pose2d pose = drive.getPose();
    assertEquals(0.0, pose.getX(), 1e-6, "X must stay at 0 while disabled and undriven");
    assertEquals(0.0, pose.getY(), 1e-6, "Y must stay at 0 while disabled and undriven");
  }

  // ---------------------------------------------------------------------------
  // Enabled motion — odometry integration
  // ---------------------------------------------------------------------------

  @Test
  void forwardVelocityAdvancesOdometryX() {
    // 1 m/s forward for ~1 simulated second (50 × 20 ms loops).
    // DCMotorSim takes a few cycles to spin up, so threshold is conservative.
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      drive.periodic();
      stepOneCycle();
    }
    double x = drive.getPose().getX();
    assertTrue(x > 0.1,
        "Forward command for 1 s should advance X by > 0.1 m; got " + x);
  }

  @Test
  void strafeVelocityAdvancesOdometryY() {
    // 1 m/s leftward strafe: positive vy in robot frame = positive Y in field frame
    // (robot starts at heading 0°, so robot and field frames coincide).
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(0.0, 1.0, 0.0));
      drive.periodic();
      stepOneCycle();
    }
    double y = drive.getPose().getY();
    assertTrue(y > 0.1,
        "Strafe command for 1 s should advance Y by > 0.1 m; got " + y);
  }

  @Test
  void rotationVelocityChangesHeading() {
    // π rad/s CCW for ~1 s. Heading is integrated by GyroIOSim from
    // kinematics-derived omega; modules need a few cycles to reach target
    // angle, so the threshold is conservative (> 0.1 rad ≈ 6°).
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, Math.PI));
      drive.periodic();
      stepOneCycle();
    }
    double heading = drive.getRotation().getRadians();
    assertTrue(Math.abs(heading) > 0.1,
        "Rotation command for 1 s should change heading by > 0.1 rad; got " + heading);
  }

  // ---------------------------------------------------------------------------
  // Pose reset
  // ---------------------------------------------------------------------------

  @Test
  void setPosePersistsAcrossDisabledCycles() {
    // After resetting pose, running several disabled periodic() cycles must
    // not corrupt the stored pose (no commands → no movement).
    Pose2d target = new Pose2d(3.5, -2.0, Rotation2d.fromDegrees(90.0));
    drive.setPose(target);

    for (int i = 0; i < 5; i++) {
      drive.periodic();
      stepOneCycle();
    }

    Pose2d actual = drive.getPose();
    assertEquals( 3.5, actual.getX(),                    0.01, "X after setPose + 5 disabled cycles");
    assertEquals(-2.0, actual.getY(),                    0.01, "Y after setPose + 5 disabled cycles");
    assertEquals(90.0, actual.getRotation().getDegrees(), 0.5,  "Heading after setPose + 5 disabled cycles");
  }

  @Test
  void setPoseResetsToOriginMidRun() {
    // Drive forward, then hard-reset to origin. The DCMotorSim keeps its
    // internal velocity after the pose reset, so the robot coasts for a short
    // distance before decelerating to a stop. After waiting long enough for
    // that coast-down (τ ≈ 100 ms → 5 cycles; 50 cycles >> 5τ), the final
    // position should be well under the pre-reset distance.
    enableTeleop();
    for (int i = 0; i < 25; i++) {
      drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      drive.periodic();
      stepOneCycle();
    }
    double xBeforeReset = drive.getPose().getX();
    assertTrue(xBeforeReset > 0.1, "Should have moved before reset; got " + xBeforeReset);

    drive.setPose(Pose2d.kZero);
    disable();

    // Allow full motor deceleration: coast distance ≈ v₀ × τ ≈ 1 m/s × 0.1 s = 0.1 m
    for (int i = 0; i < 50; i++) {
      drive.periodic();
      stepOneCycle();
    }

    Pose2d after = drive.getPose();
    assertTrue(Math.abs(after.getX()) < 0.15,
        "After reset and coast-down, X should be near origin (<0.15 m); got " + after.getX());
    assertEquals(0.0, after.getY(), 0.05, "Y stays near 0 (only forward commands were given)");
  }
}
