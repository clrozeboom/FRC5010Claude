package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.auto.BLineSwerveAuto;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — integration test that proves BLine's {@link FollowPath} drives an
 * IronMaple physics-backed {@link AkitSwerveDrive} from a start pose to a goal pose
 * within tolerance.
 *
 * <p>Mirrors the call order and {@link SimulatedArena} teardown pattern of
 * {@link AkitSwerveDriveSimPhysicsTest}. The {@link CommandScheduler} drives the
 * {@code FollowPath} command's lifecycle ({@code initialize} → {@code execute} →
 * {@code isFinished}); each scheduler tick calls {@code drive.runVelocity(...)},
 * after which {@code simulationPeriodic} → {@code periodic} → {@code stepOneCycle}
 * advances the physics world and reads it back into odometry.
 */
class BLineFollowPathSimPhysicsTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private static final Pose2d SPAWN = new Pose2d(2.0, 2.0, new Rotation2d());
  private static final Translation2d GOAL = new Translation2d(4.0, 2.0);

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.build(CONSTANTS, SPAWN);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    try {
      java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("Failed to reset SimulatedArena singleton between tests", e);
    }
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  private void step() {
    drive.simulationPeriodic();
    drive.periodic();
    stepOneCycle();
  }

  @Test
  void builderWiresSuccessfully() {
    FollowPath.Builder b = BLineSwerveAuto.builder(drive);
    assertNotNull(b, "BLineSwerveAuto.builder() must produce a non-null FollowPath.Builder");
  }

  @Test
  void physicsFollowsCodeDefinedPath_endsNearGoal() {
    FollowPath.Builder b = BLineSwerveAuto.builder(drive);

    // 2 m forward from spawn — short enough for path completion in <4 simulated seconds,
    // long enough to make BLine actually run translation control rather than terminating
    // on the end-translation tolerance at t=0.
    Path path = new Path(
        new Path.Waypoint(SPAWN.getTranslation(), SPAWN.getRotation()),
        new Path.Waypoint(GOAL, Rotation2d.fromDegrees(0)));

    Command followPath = b.build(path);
    enableTeleop();
    CommandScheduler.getInstance().schedule(followPath);

    // Spin the scheduler until the path reports finished, or 5 simulated seconds elapse.
    // 250 cycles × 20 ms = 5 s, which is generous for 2 m at the default 4.5 m/s cap.
    int maxCycles = 250;
    int cyclesRun = 0;
    while (cyclesRun < maxCycles && followPath.isScheduled()) {
      CommandScheduler.getInstance().run();
      step();
      cyclesRun++;
    }
    assertTrue(cyclesRun < maxCycles, "FollowPath did not finish within 5 simulated seconds");

    // Coast cycles: BLine's isFinished() fires when the translation controller is at
    // setpoint (end_translation_tolerance_meters = 5 cm), but the physics body still has
    // momentum from the approach. Run a half-second of coast cycles with no scheduled
    // command so module braking can settle the body.
    for (int i = 0; i < 25; i++) step();

    Pose2d finalPose = drive.getPose();
    double distToGoal = finalPose.getTranslation().getDistance(GOAL);
    assertTrue(
        distToGoal < 0.30,
        "After " + cyclesRun + " follow cycles + 25 coast cycles, BLine should have driven "
            + "within 0.30 m of goal " + GOAL + "; got pose=" + finalPose
            + " distToGoal=" + distToGoal);
  }
}
