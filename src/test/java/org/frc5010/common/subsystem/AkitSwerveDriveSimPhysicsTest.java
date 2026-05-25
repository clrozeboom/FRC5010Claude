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
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — integration/physics tests for {@link AkitSwerveDrive} backed by a full
 * IronMaple {@link SimulatedArena} physics world.
 *
 * <p>These tests use {@link SwerveFactory#build} (which creates a
 * {@link swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation} and registers
 * it with {@link SimulatedArena}) and verify two things:
 * <ol>
 *   <li><strong>Contract</strong>: the physics engine must be explicitly advanced via
 *       {@link AkitSwerveDrive#simulationPeriodic()} for any motion to occur. Commanding a
 *       velocity and calling only {@code periodic()} leaves the pose unchanged — IronMaple
 *       module caches are pre-filled with initial values (zero position) and are only refreshed
 *       by sub-ticks inside {@code simulationPeriodic()}.
 *   <li><strong>Behaviour</strong>: once the physics engine is advanced correctly, forward,
 *       strafe, and rotation commands all produce physically plausible motion.
 * </ol>
 *
 * <p>The correct per-cycle call order is:
 * <pre>
 *   drive.runVelocity(speeds);   // queue commands to physics controllers
 *   drive.simulationPeriodic();  // advance IronMaple dyn4j world (5 × 4 ms sub-ticks)
 *   drive.periodic();            // read updated physics state → odometry
 *   stepOneCycle();              // advance FPGA time by 20 ms
 * </pre>
 *
 * <p>The {@link SimulatedArena} singleton is reset between tests via
 * {@link SimulatedArena#shutDown()} and reflection so each test starts with a clean physics world.
 */
class AkitSwerveDriveSimPhysicsTest extends SimTestBase {

  /** Default 24 × 24 inch square robot; SIM module/gyro types trigger IronMaple in SwerveFactory. */
  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  /**
   * Physics spawn position — well inside the field so the dyn4j constraint solver starts
   * from a clean state with no wall penetration at (0,0).  Starting in the corner pins the
   * body against two walls simultaneously; strafing would not advance the physics body even
   * with correct motor commands, producing false-negative test results under physics injection.
   */
  private static final Pose2d SPAWN = new Pose2d(2.0, 2.0, new Rotation2d());

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    // build() creates SwerveDriveSimulation and registers it with SimulatedArena.getInstance()
    drive = SwerveFactory.build(CONSTANTS, SPAWN);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    // 1. Remove all physics bodies from the arena's dyn4j world.
    SimulatedArena.getInstance().shutDown();

    // 2. Null the singleton via reflection so the next test starts with a fresh
    //    Arena2026Rebuilt instance rather than inheriting this test's state.
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

  // ---------------------------------------------------------------------------
  // Helper
  // ---------------------------------------------------------------------------

  /**
   * Advances one full robot loop with IronMaple physics engaged:
   * <ol>
   *   <li>Advances the physics engine 5 sub-ticks of 4 ms each
   *       ({@link AkitSwerveDrive#simulationPeriodic()})
   *   <li>Reads the updated physics state into odometry
   *       ({@link AkitSwerveDrive#periodic()})
   *   <li>Steps FPGA time by 20 ms ({@link #stepOneCycle()})
   * </ol>
   * Physics <em>must</em> advance before {@code periodic()} so that the module caches
   * contain fresh positions rather than the stale initial values.
   */
  private void step() {
    drive.simulationPeriodic(); // advance dyn4j world (5 × 4 ms)
    drive.periodic();           // read results into pose estimator
    stepOneCycle();             // tick FPGA clock forward 20 ms
  }

  // ---------------------------------------------------------------------------
  // Startup
  // ---------------------------------------------------------------------------

  @Test
  void buildWithPhysicsSucceeds() {
    // Smoke test: SwerveFactory.build() in SIM mode with SIM module/gyro types
    // must produce a non-null subsystem without throwing.
    assertNotNull(drive, "SwerveFactory.build() should return a non-null AkitSwerveDrive");
  }

  @Test
  void initialPoseIsAtSpawn() {
    // With physics pose injection the estimator converges from its initial state (odometry
    // seeded at origin) toward the physics spawn position within a few idle cycles.
    // Run 5 cycles to allow the Kalman filter to settle before asserting.
    for (int i = 0; i < 5; i++) step();
    Pose2d pose = drive.getPose();
    assertEquals(SPAWN.getX(), pose.getX(),                    0.01, "X settles to spawn X");
    assertEquals(SPAWN.getY(), pose.getY(),                    0.01, "Y settles to spawn Y");
    assertEquals(0.0,          pose.getRotation().getRadians(), 1e-4, "Heading starts at 0°");
  }

  // ---------------------------------------------------------------------------
  // IronMaple contract: simulationPeriodic() is required for any motion
  // ---------------------------------------------------------------------------

  /**
   * Without {@link AkitSwerveDrive#simulationPeriodic()}, the IronMaple physics engine
   * never advances.
   *
   * <p>The IronMaple {@link swervelib.simulation.ironmaple.simulation.drivesims.SwerveModuleSimulation}
   * pre-fills its position caches with the initial value (zero) at construction time.
   * Those caches are only refreshed by sub-ticks inside {@code simulationPeriodic()}.
   * Calling only {@code periodic()} therefore reads the same stale zeros every loop,
   * so the pose estimator never integrates any displacement.
   *
   * <p>This behaviour distinguishes Layer 3 from Layer 2: a Layer 2 DCMotorSim drive
   * advances its own internal state whenever {@code periodic()} calls
   * {@code driveSim.update(dt)}, but an IronMaple physics drive does not.
   */
  @Test
  void physicsMotionRequiresSimulationPeriodic() {
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      // Intentionally skip drive.simulationPeriodic() — physics must not advance
      drive.periodic();
      stepOneCycle();
    }
    double x = drive.getPose().getX();
    // With physics injection, getPose() tracks the physics body. Without simulationPeriodic()
    // the physics body stays at SPAWN.X — the injection holds the estimator there rather than
    // letting wheel-odometry drift forward.  The key invariant is: no forward displacement.
    assertEquals(
        SPAWN.getX(), x, 0.05,
        "Without simulationPeriodic() physics body stays at spawn; X must not advance beyond "
            + "spawn (" + SPAWN.getX() + "), got " + x);
  }

  // ---------------------------------------------------------------------------
  // Enabled motion — physics-driven odometry
  // ---------------------------------------------------------------------------

  @Test
  void forwardVelocityAdvancesOdometryX() {
    // 1 m/s forward for ~1 simulated second (50 × 20 ms loops).
    // A 45 kg robot with Falcon 500 / Mark4 swerve has inertia, so the threshold
    // is conservative: the robot must have travelled more than 0.1 m in 1 s.
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      step();
    }
    double x = drive.getPose().getX();
    assertTrue(
        x > SPAWN.getX() + 0.1,
        "Physics forward command for 1 s should advance X > spawn + 0.1 m; got " + x);
  }

  @Test
  void strafeVelocityAdvancesOdometryY() {
    // 1 m/s leftward strafe: positive vy in robot frame → positive Y in field frame
    // (robot starts at heading 0°, so robot and field frames coincide).
    // Threshold is more conservative than the forward case (0.05 m vs 0.1 m) because
    // the modules start facing forward and must rotate 90° before contributing to
    // lateral motion, consuming several cycles of the 1 s window.
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(0.0, 1.0, 0.0));
      step();
    }
    double y = drive.getPose().getY();
    assertTrue(
        y > SPAWN.getY() + 0.05,
        "Physics strafe command for 1 s should advance Y > spawn + 0.05 m; got " + y);
  }

  @Test
  void rotationChangesHeadingWithPhysics() {
    // π rad/s CCW for ~1 simulated second. GyroIOSimPhysics reads heading directly
    // from the IronMaple GyroSimulation driven by the physics engine. Conservative
    // threshold: > 0.05 rad (≈ 3°) to account for module steering spin-up time.
    enableTeleop();
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, Math.PI));
      step();
    }
    double heading = drive.getRotation().getRadians();
    assertTrue(
        Math.abs(heading) > 0.05,
        "Physics rotation command for 1 s should change heading > 0.05 rad; got " + heading);
  }

  // ---------------------------------------------------------------------------
  // Pose reset with physics
  // ---------------------------------------------------------------------------

  @Test
  void setPoseResetsOdometryEstimatorTracksPhysics() {
    // Drive forward for 25 cycles to establish displacement, then hard-reset the
    // odometry estimator to origin.  Because AkitSwerveDrive injects the IronMaple
    // physics body pose as a near-perfect "vision" measurement every simulationPeriodic()
    // cycle, the estimator immediately starts re-tracking the physics body position
    // (which was NOT moved by setPose()).  After 50 coast-down cycles the estimated
    // pose should be close to the physics body's true position, NOT near origin.
    enableTeleop();
    for (int i = 0; i < 25; i++) {
      drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
      step();
    }
    assertTrue(drive.getPose().getX() > SPAWN.getX() + 0.1, "Should have moved before reset");

    drive.setPose(Pose2d.kZero);
    disable();

    // 50 coast-down cycles (>> motor time constant). The physics body decelerates;
    // the physics injection pulls the estimator toward the physics body truth.
    for (int i = 0; i < 50; i++) {
      step();
    }

    Pose2d physPose = drive.getSimulatedPose().orElseThrow();
    Pose2d estPose  = drive.getPose();
    double err = physPose.getTranslation().getDistance(estPose.getTranslation());
    assertTrue(
        err < 0.15,
        "After reset and coast-down, estimator should track physics body within 0.15 m; "
            + "physics=" + physPose + " estimated=" + estPose + " err=" + err);
  }
}
