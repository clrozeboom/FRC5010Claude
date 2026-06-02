package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — smoke tests for the refactored {@link RobotContainer} composition architecture.
 *
 * <p>Verifies that:
 * <ul>
 *   <li>{@link RobotContainer} constructs without error in both testSim and default-sim modes</li>
 *   <li>{@link RobotContainer#getAutonomousCommand()} delegates correctly to the inner
 *       {@link RealRobot}, returning {@code null} without {@code -PvisualTest} and a
 *       non-null command with it</li>
 *   <li>{@link RobotContainer#resetToAllianceStart()} delegates without throwing</li>
 *   <li>The {@link DemoIntake} subsystem runs its default command for several enabled cycles
 *       without exception</li>
 * </ul>
 *
 * <p>Each test creates a {@link RobotContainer} (which calls {@code SwerveFactory.build()}
 * internally via {@link org.frc5010.common.profiles.SimRobotProfile} or
 * {@link RealRobotProfile}) and therefore owns an IronMaple {@link SimulatedArena}.
 * The teardown shuts down the arena and resets its singleton via reflection, following
 * the same pattern as {@code AkitSwerveDriveSimPhysicsTest}.
 */
class RobotContainerSmokeTest extends SimTestBase {

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    System.clearProperty("testSim");
    System.clearProperty("visualTest");

    SimulatedArena.getInstance().shutDown();
    try {
      java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("Failed to reset SimulatedArena singleton", e);
    }

    RobotMode.resetForTesting();
    super.simTeardown();
  }

  // ── selectProfile integration ──────────────────────────────────────────────

  @Test
  void testSimModeConstructsWithSimProfile() {
    // -PtestSim → SimRobotProfile (no CTRE CAN IDs, lightweight constants)
    System.setProperty("testSim", "true");
    RobotContainer container = new RobotContainer();
    assertNotNull(container, "RobotContainer must construct without error in testSim mode");
  }

  @Test
  void defaultSimModeConstructsWithRealProfile() {
    // No testSim → RealRobotProfile instantiated reflectively; createDrive() uses SIM branch
    RobotContainer container = new RobotContainer();
    assertNotNull(container, "RobotContainer must construct without error in default sim mode");
  }

  // ── delegation: getAutonomousCommand ──────────────────────────────────────

  @Test
  void getAutonomousCommandNullWithoutVisualTest() {
    System.setProperty("testSim", "true");
    RobotContainer container = new RobotContainer();
    assertNull(container.getAutonomousCommand(),
        "Without -PvisualTest, getAutonomousCommand() must delegate null from SwerveRobotContainer");
  }

  @Test
  void getAutonomousCommandNonNullWithVisualTest() {
    System.setProperty("testSim", "true");
    System.setProperty("visualTest", "true");
    RobotContainer container = new RobotContainer();
    assertNotNull(container.getAutonomousCommand(),
        "With -PvisualTest, getAutonomousCommand() must delegate the SwerveVisualTest sequence");
  }

  // ── delegation: resetToAllianceStart ──────────────────────────────────────

  @Test
  void resetToAllianceStartDelegatesWithoutThrowing() {
    System.setProperty("testSim", "true");
    RobotContainer container = new RobotContainer();
    assertDoesNotThrow(container::resetToAllianceStart,
        "resetToAllianceStart() must delegate to RealRobot without throwing");
  }

  // ── DemoIntake subsystem default command ──────────────────────────────────

  @Test
  void demoIntakeDefaultCommandRunsWhileEnabled() {
    // Construct in testSim mode — DemoIntake.configureBindings() fires because
    // RobotBase.isSimulation() is true, registering DemoIntake with the CommandScheduler
    // and setting "DemoIntakeDefault" as its default command.
    System.setProperty("testSim", "true");
    new RobotContainer();

    enableTeleop();

    // Run five scheduler ticks — DemoIntakeDefault and KeyboardDrive default commands
    // should execute without error. DemoIntake reads SimulatedArena game pieces; no
    // extend/retract/fire inputs are active so no game-piece mutations occur.
    assertDoesNotThrow(() -> {
      for (int i = 0; i < 5; i++) {
        CommandScheduler.getInstance().run();
        stepOneCycle();
      }
    }, "Five enabled scheduler cycles must complete without exception");
  }

  @Test
  void demoIntakeDefaultCommandRunsWhileDisabled() {
    // DemoIntake's default command does not have ignoringDisable(true), so it is
    // cancelled while disabled — verifying that the disabled state does not cause errors.
    System.setProperty("testSim", "true");
    new RobotContainer();

    // Robot stays disabled (SimTestBase default)
    assertDoesNotThrow(() -> {
      for (int i = 0; i < 5; i++) {
        CommandScheduler.getInstance().run();
        stepOneCycle();
      }
    }, "Five disabled scheduler cycles must complete without exception");
  }
}
