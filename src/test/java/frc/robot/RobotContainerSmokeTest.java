package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.example.ExampleRobot;
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
 *       {@link ExampleRobot}, returning the chooser's default ({@code Commands.none()}) without
 *       {@code -PvisualTest} and the {@code SwerveVisualTest} sequence with it</li>
 *   <li>{@link RobotContainer#resetToAllianceStart()} delegates without throwing</li>
 *   <li>The {@link DemoIntake} subsystem runs its default command for several enabled cycles
 *       without exception</li>
 * </ul>
 *
 * <p>Each test creates a {@link RobotContainer} (which calls {@code SwerveFactory.build()}
 * internally via {@link org.frc5010.common.profiles.SimRobotProfile} or
 * {@link ExampleRobotProfile}) and therefore owns an IronMaple {@link SimulatedArena}.
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

    // Free the demo mechanisms' CAN devices (IDs 21–35) — the scheduler teardown
    // doesn't, and stale handles would collide with MechanismsFunctionalTest later
    // in the same JVM.
    org.frc5010.common.profiles.SwerveRobotContainer.closeMechanisms();
    org.frc5010.common.mechanisms.MechanismVisuals3d.resetForTesting();

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
    // No testSim → ExampleRobotProfile instantiated reflectively; createDrive() uses SIM branch
    RobotContainer container = new RobotContainer();
    assertNotNull(container, "RobotContainer must construct without error in default sim mode");
  }

  // ── delegation: getAutonomousCommand ──────────────────────────────────────

  @Test
  void getAutonomousCommandFallsThroughToChooserDefault() {
    System.setProperty("testSim", "true");
    RobotContainer container = new RobotContainer();
    // BuildAutos runs on the first scheduler tick (ignoringDisable), so run one cycle to
    // populate the chooser before asserting. Without -PvisualTest the chooser default
    // (Commands.none()) is returned — the contract is "non-null, no-op".
    CommandScheduler.getInstance().run();
    assertNotNull(container.getAutonomousCommand(),
        "Without -PvisualTest, getAutonomousCommand() must return the chooser's default (Commands.none())");
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
        "resetToAllianceStart() must delegate to ExampleRobot without throwing");
  }

  // ── DemoIntake subsystem periodic ─────────────────────────────────────────

  /**
   * Pumps scheduler + sim time. The container owns TalonFX demo mechanisms whose
   * simulated devices process controls on a real-time thread, so each cycle feeds DS
   * data and sleeps briefly (see docs/mechanisms.md gotcha 6), mirroring
   * MechanismsFunctionalTest's pump.
   */
  private void pumpCycles(int cycles) throws InterruptedException {
    for (int i = 0; i < cycles; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTimingAsync(LOOP_PERIOD_SECS);
      Thread.sleep(10);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
    }
  }

  @Test
  void demoIntakePeriodicRunsWhileEnabled() {
    // Construct in testSim mode — DemoIntake is wired via Trigger bindings (no default
    // command). The scheduler still calls DemoIntake.periodic() each cycle, which calls
    // IntakeSimulation.removeObtainedGamePieces(). No extend/retract/fire inputs are
    // active so no game-piece mutations occur.
    System.setProperty("testSim", "true");
    new RobotContainer();

    enableTeleop();

    assertDoesNotThrow(() -> pumpCycles(5),
        "Five enabled scheduler cycles must complete without exception");
  }

  // ── X button → all mechanisms to midpoints ────────────────────────────────

  @Test
  void xButtonDrivesMechanismsTowardMidpoints() throws InterruptedException {
    // End-to-end binding check (gotcha 11: run the flow, don't just construct): press
    // X on the simulated controller and verify the demo elevator actually climbs
    // toward its 0.75 m midpoint via the AllMechanismsToMidpoints parallel command.
    System.setProperty("testSim", "true");
    new RobotContainer();

    var elevator = ExampleRobot.getDemoElevator().orElseThrow(
        () -> new AssertionError("sim demo mechanisms should exist in simulation"));

    enableTeleop();
    XboxControllerSim controllerSim = new XboxControllerSim(0);
    controllerSim.setXButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    pumpCycles(100); // 2 s of sim time

    double height = elevator.getHeight().in(edu.wpi.first.units.Units.Meters);
    assertTrue(height > 0.3,
        "X button should drive the demo elevator from 0.1 m toward 0.75 m, was " + height);

    // Releasing X must send everything back to its start point (0.1 m for the elevator).
    controllerSim.setXButton(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(150); // 3 s of sim time to descend

    double restored = elevator.getHeight().in(edu.wpi.first.units.Units.Meters);
    assertTrue(restored < 0.3,
        "Releasing X should return the demo elevator toward its 0.1 m start, was " + restored);
  }

  @Test
  void demoIntakePeriodicRunsWhileDisabled() {
    // DemoIntake has no default command; its periodic() runs regardless of enable state
    // because SubsystemBase.periodic() is always called by the scheduler.
    System.setProperty("testSim", "true");
    new RobotContainer();

    // Robot stays disabled (SimTestBase default)
    assertDoesNotThrow(() -> pumpCycles(5),
        "Five disabled scheduler cycles must complete without exception");
  }
}
