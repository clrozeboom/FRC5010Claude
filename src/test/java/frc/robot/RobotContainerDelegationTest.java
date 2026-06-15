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
 * Layer 3 — pins {@link RobotContainer}'s own construct-and-delegate contract,
 * independent of which {@link org.frc5010.common.profiles.SwerveRobotContainer} subclass
 * it currently builds. The demo composition itself ({@link org.frc5010.examples.ExampleRobot}'s
 * DemoIntake + 11 mechanisms + X-button bindings) is covered by {@code ExampleRobotSmokeTest}.
 *
 * <p>Teardown mirrors {@code ExampleRobotSmokeTest}'s so the file works whether
 * {@code RobotContainer} owns CAN-21–35 demo mechanisms (ExampleRobot) or different CAN
 * handles (a future swap).
 */
class RobotContainerDelegationTest extends SimTestBase {

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

    // Free any mechanism CAN handles the delegate registered — same teardown ordering as
    // ExampleRobotSmokeTest so the suite stays clean regardless of which subclass owns
    // the handles.
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

  @Test
  void robotContainerConstructsWithoutThrowing() {
    System.setProperty("testSim", "true");
    RobotContainer container = new RobotContainer();
    assertNotNull(container, "RobotContainer must construct without error in testSim mode");
  }

  @Test
  void getAutonomousCommandReturnsNonNullAfterFirstTick() {
    System.setProperty("testSim", "true");
    RobotContainer container = new RobotContainer();
    // BuildAutos runs on the first scheduler tick (ignoringDisable), so step once to
    // populate the chooser before asserting.
    CommandScheduler.getInstance().run();
    assertNotNull(container.getAutonomousCommand(),
        "RobotContainer.getAutonomousCommand() must delegate to a non-null command");
  }
}
