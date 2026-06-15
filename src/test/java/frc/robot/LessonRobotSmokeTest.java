package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FuelHandler;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — smoke tests for {@link LessonRobot} and its {@link FuelHandler}.
 *
 * <p>Verifies the lesson robot constructs, runs its subsystem periodics cleanly, deploys the
 * intake arm, and launches Fuel from the shooter. Mirrors {@code ExampleRobotSmokeTest}'s
 * pump and {@code SimulatedArena} teardown (the lesson robot owns TalonFX mechanisms on
 * CAN 41–43 and an IronMaple arena).
 */
class LessonRobotSmokeTest extends SimTestBase {

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

    // Free the FuelHandler's CAN devices (41–43) and the LED PWM port; the scheduler teardown
    // doesn't, and stale handles would collide with later tests in the same JVM.
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

  /** Feeds DS data and steps sim time each cycle (TalonFX sim runs on a real-time thread). */
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
  void constructsInTestSim() {
    System.setProperty("testSim", "true");
    LessonRobot robot = new LessonRobot();
    assertNotNull(robot, "LessonRobot must construct without error in testSim mode");
    assertNotNull(robot.getFuelHandler(), "FuelHandler should exist in simulation");
  }

  @Test
  void periodicsRunWhileEnabledAndDisabled() throws InterruptedException {
    System.setProperty("testSim", "true");
    new LessonRobot();

    assertDoesNotThrow(() -> pumpCycles(5), "Five disabled cycles must run without exception");
    enableTeleop();
    assertDoesNotThrow(() -> pumpCycles(5), "Five enabled cycles must run without exception");
  }

  @Test
  void deployExtendsTheIntakeAndRaisesTheArm() throws InterruptedException {
    System.setProperty("testSim", "true");
    LessonRobot robot = new LessonRobot();
    FuelHandler fuel = robot.getFuelHandler();

    enableTeleop();
    CommandScheduler.getInstance().schedule(fuel.deployCommand());
    pumpCycles(100); // 2 s for the LQR arm to swing out

    assertTrue(fuel.isIntakeExtended(), "Deploy should mark the intake extended");
    double armDeg = fuel.getArmAngle().in(Units.Degrees);
    double deployDeg = frc.robot.subsystems.IntakeArm.DEPLOY_ANGLE.in(Units.Degrees);
    assertTrue(armDeg > 40,
        "Deploy should swing the arm toward " + deployDeg + "°, was " + armDeg);
  }

  @Test
  void scoringLaunchesAHeldFuel() throws InterruptedException {
    System.setProperty("testSim", "true");
    LessonRobot robot = new LessonRobot();
    FuelHandler fuel = robot.getFuelHandler();

    enableTeleop();
    int before = fuel.getHeldFuel();
    assertTrue(before > 0, "Robot should start with preloaded Fuel");

    // Spin up the shooter, wait for speed, fire one piece.
    CommandScheduler.getInstance().schedule(fuel.scoreCommand());
    pumpCycles(200); // 4 s: reach speed, then launch

    assertTrue(fuel.getHeldFuel() < before,
        "Scoring should launch one Fuel (held " + before + " → " + fuel.getHeldFuel() + ")");
  }
}
