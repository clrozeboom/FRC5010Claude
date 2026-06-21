package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import frc.robot.rebuilt.subsystems.RebuiltIntake.IntakeState;
import frc.robot.rebuilt.subsystems.RebuiltLauncher.LauncherState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — smoke + functional tests for {@link RebuiltRobot} (the ported 2026 "Rebuilt"
 * competition robot). Construction in both testSim and default-sim modes, the auto chooser,
 * and the live control flow driven from the simulated driver controller (CLAUDE.md gotcha
 * 11: run the flow, don't just construct).
 *
 * <p>Teardown mirrors {@code ExampleRobotSmokeTest}: free the mechanism CAN handles
 * (hopper 14/15, flywheel 16/17, turret 18, hood 19) and reset the {@link SimulatedArena}
 * singleton so the suite stays clean across tests in the same JVM.
 */
class RebuiltRobotSmokeTest extends SimTestBase {

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

    // Clear button bindings from this test's robot so they don't fire on a later test's
    // controller presses (the CommandScheduler default button loop is a singleton).
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    CommandScheduler.getInstance().cancelAll();
    // Release all controller buttons on both ports: a button left pressed would make the
    // next test's freshly-bound onTrue initialise already-pressed (no rising edge → never fires).
    for (int port = 0; port <= 1; port++) {
      XboxControllerSim c = new XboxControllerSim(port);
      c.setAButton(false);
      c.setBButton(false);
      c.setXButton(false);
      c.setYButton(false);
      c.setLeftBumperButton(false);
      c.setRightBumperButton(false);
      c.setStartButton(false);
      c.setBackButton(false);
    }
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

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

  /**
   * Pumps scheduler + sim time. The launcher/intake mechanisms run onboard MotionMagic /
   * VelocityVoltage, so each cycle feeds DS data AND the Phoenix enable watchdog (and sleeps
   * a few ms) or the simulated TalonFX silently neutrals — see docs/mechanisms.md gotcha 6.
   */
  private void pumpCycles(int cycles) throws InterruptedException {
    for (int i = 0; i < cycles; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTimingAsync(LOOP_PERIOD_SECS);
      Thread.sleep(10); // the simulated TalonFX processes controls on a real-time thread
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
    }
  }

  /** Presses A and pumps until the hopper deploys (clears the turret) and the intake is intaking. */
  private void deployIntake(XboxControllerSim driver) throws InterruptedException {
    driver.setAButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(60);
  }

  // ── construction ───────────────────────────────────────────────────────────

  @Test
  void constructsInTestSimMode() {
    System.setProperty("testSim", "true");
    assertNotNull(new RebuiltRobot(), "RebuiltRobot must construct in testSim mode");
  }

  @Test
  void constructsInDefaultSimMode() {
    assertNotNull(new RebuiltRobot(), "RebuiltRobot must construct in default sim mode");
  }

  // ── autos ──────────────────────────────────────────────────────────────────

  @Test
  void autonomousCommandIsNonNullAfterFirstTick() {
    System.setProperty("testSim", "true");
    RebuiltRobot robot = new RebuiltRobot();
    CommandScheduler.getInstance().run(); // buildAutos runs on first tick
    assertNotNull(robot.getAutonomousCommand(), "chooser default must be available");
  }

  // ── intake state machine ───────────────────────────────────────────────────

  @Test
  void aButtonRequestsIntakingAndDeploysTheHopper() throws InterruptedException {
    RebuiltRobot robot = new RebuiltRobot();
    var intake = robot.getIntake();
    assertNotNull(intake, "intake exists in sim");

    enableTeleop();
    XboxControllerSim driver = new XboxControllerSim(0);
    driver.setAButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    pumpCycles(50); // > the 0.6 s hopper settle time

    assertEquals(IntakeState.INTAKING, intake.getRequestedState(), "A requests INTAKING");
    assertEquals(IntakeState.INTAKING, intake.getCurrentState(),
        "intake completes the deploy transit and reaches INTAKING");
    assertTrue(
        intake.getHopper().getAngleDegrees() < 120,
        "hopper should drive down from its 120° retracted start, was "
            + intake.getHopper().getAngleDegrees());
    assertTrue(intake.isCollecting(), "spintakes run once deployed and intaking");
  }

  // ── launcher state machine ─────────────────────────────────────────────────

  @Test
  void bButtonEntersPrepThenLowSpeedOnRelease() throws InterruptedException {
    RebuiltRobot robot = new RebuiltRobot();
    var launcher = robot.getLauncher();

    enableTeleop();
    XboxControllerSim driver = new XboxControllerSim(0);
    // The hopper arm blocks the turret while retracted, so the launcher is stowed in
    // HAMMERTIME until the intake deploys clear (faithful intake↔turret interference rule).
    deployIntake(driver);
    assertEquals(IntakeState.INTAKING, robot.getIntake().getCurrentState(), "intake deployed clear");

    driver.setBButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(5);
    assertEquals(LauncherState.PREP, launcher.getState(), "B holds launcher in PREP once clear");

    driver.setBButton(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(5);
    assertEquals(LauncherState.LOW_SPEED, launcher.getState(), "releasing B drops to LOW_SPEED");
  }

  @Test
  void launcherIsStowedWhileIntakeRetracted() throws InterruptedException {
    // Faithful interference: pressing B while the hopper is retracted keeps the launcher in
    // HAMMERTIME (the hopper arm blocks the turret) — it does not enter PREP.
    RebuiltRobot robot = new RebuiltRobot();
    var launcher = robot.getLauncher();
    enableTeleop();
    XboxControllerSim driver = new XboxControllerSim(0);
    driver.setBButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(5);
    assertEquals(
        LauncherState.HAMMERTIME, launcher.getState(), "launcher stows while hopper blocks turret");
  }

  // ── hopper retract via X ───────────────────────────────────────────────────

  /**
   * X is now the retract button (mirrors Y): A deploys to 0°, X retracts to 120°.
   * Firing is automatic via the coupling loop when the launcher is at goal.
   */
  @Test
  void xButtonRetractsHopperAfterDeploy() throws InterruptedException {
    RebuiltRobot robot = new RebuiltRobot();
    var intake = robot.getIntake();

    enableTeleop();
    XboxControllerSim driver = new XboxControllerSim(0);

    // Deploy with A first.
    driver.setAButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(60); // wait for hopper to reach 0° (0.6 s settle + motion time)
    assertEquals(IntakeState.INTAKING, intake.getCurrentState(), "A deploys the hopper");

    // Release A, press X to retract.
    driver.setAButton(false);
    driver.setXButton(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    pumpCycles(60); // wait for hopper to return to 120°

    assertEquals(IntakeState.RETRACTED, intake.getCurrentState(), "X retracts the hopper");
    assertTrue(
        intake.getHopper().getAngleDegrees() >= 117,
        "hopper should return to retracted position (~120°), was "
            + intake.getHopper().getAngleDegrees());
  }
}
