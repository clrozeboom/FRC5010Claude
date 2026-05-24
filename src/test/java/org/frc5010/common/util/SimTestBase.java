package org.frc5010.common.util;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

/**
 * Base class for all simulation-based tests (Layers 2 and 3).
 *
 * <p>Handles HAL initialization, DriverStation sim setup, and sim timing
 * so individual tests don't repeat boilerplate. Tests that need sim timing
 * should call stepTiming() or runFor() rather than managing SimHooks directly.
 */
public abstract class SimTestBase {

  /** Loop period matching real robot: 20ms */
  public static final double LOOP_PERIOD_SECS = 0.02;

  @BeforeEach
  public void simSetup() {
    // Initialize the HAL in simulation mode. Returns false if already initialized,
    // which is fine in a multi-test run.
    HAL.initialize(500, 0);

    // Pause sim time so tests control it explicitly via stepTiming().
    // This makes tests deterministic regardless of host machine speed.
    SimHooks.pauseTiming();
    SimHooks.restartTiming();

    // Put DriverStation into a known disabled state before each test.
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @AfterEach
  public void simTeardown() {
    // Cancel all scheduled commands and unregister subsystems so each test
    // starts with a clean CommandScheduler. Subsystem tests create real
    // SubsystemBase instances that register themselves at construction time.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    // Resume timing so other tests or the IDE aren't left in paused state.
    SimHooks.resumeTiming();
  }

  /**
   * Steps simulation time by exactly one robot loop period (20ms).
   * Use this for precise single-cycle assertions.
   */
  protected void stepOneCycle() {
    SimHooks.stepTiming(LOOP_PERIOD_SECS);
    DriverStation.refreshData();
  }

  /**
   * Steps simulation time forward by the given number of seconds,
   * in 20ms increments matching the real robot loop rate.
   *
   * <p>This means a 1-second run executes exactly 50 periodic loops,
   * just like a real robot would.
   *
   * @param seconds how many seconds of sim time to advance
   */
  protected void runFor(double seconds) {
    int cycles = (int) Math.round(seconds / LOOP_PERIOD_SECS);
    for (int i = 0; i < cycles; i++) {
      stepOneCycle();
    }
  }

  /**
   * Enables the simulated DriverStation in teleop mode.
   * Call this before tests that require an enabled robot.
   */
  protected void enableTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  /**
   * Enables the simulated DriverStation in autonomous mode.
   */
  protected void enableAuto() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  /**
   * Disables the simulated DriverStation.
   */
  protected void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }
}