package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.rebuilt.subsystems.SmartTurretConfig;
import frc.robot.rebuilt.subsystems.SmartTurretController;
import frc.robot.rebuilt.subsystems.SmartTurretController.TurretState;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Functional tests for {@link SmartTurretController} — the torque-current-FOC turret loop.
 * Drives {@code step()} directly at 200 Hz (no Notifier) for determinism, feeding the Phoenix
 * enable watchdog each tick so the simulated TalonFX actuates (see docs/mechanisms.md gotcha 6).
 *
 * <p>Verifies the two things that matter for fast moving-target tracking: it seeks to a target
 * and settles into TRACKING, and it tracks a continuously moving target with low lag thanks to
 * the velocity feedforward.
 */
class SmartTurretControllerTest extends SimTestBase {

  private SmartTurretController turret;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    SmartTurretConfig c = new SmartTurretConfig();
    c.canId = 18;
    c.canBus = ""; // rio bus in sim
    c.motorModel = DCMotor.getKrakenX60(1);
    turret = new SmartTurretController(c);
    enableTeleop();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @AfterEach
  @Override
  public void simTeardown() {
    turret.close();
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  /** One 5 ms control tick with the Phoenix enable fed + a little real time for the device thread. */
  private void tick() throws InterruptedException {
    DriverStationSim.notifyNewData();
    Unmanaged.feedEnable(100);
    turret.step(0.005);
    Thread.sleep(2);
    SimHooks.stepTiming(0.005);
    DriverStation.refreshData();
  }

  private double actualDeg() {
    return turret.getActualPositionRot() * 360.0;
  }

  @Test
  void seeksToTargetAndSettlesIntoTracking() throws InterruptedException {
    for (int i = 0; i < 400; i++) { // 2 s at 200 Hz
      turret.setTarget(Degrees.of(45), 0, 0);
      tick();
    }
    assertEquals(45.0, actualDeg(), 5.0, "turret should reach the 45° target");
    assertEquals(TurretState.TRACKING, turret.getState(), "settled within threshold → TRACKING");
  }

  @Test
  void respectsSoftLimit() throws InterruptedException {
    // Command well beyond the +150° soft limit; the turret must clamp at the limit.
    for (int i = 0; i < 500; i++) {
      turret.setTarget(Degrees.of(300), 0, 0);
      tick();
    }
    assertTrue(actualDeg() <= 152.0, "turret must not pass the +150° soft limit, was " + actualDeg());
  }

  @Test
  void tracksAMovingTargetWithLowLag() throws InterruptedException {
    // First seek to the start so we measure tracking lag, not the initial catch-up.
    for (int i = 0; i < 200; i++) {
      turret.setTarget(Degrees.of(0), 0, 0);
      tick();
    }

    double rateDegPerSec = 40.0;
    double velRadPerSec = Math.toRadians(rateDegPerSec);
    double targetDeg = 0.0;
    double maxLagDeg = 0.0;
    for (int i = 0; i < 300; i++) { // 1.5 s of moving target
      targetDeg += rateDegPerSec * 0.005;
      turret.setTarget(Degrees.of(targetDeg), velRadPerSec, 0.0);
      tick();
      if (i > 60) { // skip the first 0.3 s of settling
        maxLagDeg = Math.max(maxLagDeg, Math.abs(targetDeg - actualDeg()));
      }
    }
    assertTrue(
        maxLagDeg < 6.0,
        "velocity feedforward should keep moving-target lag small (< 6°), was " + maxLagDeg);
  }
}
