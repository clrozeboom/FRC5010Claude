package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.Supplier;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import frc.robot.rebuilt.subsystems.RebuiltIndexer;
import frc.robot.rebuilt.subsystems.RebuiltIntake;
import frc.robot.rebuilt.subsystems.RebuiltLauncher;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — runs a real multi-path {@link RebuiltAutoRoutines} auto end to end under IronMaple
 * physics (drive + the three ported subsystems + the paths' embedded event markers), to confirm
 * the tuned BLine following drives the whole routine cleanly rather than looping.
 */
class RebuiltAutoEndToEndSimPhysicsTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder().moduleType(ModuleType.SIM).gyroType(GyroType.SIM).build();

  private AkitSwerveDrive drive;
  private RebuiltIntake intake;
  private RebuiltLauncher launcher;
  private RebuiltIndexer indexer;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.build(CONSTANTS, new Pose2d(4, 1, Rotation2d.kZero));
    Supplier<Pose2d> pose = drive::getPose;
    Supplier<ChassisSpeeds> fieldVel =
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getPose().getRotation());
    intake = new RebuiltIntake(pose);
    indexer = new RebuiltIndexer();
    launcher = new RebuiltLauncher(pose, fieldVel);
    launcher.setTurretBlocked(intake::isBlockingTurret);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    CommandScheduler.getInstance().cancelAll();
    intake.close();
    indexer.close();
    launcher.close();
    org.frc5010.common.mechanisms.MechanismVisuals3d.resetForTesting();
    SimulatedArena.getInstance().shutDown();
    try {
      java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException(e);
    }
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  private void step() {
    CommandScheduler.getInstance().run();
    drive.simulationPeriodic();
    stepOneCycle();
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @Test
  void right5010DoubleDrivesTheWholeRoutineWithoutLooping() {
    RebuiltAutoRoutines auto = new RebuiltAutoRoutines(drive, intake, launcher, indexer);
    Command routine = auto.right5010DoubleShort();

    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    CommandScheduler.getInstance().schedule(routine);

    // Record the path-relative start (after the first path's pose reset settles).
    for (int i = 0; i < 3; i++) step();
    Pose2d start = drive.getPose();

    double traveled = 0.0;
    double maxFromStart = 0.0;
    Pose2d prev = drive.getPose();
    boolean intakeReachedIntaking = false;
    boolean launcherLeftHammertime = false;
    int c = 0;
    while (c < 1600 && routine.isScheduled()) {
      step();
      Pose2d p = drive.getPose();
      assertFalse(Double.isNaN(p.getX()) || Double.isNaN(p.getY()), "pose must never go NaN");
      traveled += p.getTranslation().getDistance(prev.getTranslation());
      maxFromStart = Math.max(maxFromStart, p.getTranslation().getDistance(start.getTranslation()));
      prev = p;
      if (intake.getCurrentState() == RebuiltIntake.IntakeState.INTAKING) {
        intakeReachedIntaking = true;
      }
      if (launcher.getState() != RebuiltLauncher.LauncherState.HAMMERTIME) {
        launcherLeftHammertime = true;
      }
      c++;
    }

    System.out.println(
        ">> right5010DoubleShort: cycles=" + c + " finished=" + !routine.isScheduled()
            + " maxFromStart=" + maxFromStart + " traveled=" + traveled
            + " intakeState=" + intake.getCurrentState()
            + " intakeReachedIntaking=" + intakeReachedIntaking
            + " launcherState=" + launcher.getState()
            + " launcherLeftHammertime=" + launcherLeftHammertime);
    assertTrue(c < 1600, "the auto routine must finish, ran " + c);
    // The routine drives out and back (~two ~5 m legs); it must cover real ground...
    assertTrue(maxFromStart > 3.0, "robot must actually drive the routine; maxFromStart=" + maxFromStart);
    // ...without the dense-sampling looping. This 3-leg routine is ~18-19 m of path arc; tuned
    // following covers it in ~32 m (≈1.7×), whereas the dense-sampling regression weaved to ~70 m+.
    assertTrue(
        traveled < 45.0,
        "tuned following must not loop across the routine; traveled=" + traveled + " m");
    // The intakeIntake path event must fire, settling the hopper to INTAKING within 0.6 s.
    assertTrue(
        intakeReachedIntaking,
        "intakeIntake event must deploy the hopper and settle to INTAKING; final=" + intake.getCurrentState());
    // The intakeIntake event unblocks the turret (DEPLOYING is not blocking); launcherPrep/Low must
    // then push the launcher out of HAMMERTIME before the auto ends.
    assertTrue(
        launcherLeftHammertime,
        "auto must command the launcher out of HAMMERTIME once turret is unblocked; final=" + launcher.getState());
    // The last command in the sequence is launcherPrep() — launcher must end in PREP.
    assertEquals(
        RebuiltLauncher.LauncherState.PREP,
        launcher.getState(),
        "auto must end with launcher in PREP (launcherPrep() is the last command); final=" + launcher.getState());
  }
}
